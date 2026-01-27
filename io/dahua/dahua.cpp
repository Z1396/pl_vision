#include "dahua.hpp"
#include <libusb-1.0/libusb.h>
#include "tools/logger.hpp"
// 引入大华SDK头文件（根据SDK安装目录调整路径）
#include "GenICam/System.h"
#include "GenICam/Camera.h"
#include "GenICam/StreamSource.h"
#include "GenICam/Defs.h"  
#include "GenICam/Frame.h"
#include "GenICam/PixelType.h"
#include "GenICam/ParameterNode.h"
#include "Media/ImageConvert.h"



using namespace std::chrono_literals;
// 引入大华SDK命名空间（简化代码）
using namespace Dahua::GenICam;
using namespace Dahua::Infra;

#define CameraExposureTime 20000//曝光值
#define GainRawValue 1//增益值
#define Gamma 1//伽马值
/*****shark:打什么颜色的队伍就把相应的增益调高，把己方颜色增益降低(正常显示颜色的参数：R（1.84），G（1），B（2.42）)*/
#define dRED 1.52//红平衡值
#define dGREEN 1.1//绿平衡值
#define dBLUE 1.52//蓝平衡值

namespace io
{
/**
 * @brief 大华相机类的构造函数，初始化相机参数并启动守护线程
 * @param exposure_ms 曝光时间（毫秒）
 * @param gain 增益值
 * @param vid_pid 相机的VID:PID字符串（如"174F:2435"）
 */
DHUA::DHUA(double exposure_ms, double gain, const std::string & vid_pid)
: exposure_us_(exposure_ms * 1e4),  // 曝光时间：毫秒转微秒
  gain_(gain),                      // 初始化增益
  queue_(1),                        // 线程安全队列（容量1）
  daemon_quit_(false),              // 守护线程退出标志
  vid_(-1),                         // 厂商ID（初始无效）
  pid_(-1),                         // 产品ID（初始无效）
  system_sptr_(CSystem::getInstance()),  // 大华SDK系统单例（核心入口）头文件：GenICam\System.h 
                                        /*接口： static CSystem& getInstance();
                                        功能说明：CSystem 单例获取接口，返回 CSystem 单例对象的引用
                                        代码示例：CSystem &systemObj = CSystem::getInstance();*/

  camera_sptr_(),             // 相机智能指针（初始空）
  stream_sptr_(),             // 流对象智能指针（初始空）
  capturing_(false),                 // 采集状态标志
  capture_quit_(false)               // 采集线程退出标志
{
    // 解析VID/PID
    set_vid_pid(vid_pid);
    
    // 初始化libusb（USB重置用）
    if (libusb_init(NULL)) 
        tools::logger()->warn("Unable to init libusb!");

    // 启动守护线程（负责异常恢复）
    daemon_thread_ = std::thread{[this] 
    {
        tools::logger()->info("DHUA's daemon thread started.");

        // 启动采集
        capture_start();

        // 守护循环：异常检测与恢复
        while (!daemon_quit_) 
        {
            std::this_thread::sleep_for(100ms);
            if (capturing_) continue;  // 正常采集时跳过

            // 异常恢复流程
            capture_stop();    // 停止采集（清理状态）
            reset_usb();       // 重置USB
            capture_start();   // 重新启动采集
        }

        // 退出时停止采集
        capture_stop();
        tools::logger()->info("DHUA's daemon thread stopped.");
    }};
}

/**
 * @brief 析构函数：释放资源与线程回收
 */
DHUA::~DHUA()
{
    daemon_quit_ = true;
    if (daemon_thread_.joinable()) 
        daemon_thread_.join();  // 等待守护线程退出
    
    // 释放大华SDK资源
    if (stream_sptr_) stream_sptr_.reset();
    if (camera_sptr_) 
    {
        if (camera_sptr_->isConnected()) 
            camera_sptr_->disConnect();  // 断开相机连接
        camera_sptr_.reset();
    }

    tools::logger()->info("DHUA destructed.");
}

/**
 * @brief 读取图像数据（从队列获取）
 * @param img 输出图像（OpenCV格式）
 * @param timestamp 输出时间戳
 */
void DHUA::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
    CameraData data;
    queue_.pop(data);  // 从线程安全队列取数据
    img = data.img;
    timestamp = data.timestamp;
    //tools::logger()->info("DHUA's daemon thread stopped.");
}

/**
 * @brief 启动相机采集（适配大华SDK核心流程）
 * 流程：设备发现 → 连接 → 参数配置 → 流对象创建 → 采集线程启动
 */
void DHUA::capture_start()
{
    // 重置状态变量（避免重复启动导致的冲突）
    capturing_ = false;
    capture_quit_ = false;
    // 释放可能存在的旧资源
    if (streamThreadSptr) {
        streamThreadSptr->stop(); // 假设StreamRetrieve有stop方法
        streamThreadSptr.reset();
    }
    if (stream_sptr_) {
        stream_sptr_->stopGrabbing();
        stream_sptr_.reset();
    }
    if (camera_sptr_) {
        camera_sptr_->disConnect();
        camera_sptr_.reset();
    }
    queue_.clear(); // 清空图像队列

    // 1. 设备发现（仅USB设备）  
    TVector<ICameraPtr> camera_list;
    bool discovery_ok = system_sptr_.discovery(camera_list);  

    if (!discovery_ok || camera_list.empty()) 
    {
        tools::logger()->warn("DHUA camera discovery failed: no USB camera found");
        return;
    }

    // 2. 选择第一个设备并连接
    camera_sptr_ = camera_list[0];
    bool connect_ok = camera_sptr_->connect();  
    if (!connect_ok) 
    {
        tools::logger()->warn("DHUA camera connect failed");
        camera_sptr_.reset();
        return;
    }

    // 显示相机基本信息
    displayDeviceInfo(camera_list);
    
    // 手动设置参数（增加有效性检查）
    if (!setGrabMode(camera_sptr_, true)) 
    {
        tools::logger()->warn("Failed to set grab mode");
    }
    if (!setExposureMODE(camera_sptr_, false)) 
    {
        tools::logger()->warn("Failed to set exposure mode");
    }
    setGrabMode(camera_sptr_, true);    // 设置相机为连续取流模式
    setExposureMODE(camera_sptr_,false);//设置为非自动曝光模式
    setCameraExposureTime(camera_sptr_,CameraExposureTime, true);//设置曝光时间
    setBalanceRatio(camera_sptr_,dRED,dGREEN,dBLUE);//设置白平衡值
    setGainRawValue(camera_sptr_,GainRawValue,false);//设置总体增益值（不改曝光的前提下提高可见度）
    SetGamma(camera_sptr_,Gamma,true);//设置gamma值

    // 获取并设置分辨率（避免ROI越界）
    int64_t nwidth, nheight;
    if (getResolution(camera_sptr_, nwidth, nheight)) 
    {
        // 示例ROI设置（确保在有效范围内）
        int64_t roi_x = 640, roi_y = 300, roi_w = 640, roi_h = 480;
        if (roi_x + roi_w <= nwidth && roi_y + roi_h <= nheight) 
        {
            setROI(camera_sptr_, roi_x, roi_y, roi_w, roi_h);
        } else 
        {
            tools::logger()->warn("ROI out of range, using full resolution");
        }
    } else 
    {
        tools::logger()->warn("Failed to get resolution");
    }

    // 帧率设置（避免超出相机能力）
    double max_fps;
    if (getAcquisitionFrameRate(camera_sptr_, max_fps)) {
        double target_fps = std::min(500.0, max_fps); // 不超过相机最大帧率
        setAcquisitionFrameRate(camera_sptr_, target_fps);
    } else 
    {
        tools::logger()->warn("Failed to get max fps, using default");
        // double target_fps = std::min(500.0, max_fps); // 不超过相机最大帧率
        // setAcquisitionFrameRate(camera_sptr_, target_fps);
    }

    // 3. 创建流对象
    stream_sptr_ = system_sptr_.createStreamSource(camera_sptr_, 0);  
    if (!stream_sptr_) 
    {
        tools::logger()->warn("DHUA create stream failed");
        camera_sptr_->disConnect();
        camera_sptr_.reset();
        return;
    }

    // 4. 启动流采集
    if (!stream_sptr_->startGrabbing()) 
    {
        tools::logger()->warn("DHUA start grabbing failed");
        stream_sptr_.reset();
        camera_sptr_->disConnect();
        camera_sptr_.reset();
        return;
    }

    // 5. 创建并启动取流线程（增加线程安全保护）
    // std::lock_guard<std::mutex> lock(stream_mtx_); // 新增互斥锁保护
    // streamThreadSptr = Dahua::Memory::TSharedPtr<StreamRetrieve>(new StreamRetrieve(stream_sptr_));
    // if (!streamThreadSptr)
    // {
    //     tools::logger()->error("create thread obj failed.");
    //     stream_sptr_->stopGrabbing();
    //     stream_sptr_.reset();
    //     camera_sptr_->disConnect();
    //     camera_sptr_.reset();
    //     return;
    // }
    // streamThreadSptr->start();

    // 等待线程就绪（替代固定sleep，使用条件变量）
    //std::unique_lock<std::mutex> ready_lock(ready_mtx_);
    //最多等待500ms，超时则判定为失败
    // if (!ready_cv_.wait_for(ready_lock, std::chrono::milliseconds(10000000), 
    //                        [this]() { return streamThreadSptr->start(); })) { // 假设StreamRetrieve有isReady方法
    //     tools::logger()->error("stream thread not ready in time");
    //     streamThreadSptr->stop();
    //     stream_sptr_->stopGrabbing();
    //     stream_sptr_.reset();
    //     camera_sptr_->disConnect();
    //     camera_sptr_.reset();
    //     streamThreadSptr.reset();
    //     return;
    // }

    
    

    // 6. 启动采集线程（优化图像获取逻辑）
    capture_thread_ = std::thread{[this] 
    {
        //tools::logger()->info("DHUA's capture thread started.");
        capturing_ = true;
        CFrame frame;
        //const void* frame;
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 降低CPU占用
        while (!capture_quit_) 
        {
            
            stream_sptr_->getFrame(frame, 100);
            // 第一步：配置转码参数，将非Mono8数据转为BGR24
            // 1.1 计算BGR24缓冲区大小（宽×高×3，3表示B/G/R三通道）
            int imgWidth = frame.getImageWidth();
            int imgHeight = frame.getImageHeight();
            int bgr24BufSize = imgWidth * imgHeight * 3;

            // 1.2 分配BGR24输出缓冲区（需手动释放，避免内存泄漏）
            uint8_t* pBGR24Data = new uint8_t[bgr24BufSize];
            if (pBGR24Data == nullptr) {
                printf("内存分配失败！\n");
                return;
            }

            // 1.3 配置转码参数结构体IMGCNV_SOpenParam
            IMGCNV_SOpenParam openParam;
            openParam.width = imgWidth;                  // 图像宽度
            openParam.height = imgHeight;                // 图像高度
            openParam.paddingX = frame.getImagePadddingX(); // 图像X方向padding（SDK内部对齐用）
            openParam.paddingY = frame.getImagePadddingY(); // 图像Y方向padding
            openParam.dataSize = frame.getImageSize();    // 原始裸数据大小（frame.getImageSize()获取）
            openParam.pixelForamt = frame.getImagePixelFormat(); // 原始像素格式（如Bayer格式）

            // 1.4 调用转码函数，将原始数据转为BGR24
            IMGCNV_EErr transStatus = IMGCNV_ConvertToBGR24(
                const_cast<unsigned char*>(static_cast<const unsigned char*>(frame.getImage())),       // 输入：原始裸数据
                &openParam,             // 输入：转码参数
                pBGR24Data,             // 输出：BGR24数据缓冲区
                &bgr24BufSize           // 输入输出：缓冲区大小（需确保足够）
            );
            if (transStatus != IMGCNV_SUCCESS) {
                printf("转码失败！错误码：%d\n", transStatus);
                delete[] pBGR24Data;    // 转码失败时释放内存
                return;
            }

            // 第二步：用BGR24数据构建cv::Mat对象
            // CV_8UC3表示3通道8位无符号类型（匹配BGR24格式）
            // 第二步：用BGR24数据构建cv::Mat对象（添加clone()确保数据拷贝）
            cv::Mat bgr24Mat(imgHeight, imgWidth, CV_8UC3, pBGR24Data);
            cv::Mat img_bgr = bgr24Mat.clone(); // 关键：clone()会拷贝数据到新内存，脱离原缓冲区
            queue_.push({img_bgr, std::chrono::steady_clock::now()});
            delete[] pBGR24Data; // 释放原缓冲区（此时img_bgr已持有独立数据，不会悬空）

            // 后续入队使用img_bgr（而非bgr24Mat）
            
            
            // 线程安全访问streamThreadSptr
            // cv::Mat img_rgb;
            // {
            //     std::lock_guard<std::mutex> lock(stream_mtx_);
            //     if (streamThreadSptr) 
            //     {
            //         img_rgb = streamThreadSptr->getMatImage();
                    
            //     }
            // }

            // 检查图像有效性
            // if (img_rgb.empty()) 
            // {
            //     //tools::logger()->warn("get empty image, skip");
            //     continue;
            // }

            // 图像入队（线程安全队列）
            // {
            //     //tools::logger()->info("get  image, ok");
            //     std::lock_guard<std::mutex> q_lock(queue_mtx_); // 新增队列锁
            //     queue_.push({bgr24Mat, std::chrono::steady_clock::now()});
            //     // 限制队列大小，避免内存溢出

            // }
        }

        // 采集线程退出清理
        capturing_ = false;
        tools::logger()->info("DHUA's capture thread stopped.");
    }};
}



/**
 * @brief 停止采集（释放大华SDK资源）
 */
void DHUA::capture_stop()
{
    // 1. 停止采集线程
    capture_quit_ = true;
    if (capture_thread_.joinable()) 
        capture_thread_.join();

    // 2. 停止流采集（对应MV_CC_StopGrabbing）
    if (stream_sptr_) 
    {
        stream_sptr_->stopGrabbing();  // 大华：停止采集
        stream_sptr_.reset();          // 释放流对象
    }

    // 3. 断开相机连接（对应MV_CC_CloseDevice + DestroyHandle）
    if (camera_sptr_) 
    {
        if (camera_sptr_->isConnected()) 
            camera_sptr_->disConnect();  // 大华：断开连接
        camera_sptr_.reset();            // 释放相机对象
    }
}

/**
 * @brief 设置浮点型属性（适配大华CIntNode/CDoubleNode）
 * @param name 属性名（如"ExposureTime"）
 * @param value 属性值
 */
void DHUA::set_float_value(const std::string & name, double value)
{
    if (!camera_sptr_ || !camera_sptr_->isConnected()) 
    {
        tools::logger()->warn("DHUA camera not connected (set_float_value)");
        return;
    }

    // 大华：判断属性类型（整数/浮点），优先用CDoubleNode
    CDoubleNode node(camera_sptr_, name.c_str());
    if (node.isValid()) 
    {
        if (!node.setValue(value))  // 大华：设置浮点值
            tools::logger()->warn("DHUA set_float_value failed: {}", name);
        return;
    }

    // 若为整数属性（如部分相机的ExposureTime用整数）
    CIntNode int_node(camera_sptr_, name.c_str());
    if (int_node.isValid()) 
    {
        if (!int_node.setValue(static_cast<int64_t>(value))) 
            tools::logger()->warn("DHUA set_int_value failed: {}", name);
    }
    else 
    {
        tools::logger()->warn("DHUA float property not found: {}", name);
    }
}

/**
 * @brief 设置枚举型属性（适配大华CEnumNode）
 * @param name 属性名（如"ExposureAuto"）
 * @param value 属性值（字符串，如"Off"）
 */
void DHUA::set_enum_value(const std::string & name, const std::string & value)
{
    if (!camera_sptr_ || !camera_sptr_->isConnected()) 
    {
        tools::logger()->warn("DHUA camera not connected (set_enum_value)");
        return;
    }

    // 大华：枚举属性操作（通过symbol值设置）
    CEnumNode node(camera_sptr_, name.c_str());
    if (!node.isValid()) 
    {
        tools::logger()->warn("DHUA enum property not found: {}", name);
        return;
    }

    if (!node.setValueBySymbol(value.c_str()))  // 大华：用字符串值设置枚举
        tools::logger()->warn("DHUA set_enum_value failed: {} → {}", name, value);
}

/**
 * @brief 解析VID:PID字符串（与原逻辑一致）
 */
void DHUA::set_vid_pid(const std::string & vid_pid)
{
    auto index = vid_pid.find(':');
    if (index == std::string::npos) 
    {
        tools::logger()->warn("Invalid vid_pid: {}", vid_pid);
        return;
    }

    auto vid_str = vid_pid.substr(0, index);
    auto pid_str = vid_pid.substr(index + 1);

    try 
    {
        vid_ = std::stoi(vid_str, 0, 16);  // 十六进制转整数
        pid_ = std::stoi(pid_str, 0, 16);
    } 
    catch (const std::exception &) 
    {
        tools::logger()->warn("Invalid vid_pid: {}", vid_pid);
    }
}

/**
 * @brief USB重置（与原逻辑一致）
 */
void DHUA::reset_usb() const
{
    if (vid_ == -1 || pid_ == -1) return;

    libusb_device_handle* handle = libusb_open_device_with_vid_pid(NULL, vid_, pid_);
    if (!handle) 
    {
        tools::logger()->warn("Unable to open USB device (VID:{:04X}, PID:{:04X})", vid_, pid_);
        return;
    }

    if (libusb_reset_device(handle) != 0)
        tools::logger()->warn("USB reset failed (VID:{:04X}, PID:{:04X})", vid_, pid_);
    else
        tools::logger()->info("USB reset successfully (VID:{:04X}, PID:{:04X})", vid_, pid_);

    libusb_close(handle);
}

/**
 * @brief 辅助函数：大华CFrame转OpenCV Mat（处理Bayer格式）
 * @param frame 大华图像帧
 * @param img_rgb 输出RGB图像（OpenCV）
 * @return 转换成功标志
 */
// 3. 修正后的转换函数（完全匹配头文件枚举定义）
bool DHUA::convert_dahua_frame_to_opencv(const CFrame & frame, cv::Mat & img_rgb)
{
    // 步骤1：获取帧基本信息（宽度、高度、裸数据，参考手册3.5.4节）
    int width = frame.getImageWidth();
    int height = frame.getImageHeight();

    /*获取图像裸数据(RAW Data)
    使用主动采图或回调取图方式，可以获取到相机采集的一帧图像数据(CFrame 对象)。
    CFrame 对象提供了获取图像的裸数据、裸数据长度、图像宽高等信息的接口。
    此处仅描述获取图像裸数据接口。
    头文件：Frame.h
    接口：const void * getImage() const;
    功能说明：获取该帧图像裸数据的内存首地址。

    补充说明：可以通过 Frame.h 中的 getImageSize()接口，获取图像裸数据（RAW Data）
    的长度。这样从首地址开始，偏移相应的长度，就可以获得整个图像裸数
    据*/
    const void* raw_data = frame.getImage();
    if (!raw_data || width <= 0 || height <= 0) 
    {
        tools::logger()->warn("Invalid frame data: width={}, height={}, raw_data={}", 
                             width, height, raw_data);
        return false;
    }

    // 步骤2：获取像素格式（关键修正：使用EPixelType，匹配头文件定义）
    // 说明：frame.getImagePixelFormat()返回值即为EPixelType类型，无需强制转换
    EPixelType pixel_type = frame.getImagePixelFormat();
    // 构建单通道原始Mat（Bayer/Mono8均为单通道裸数据，参考手册4.2节）
    cv::Mat img_mono(width, height, CV_8UC1, const_cast<void*>(raw_data));

    // 步骤3：Bayer/Mono8转RGB（枚举值完全匹配头文件，带命名空间）
    switch (pixel_type) 
    {
        case gvspPixelBayGR8:  // 匹配头文件BayerGR8定义（无Type_Gvsp_，Bay缩写）
            cv::cvtColor(img_mono, img_rgb, cv::COLOR_BayerGR2RGB);
            break;
        case gvspPixelBayRG8:  // 匹配头文件BayerRG8定义
            cv::cvtColor(img_mono, img_rgb, cv::COLOR_BayerRG2RGB);
            break;
        case gvspPixelBayGB8:  // 匹配头文件BayerGB8定义
            cv::cvtColor(img_mono, img_rgb, cv::COLOR_BayerGB2RGB);
            break;
        case gvspPixelBayBG8:  // 匹配头文件BayerBG8定义
            cv::cvtColor(img_mono, img_rgb, cv::COLOR_BayerBG2RGB);
            break;
        case gvspPixelMono8:   // 匹配头文件Mono8定义
            cv::cvtColor(img_mono, img_rgb, cv::COLOR_GRAY2RGB);  // 单色转RGB
            break;
        default:
            // 打印未支持的枚举值（便于调试，可对照头文件确认格式）
            tools::logger()->warn("Unsupported pixel type: {} (enum name: {}, value: {})");
            return false;
    }

    return true;
}




/*shark:**************************************下面不用看，主要功能都在上面************************************************************/


// 设置相机采图模式（连续采图、触发采图）
// Set camera acquisition mode (continuous acquisition, triggered acquisition)
int32_t DHUA::setGrabMode(Dahua::GenICam::ICameraPtr camera_sptr_, bool bContious)
{
    int32_t bRet;
    Dahua::GenICam::IAcquisitionControlPtr sptrAcquisitionControl = Dahua::GenICam::CSystem::getInstance().createAcquisitionControl(camera_sptr_);//shark:Dahua::GenICam::IAcquisitionControlPtr（In Acquisition control pointer输入采集控制指针）
    if (NULL == sptrAcquisitionControl)
    {
        return -1;
    }

    Dahua::GenICam::CEnumNode enumNode = sptrAcquisitionControl->triggerSelector();//shark：triggerSelector触发器选择器
    bRet = enumNode.setValueBySymbol("FrameStart");//设置为FrameStart帧开始类型（连续采图）
    if (false == bRet)//shark:设置失败
    {
        printf("set TriggerSelector fail.\n");
        return -1;
    }

    if (true == bContious)//shark:设置为连续时执行
    {
        enumNode = sptrAcquisitionControl->triggerMode();//shark:获取触发器模式
        bRet = enumNode.setValueBySymbol("Off");//
        if (false == bRet)
        {
            printf("set triggerMode fail.\n");
            return -1;
        }
    }
    else
    {
        enumNode = sptrAcquisitionControl->triggerMode();
        bRet = enumNode.setValueBySymbol("On");
        if (false == bRet)
        {
            printf("set triggerMode fail.\n");
            return -1;
        }

        // 设置触发源为软触发（硬触发为Line1）
        // Set trigger source as soft trigger (hard trigger as Line1)
        enumNode = sptrAcquisitionControl->triggerSource();
        bRet = enumNode.setValueBySymbol("Software");
        if (false == bRet)
        {
            printf("set triggerSource fail.\n");
            return -1;
        }
    }
    return 0;
}

// 获取相机采图模式
// Get camera acquisition mode
int32_t getGrabMode(Dahua::GenICam:: ICameraPtr& camera_sptr_, bool &bContious)
{
    int32_t bRet;
    Dahua::GenICam::IAcquisitionControlPtr sptrAcquisitionControl = Dahua::GenICam::CSystem::getInstance().createAcquisitionControl(camera_sptr_);
    if (NULL == sptrAcquisitionControl)
    {
        return -1;
    }

    Dahua::GenICam::CEnumNode enumNode = sptrAcquisitionControl->triggerSelector();
    bRet = enumNode.setValueBySymbol("FrameStart");
    if (false == bRet)
    {
        printf("set TriggerSelector fail.\n");
        return -1;
    }

    Dahua::Infra:: CString strValue;
    enumNode = sptrAcquisitionControl->triggerMode();
    bRet = enumNode.getValueSymbol(strValue);
    if (false == bRet)
    {
        printf("get triggerMode fail.\n");
        return -1;
    }

    if (strValue == "Off")
    {
        bContious = true;
    }
    else if (strValue == "On")
    {
        bContious = false;
    }
    else
    {
        printf("get triggerMode fail.\n");
        return -1;
    }
    return 0;
}

// 软件触发
// software trigger
int32_t triggerSoftware(Dahua::GenICam:: ICameraPtr& camera_sptr_)
{
    int32_t bRet;
    Dahua::GenICam::IAcquisitionControlPtr sptrAcquisitionControl = Dahua::GenICam::CSystem::getInstance().createAcquisitionControl(camera_sptr_);
    if (NULL == sptrAcquisitionControl)
    {
        printf("AcquisitionControl fail.\n");
        return -1;
    }

    Dahua::GenICam:: CCmdNode  cmdNode = sptrAcquisitionControl->triggerSoftware();
    bRet = cmdNode.execute();
    if (false == bRet)
    {
        printf("triggerSoftware execute fail.\n");
        return -1;
    }
    return 0;
}

// 设置传感器采样率（采集分辨率）
// Set sensor sampling rate (acquisition resolution)
int32_t setResolution(Dahua::GenICam:: ICameraPtr& camera_sptr_, int nWidth, int nHeight)
{
    int32_t bRet;
    Dahua::GenICam::IImageFormatControlPtr sptrImageFormatControl = Dahua::GenICam::CSystem::getInstance().createImageFormatControl(camera_sptr_);
    if (NULL == sptrImageFormatControl)
    {
        return -1;
    }

    Dahua::GenICam::CIntNode intNode = sptrImageFormatControl->width();
    bRet = intNode.setValue(nWidth);
    if (false == bRet)
    {
        printf("set width fail.\n");
        return -1;
    }

    intNode = sptrImageFormatControl->height();
    bRet = intNode.setValue(nHeight);
    if (false == bRet)
    {
        printf("set height fail.\n");
        return -1;
    }
    return 0;
}

// 获取传感器采样率
// Get sensor sample rate
int32_t DHUA::getResolution(Dahua::GenICam:: ICameraPtr& camera_sptr_, int64_t &nWidth, int64_t &nHeight)
{
    int32_t bRet;
    Dahua::GenICam:: IImageFormatControlPtr sptrImageFormatControl = Dahua::GenICam::CSystem::getInstance().createImageFormatControl(camera_sptr_);
    if (NULL == sptrImageFormatControl)
    {
        return -1;
    }

    Dahua::GenICam::CIntNode intNode = sptrImageFormatControl->width();
    bRet = intNode.getValue(nWidth);
    if (false == bRet)
    {
        printf("get width fail.\n");
        return -1;
    }

    intNode = sptrImageFormatControl->height();
    bRet = intNode.getValue(nHeight);
    if (false == bRet)
    {
        printf("get height fail.\n");
        return -1;
    }
    return 0;
}

// 设置binning (Off X Y XY)
// set binning (Off X Y XY)
int32_t setBinning(Dahua::GenICam:: ICameraPtr& camera_sptr_)
{
    Dahua::GenICam::CEnumNodePtr ptrParam(new Dahua::GenICam::CEnumNode(camera_sptr_, "Binning"));
    if (ptrParam)
    {
        if (false == ptrParam->isReadable())
        {
            printf("binning not support.\n");
            return -1;
        }

        if (false == ptrParam->setValueBySymbol("XY"))
        {
            printf("set Binning XY fail.\n");
            return -1;
        }

        if (false == ptrParam->setValueBySymbol("Off"))
        {
            printf("set Binning Off fail.\n");
            return -1;
        }
    }
    return 0;
}

// 获取传感器最大分辩率
// get the maximum resolution of the sensor
int32_t getMaxResolution(Dahua::GenICam:: ICameraPtr& camera_sptr_, int64_t &nWidthMax, int64_t &nHeightMax)
{
    Dahua::GenICam::CIntNodePtr ptrParamSensorWidth(new Dahua::GenICam::CIntNode(camera_sptr_, "SensorWidth"));
    if (ptrParamSensorWidth)
    {
        if (false == ptrParamSensorWidth->getValue(nWidthMax))
        {
            printf("get WidthMax fail.\n");
            return -1;
        }
    }

    Dahua::GenICam::CIntNodePtr ptrParamSensorHeight(new Dahua::GenICam::CIntNode(camera_sptr_, "SensorHeight"));
    if (ptrParamSensorHeight)
    {
        if (false == ptrParamSensorHeight->getValue(nWidthMax))
        {
            printf("get WidthMax fail.\n");
            return -1;
        }
    }

    return 0;
}

// 设置图像ROI
// set image ROI
int32_t DHUA::setROI(Dahua::GenICam:: ICameraPtr& camera_sptr_, int64_t nX, int64_t nY, int64_t nWidth, int64_t nHeight)
{
    bool bRet;
    Dahua::GenICam:: IImageFormatControlPtr sptrImageFormatControl = Dahua::GenICam::CSystem::getInstance().createImageFormatControl(camera_sptr_);
    if (NULL == sptrImageFormatControl)
    {
        return -1;
    }

    // 设置宽
    // set width
    Dahua::GenICam::CIntNode intNode = sptrImageFormatControl->width();
    bRet = intNode.setValue(nWidth);
    if (!bRet)
    {
        printf("set width fail.\n");
        return -1;
    }

    // 设置长
    // set height
    intNode = sptrImageFormatControl->height();
    bRet = intNode.setValue(nHeight);
    if (!bRet)
    {
        printf("set height fail.\n");
        return -1;
    }

    // 设置X偏移
    // set OffsetX
    intNode = sptrImageFormatControl->offsetX();
    bRet = intNode.setValue(nX);
    if (!bRet)
    {
        printf("set offsetX fail.\n");
        return -1;
    }

    // 设置Y偏移
    // set OffsetY
    intNode = sptrImageFormatControl->offsetY();
    bRet = intNode.setValue(nY);
    if (!bRet)
    {
        printf("set offsetY fail.\n");
        return -1;
    }

    return 0;
}

// 获取图像ROI
// get image ROI
int32_t DHUA::getROI(Dahua::GenICam:: ICameraPtr& camera_sptr_, int64_t &nX, int64_t &nY, int64_t &nWidth, int64_t &nHeight)
{
    bool bRet;
    Dahua::GenICam:: IImageFormatControlPtr sptrImageFormatControl = Dahua::GenICam::CSystem::getInstance().createImageFormatControl(camera_sptr_);
    if (NULL == sptrImageFormatControl)
    {
        return -1;
    }

    // 设置宽
    // set width
    Dahua::GenICam::CIntNode intNode = sptrImageFormatControl->width();
    bRet = intNode.getValue(nWidth);
    if (!bRet)
    {
        printf("get width fail.\n");
    }

    // 设置长
    // set height
    intNode = sptrImageFormatControl->height();
    bRet = intNode.getValue(nHeight);
    if (!bRet)
    {
        printf("get height fail.\n");
    }

    // 设置X偏移
    // set OffsetX
    intNode = sptrImageFormatControl->offsetX();
    bRet = intNode.getValue(nX);
    if (!bRet)
    {
        printf("get offsetX fail.\n");
    }

    // 设置Y偏移
    // set OffsetY
    intNode = sptrImageFormatControl->offsetY();
    bRet = intNode.getValue(nY);
    if (!bRet)
    {
        printf("get offsetY fail.\n");
    }
    return 0;
}

// 获取采图图像宽度
// Get the width of the image
int32_t getWidth(Dahua::GenICam:: ICameraPtr& camera_sptr_, int64_t &nWidth)
{
    bool bRet;
    Dahua::GenICam:: IImageFormatControlPtr sptrImageFormatControl = Dahua::GenICam::CSystem::getInstance().createImageFormatControl(camera_sptr_);
    if (NULL == sptrImageFormatControl)
    {
        return -1;
    }

    Dahua::GenICam::CIntNode intNode = sptrImageFormatControl->width();
    bRet = intNode.getValue(nWidth);
    if (!bRet)
    {
        printf("get width fail.\n");
    }
    return 0;
}

// 获取采图图像高度
// Get the height of the image
int32_t DHUA::getHeight(Dahua::GenICam:: ICameraPtr& camera_sptr_, int64_t &nHeight)
{
    bool bRet;
    Dahua::GenICam:: IImageFormatControlPtr sptrImageFormatControl = Dahua::GenICam::CSystem::getInstance().createImageFormatControl(camera_sptr_);
    if (NULL == sptrImageFormatControl)
    {
        return -1;
    }

    Dahua::GenICam::CIntNode intNode = sptrImageFormatControl->height();
    bRet = intNode.getValue(nHeight);
    if (!bRet)
    {
        printf("get height fail.\n");
        return -1;
    }
    return 0;
}

// 设置曝光模式(曝光、自动曝光/手动曝光)shark:这只有连续曝光和手动曝光，没有一次曝光
// Set exposure value (exposure, auto exposure / manual exposure)
int32_t DHUA::setExposureMODE(Dahua::GenICam:: ICameraPtr& camera_sptr_, bool bAutoExposure = false)
{
    bool bRet;
    Dahua::GenICam::IAcquisitionControlPtr sptrAcquisitionControl = Dahua::GenICam::CSystem::getInstance().createAcquisitionControl(camera_sptr_);
    if (NULL == sptrAcquisitionControl)
    {
        return -1;
    }

    if (bAutoExposure)
    {
        Dahua::GenICam::CEnumNode enumNode = sptrAcquisitionControl->exposureAuto();//shark：获取曝光属性
        bRet = enumNode.setValueBySymbol("Continuous");
        if (false == bRet)
        {
            printf("set exposureAuto fail.\n");
            return -1;
        }
    }
    else
    {
        Dahua::GenICam::CEnumNode enumNode = sptrAcquisitionControl->exposureAuto();
        bRet = enumNode.setValueBySymbol("Off");
        if (false == bRet)
        {
            printf("set exposureAuto fail.\n");
            return -1;
        }
    }
    return 0;
}

//shark:自己写的设置曝光值(addFlag：true添加型)
void DHUA::setCameraExposureTime(Dahua::GenICam:: ICameraPtr &camera_sptr_, double exposureTimeSet, bool addFlag) 
{
    Dahua::GenICam::IAcquisitionControlPtr sptrAcquisitionControl =Dahua::GenICam::CSystem::getInstance().createAcquisitionControl(camera_sptr_);
    if (nullptr == sptrAcquisitionControl) 
    {
        return;
    }
    double exposureTimeValue = 0.0;
    Dahua::GenICam::CDoubleNode exposureTime = sptrAcquisitionControl->exposureTime();
    if(exposureTime.getValue(exposureTimeValue))
    {
        if (addFlag) 
        {
            exposureTimeValue += exposureTimeSet;
            if (exposureTimeValue < 1.0)
            {
                exposureTimeValue = 1.0;
            }
                
            else if (exposureTimeValue > 20000.0)
            {
                exposureTimeValue = 20000.0; //因为这是放在线程里面的所以每进一次加一次
            }
                
            exposureTime.setValue(exposureTimeSet);
        } else 
        {
            if (exposureTimeSet < 1.0)
            {
                exposureTimeSet = 1.0;

            }
            else if (exposureTimeSet > 100000.0)
            {
                exposureTimeSet = 100000.0;
            }
                
            exposureTime.setValue(exposureTimeSet);
        }
        exposureTime.getValue(exposureTimeValue);
        printf("after change ,exposureTime is %lf\n",(exposureTimeValue));
    }
    else if(!exposureTime.getValue(exposureTimeValue))
    {
        printf("get exposureTime fail.\n");
    }
}

int32_t DHUA::setResolution(Dahua::GenICam::ICameraPtr &camera_sptr_, int nWidth, int nHeight)
{
    return 0;
}

/*（官方）
            // 获取曝光时间
            // get exposureTime
              int32_t getExposureTime(Dahua::GenICam:: ICameraPtr& camera_sptr_, double &dExposureTime)
            {
                bool bRet;
                Dahua::GenICam::IAcquisitionControlPtr sptrAcquisitionControl = Dahua::GenICam::CSystem::getInstance().createAcquisitionControl(camera_sptr_);
                if (NULL == sptrAcquisitionControl)
                {
                    return -1;
                }

                Dahua::GenICam::CDoubleNode doubleNode = sptrAcquisitionControl->exposureTime();
                bRet = doubleNode.getValue(dExposureTime);
                if (false == bRet)
                {
                    printf("get exposureTime fail.\n");
                    return -1;
                }
                return 0;
            }

            // 获取曝光范围
            // Get exposure range
              int32_t getExposureTimeMinMaxValue(Dahua::GenICam:: ICameraPtr& camera_sptr_, double &dMinValue, double &dMaxValue)
            {
                bool bRet;
                Dahua::GenICam::IAcquisitionControlPtr sptrAcquisitionControl = Dahua::GenICam::CSystem::getInstance().createAcquisitionControl(camera_sptr_);
                if (NULL == sptrAcquisitionControl)
                {
                    return -1;
                }

                Dahua::GenICam::CDoubleNode doubleNode = sptrAcquisitionControl->exposureTime();
                bRet = doubleNode.getMinVal(dMinValue);
                if (false == bRet)
                {
                    printf("get exposureTime minValue fail.\n");
                    return -1;
                }

                bRet = doubleNode.getMaxVal(dMaxValue);
                if (false == bRet)
                {
                    printf("get exposureTime maxValue fail.\n");
                    return -1;
                }
                return 0;
            }
            */
/*（官方）
            // 设置增益值
            // set gain
              int32_t setGainRaw(Dahua::GenICam:: ICameraPtr& camera_sptr_, double dGainRaw)
            {
                bool bRet;
                Dahua::GenICam::IAnalogControlPtr sptrAnalogControl = Dahua::GenICam::CSystem::getInstance().createAnalogControl(camera_sptr_);
                if (NULL == sptrAnalogControl)
                {
                    return -1;
                }

                Dahua::GenICam::CDoubleNode doubleNode = sptrAnalogControl->gainRaw();
                bRet = doubleNode.setValue(dGainRaw);
                if (false == bRet)
                {
                    printf("set gainRaw fail.\n");
                    return -1;
                }
                return 0;
            }

            // 获取增益值
            // get gain value
              int32_t getGainRaw(Dahua::GenICam:: ICameraPtr& camera_sptr_, double &dGainRaw)
            {
                bool bRet;
                Dahua::GenICam::IAnalogControlPtr sptrAnalogControl = Dahua::GenICam::CSystem::getInstance().createAnalogControl(camera_sptr_);
                if (NULL == sptrAnalogControl)
                {
                    return -1;
                }

                Dahua::GenICam::CDoubleNode doubleNode = sptrAnalogControl->gainRaw();
                bRet = doubleNode.getValue(dGainRaw);
                if (false == bRet)
                {
                    printf("get gainRaw fail.\n");
                    return -1;
                }
                return 0;
            }

            // 获取增益值范围
            // Get gain range
              int32_t getGainRawMinMaxValue(Dahua::GenICam:: ICameraPtr& camera_sptr_, double &dMinValue, double &dMaxValue)
            {
                bool bRet;
                Dahua::GenICam::IAnalogControlPtr sptrAnalogControl = Dahua::GenICam::CSystem::getInstance().createAnalogControl(camera_sptr_);
                if (NULL == sptrAnalogControl)
                {
                    return -1;
                }

                Dahua::GenICam::CDoubleNode doubleNode = sptrAnalogControl->gainRaw();
                bRet = doubleNode.getMinVal(dMinValue);//shark:将获取的最小值存到dMinValue中
                if (false == bRet)
                {
                    printf("get gainRaw minValue fail.\n");
                    return -1;
                }

                bRet = doubleNode.getMaxVal(dMaxValue);
                if (false == bRet)
                {
                    printf("get gainRaw maxValue fail.\n");
                    return -1;
                }
                return 0;
            }
            */

//shark：自己写的设置相机增益
void DHUA::setGainRawValue(Dahua::GenICam:: ICameraPtr &camera_sptr_,double gainRawSet ,bool addFlag)
{Dahua::GenICam::IAnalogControlPtr sptrAnalogControl = Dahua::GenICam::CSystem::getInstance().createAnalogControl(camera_sptr_);
    if (NULL == sptrAnalogControl) {
        return ;
    }
    double gainRawValue = 0.0;//shark：原属性值
    double dMaxValue,dMinValue;
    Dahua::GenICam::CDoubleNode gainRaw = sptrAnalogControl->gainRaw();
    gainRaw.getValue(gainRawValue);
    gainRaw.getMaxVal(dMaxValue);
    gainRaw.getMinVal(dMinValue);
    if (addFlag) {//shark：如果是添加模式就将原值加设值
        gainRawValue += gainRawSet;
        if (gainRawValue < dMinValue)//shark：保证最小值和最大值不超过限度
            gainRawValue = dMinValue;
        else if (gainRawValue > dMaxValue)
            gainRawValue = dMaxValue;
        gainRaw.setValue(gainRawSet);
    } else {//如果是设定模式就直接设置
        if (gainRawSet < dMinValue)
            gainRawSet = dMinValue;
        else if (gainRawSet > dMaxValue)
            gainRawSet =dMaxValue;
        gainRaw.setValue(gainRawSet);
    }
    gainRaw.getValue(gainRawSet);
    printf("after change ,gainraw is:%lf\n",gainRawSet);
}

/*(官方)
            // 设置伽马值
            // Set gamma
              int32_t setGamma(Dahua::GenICam:: ICameraPtr& camera_sptr_, double dGamma)
            {
                bool bRet;
                Dahua::GenICam::IAnalogControlPtr sptrAnalogControl = Dahua::GenICam::CSystem::getInstance().createAnalogControl(camera_sptr_);
                if (NULL == sptrAnalogControl)
                {
                    return -1;
                }

                Dahua::GenICam::CDoubleNode doubleNode = sptrAnalogControl->gamma();
                bRet = doubleNode.setValue(dGamma);
                if (false == bRet)
                {
                    printf("set gamma fail.\n");
                    return -1;
                }
                return 0;
            }

            // 获取伽马值
            // Get gamma
              int32_t getGamma(Dahua::GenICam:: ICameraPtr& camera_sptr_, double &dGamma)
            {
                bool bRet;
                Dahua::GenICam::IAnalogControlPtr sptrAnalogControl = Dahua::GenICam::CSystem::getInstance().createAnalogControl(camera_sptr_);
                if (NULL == sptrAnalogControl)
                {
                    return -1;
                }

                Dahua::GenICam::CDoubleNode doubleNode = sptrAnalogControl->gamma();
                bRet = doubleNode.getValue(dGamma);
                if (false == bRet)
                {
                    printf("get gamma fail.\n");
                    return -1;
                }
                return 0;
            }

            // 获取伽马值范围
            // Get gamma range
              int32_t getGammaMinMaxValue(Dahua::GenICam:: ICameraPtr& camera_sptr_, double &dMinValue, double &dMaxValue)
            {
                bool bRet;
                Dahua::GenICam::IAnalogControlPtr sptrAnalogControl = Dahua::GenICam::CSystem::getInstance().createAnalogControl(camera_sptr_);
                if (NULL == sptrAnalogControl)
                {
                    return -1;
                }

                Dahua::GenICam::CDoubleNode doubleNode = sptrAnalogControl->gamma();
                bRet = doubleNode.getMinVal(dMinValue);
                if (false == bRet)
                {
                    printf("get gamma minValue fail.\n");
                    return -1;
                }

                bRet = doubleNode.getMaxVal(dMaxValue);
                if (false == bRet)
                {
                    printf("get gamma maxValue fail.\n");
                    return -1;
                }
                return 0;
            }
            */

//shark：自己写的gamma设置
void DHUA::SetGamma(Dahua::GenICam:: ICameraPtr &camera_sptr_, double gammaSet, bool addFlag) {
    Dahua::GenICam::IAnalogControlPtr sptrAnalogControl =Dahua::GenICam::CSystem::getInstance().createAnalogControl(camera_sptr_);
    if (nullptr == sptrAnalogControl) {
        return;
    }
    double gammaValue = 0.0;
    Dahua::GenICam::CDoubleNode gamma = sptrAnalogControl->gamma();
    gamma.getValue(gammaValue);
    if (addFlag) {
        gammaValue += gammaSet;
        if (gammaValue < 0.1)
            gammaValue = 0.1;
        else if (gammaValue > 10.0)
            gammaValue = 10.0;
        gamma.setValue(gammaSet);
    } else {
        if (gammaSet < 0.1)
            gammaSet = 0.1;
        else if (gammaSet > 4.0)
            gammaSet = 4.0;
        gamma.setValue(gammaSet);
    }
    gamma.getValue(gammaSet);
    printf("after change ,gamma is %lf\n",gammaSet);
}



// 设置白平衡值（有三个白平衡值）
// Set the white balance value ( three white balance values)
int32_t DHUA::setBalanceRatio(Dahua::GenICam:: ICameraPtr& camera_sptr_, double dRedBalanceRatio, double dGreenBalanceRatio, double dBlueBalanceRatio)
{
    bool bRet;
    Dahua::GenICam::IAnalogControlPtr sptrAnalogControl = Dahua::GenICam::CSystem::getInstance().createAnalogControl(camera_sptr_);
    if (NULL == sptrAnalogControl)
    {
        return -1;
    }

    // 关闭自动白平衡
    // Turn off auto white balance
    Dahua::GenICam::CEnumNode enumNode = sptrAnalogControl->balanceWhiteAuto();
    if (false == enumNode.isReadable())
    {
        printf("balanceRatio not support.\n");
        return -1;
    }

    bRet = enumNode.setValueBySymbol("Off");
    if (false == bRet)
    {
        printf("set balanceWhiteAuto Off fail.\n");
        return -1;
    }

    enumNode = sptrAnalogControl->balanceRatioSelector();
    bRet = enumNode.setValueBySymbol("Red");
    if (false == bRet)
    {
        printf("set red balanceRatioSelector fail.\n");
        return -1;
    }

    Dahua::GenICam::CDoubleNode doubleNode = sptrAnalogControl->balanceRatio();
    bRet = doubleNode.setValue(dRedBalanceRatio);
    if (false == bRet)
    {
        printf("set red balanceRatio fail.\n");
        return -1;
    }

    enumNode = sptrAnalogControl->balanceRatioSelector();
    bRet = enumNode.setValueBySymbol("Green");
    if (false == bRet)
    {
        printf("set green balanceRatioSelector fail.\n");
        return -1;
    }

    doubleNode = sptrAnalogControl->balanceRatio();
    bRet = doubleNode.setValue(dGreenBalanceRatio);
    if (false == bRet)
    {
        printf("set green balanceRatio fail.\n");
        return -1;
    }

    enumNode = sptrAnalogControl->balanceRatioSelector();
    bRet = enumNode.setValueBySymbol("Blue");
    if (false == bRet)
    {
        printf("set blue balanceRatioSelector fail.\n");
        return -1;
    }

    doubleNode = sptrAnalogControl->balanceRatio();
    bRet = doubleNode.setValue(dBlueBalanceRatio);
    if (false == bRet)
    {
        printf("set blue balanceRatio fail.\n");
        return -1;
    }
    return 0;
}

// 获取白平衡值（有三个白平衡值)
// Get white balance value (three white balance values)
int32_t DHUA::getBalanceRatio(Dahua::GenICam:: ICameraPtr& camera_sptr_, double &dRedBalanceRatio, double &dGreenBalanceRatio, double &dBlueBalanceRatio)
{
    bool bRet;
    Dahua::GenICam::IAnalogControlPtr sptrAnalogControl = Dahua::GenICam::CSystem::getInstance().createAnalogControl(camera_sptr_);
    if (NULL == sptrAnalogControl)
    {
        return -1;
    }

    Dahua::GenICam::CEnumNode enumNode = sptrAnalogControl->balanceRatioSelector();
    if (false == enumNode.isReadable())
    {
        printf("balanceRatio not support.\n");
        return -1;
    }

    bRet = enumNode.setValueBySymbol("Red");
    if (false == bRet)
    {
        printf("set red balanceRatioSelector fail.\n");
        return -1;
    }

    Dahua::GenICam::CDoubleNode doubleNode = sptrAnalogControl->balanceRatio();
    bRet = doubleNode.getValue(dRedBalanceRatio);
    if (false == bRet)
    {
        printf("get red balanceRatio fail.\n");
        return -1;
    }

    enumNode = sptrAnalogControl->balanceRatioSelector();
    bRet = enumNode.setValueBySymbol("Green");
    if (false == bRet)
    {
        printf("set green balanceRatioSelector fail.\n");
        return -1;
    }

    doubleNode = sptrAnalogControl->balanceRatio();
    bRet = doubleNode.getValue(dGreenBalanceRatio);
    if (false == bRet)
    {
        printf("get green balanceRatio fail.\n");
        return -1;
    }

    enumNode = sptrAnalogControl->balanceRatioSelector();
    bRet = enumNode.setValueBySymbol("Blue");
    if (false == bRet)
    {
        printf("set blue balanceRatioSelector fail.\n");
        return -1;
    }

    doubleNode = sptrAnalogControl->balanceRatio();
    bRet = doubleNode.getValue(dBlueBalanceRatio);
    if (false == bRet)
    {
        printf("get blue balanceRatio fail.\n");
        return -1;
    }
    return 0;
}

// 获取白平衡值范围
// Get white balance value range
int32_t DHUA::getBalanceRatioMinMaxValue(Dahua::GenICam:: ICameraPtr& camera_sptr_, double &dMinValue, double &dMaxValue)
{
    bool bRet;
    Dahua::GenICam::IAnalogControlPtr sptrAnalogControl = Dahua::GenICam::CSystem::getInstance().createAnalogControl(camera_sptr_);
    //shark:根据相机客户端找到相应属性的位置然后创建相应的对象指针（sptrAnalogControl），然后通过相机实例赋值
    if (NULL == sptrAnalogControl)//若对象为空
    {
        return -1;
    }

    Dahua::GenICam::CDoubleNode doubleNode = sptrAnalogControl->balanceRatio();//获取浮点数属性
    if (false == doubleNode.isReadable())//判断该属性是否可读
    {
        printf("balanceRatio not support.\n");
        return -1;
    }

    bRet = doubleNode.getMinVal(dMinValue);
    if (false == bRet)
    {
        printf("get balanceRatio min value fail.\n");
        return -1;
    }

    bRet = doubleNode.getMaxVal(dMaxValue);
    if (false == bRet)
    {
        printf("get balanceRatio max value fail.\n");
        return -1;
    }

    return 0;
}

// 设置采图速度（秒帧数）
// Set the acquisition speed (seconds\frames)
int32_t DHUA::setAcquisitionFrameRate(Dahua::GenICam:: ICameraPtr& camera_sptr_, double dFrameRate)
{
    bool bRet;
    Dahua::GenICam::IAcquisitionControlPtr sptAcquisitionControl = Dahua::GenICam::CSystem::getInstance().createAcquisitionControl(camera_sptr_);
    if (NULL == sptAcquisitionControl)
    {
        return -1;
    }

    Dahua::GenICam::CBoolNode booleanNode = sptAcquisitionControl->acquisitionFrameRateEnable();
    bRet = booleanNode.setValue(true);
    if (false == bRet)
    {
        printf("set acquisitionFrameRateEnable fail.\n");
        return -1;
    }

    Dahua::GenICam::CDoubleNode doubleNode = sptAcquisitionControl->acquisitionFrameRate();
    bRet = doubleNode.setValue(dFrameRate);
    if (false == bRet)
    {
        printf("set acquisitionFrameRate fail.\n");
        return -1;
    }
    printf("after set, acquisitonFrameRate is:%lf\n",dFrameRate);
    return 0;
}

// 获取采图速度（秒帧数）
// Get the acquisition speed (seconds and frames)
int32_t DHUA::getAcquisitionFrameRate(Dahua::GenICam:: ICameraPtr& camera_sptr_, double &dFrameRate)
{
    bool bRet;
    Dahua::GenICam::IAcquisitionControlPtr sptAcquisitionControl = Dahua::GenICam::CSystem::getInstance().createAcquisitionControl(camera_sptr_);
    if (NULL == sptAcquisitionControl)
    {
        return -1;
    }

    Dahua::GenICam::CDoubleNode doubleNode = sptAcquisitionControl->acquisitionFrameRate();
    bRet = doubleNode.getValue(dFrameRate);
    if (false == bRet)
    {
        printf("get acquisitionFrameRate fail.\n");
        return -1;
    }
    printf("get acquistionFrameRate is :%lf\n",dFrameRate);
    return 0;
}

// 保存参数
// Save parameters
int32_t DHUA::userSetSave(Dahua::GenICam:: ICameraPtr& camera_sptr_)
{
    bool bRet;
    Dahua::GenICam::IUserSetControlPtr sptUserSetControl = Dahua::GenICam::CSystem::getInstance().createUserSetControl(camera_sptr_);
    if (NULL == sptUserSetControl)
    {
        return -1;
    }

    bRet = sptUserSetControl->saveUserSet(Dahua::GenICam::IUserSetControl::userSet1);
    if (false == bRet)
    {
        printf("saveUserSet fail.\n");
        return -1;
    }

    return 0;
}

// 加载参数
// Load parameters
int32_t DHUA::loadUserSet(Dahua::GenICam:: ICameraPtr& camera_sptr_)
{
    bool bRet;
    Dahua::GenICam::IUserSetControlPtr sptUserSetControl = Dahua::GenICam::CSystem::getInstance().createUserSetControl(camera_sptr_);
    if (NULL == sptUserSetControl)
    {
        return -1;
    }

    bRet = sptUserSetControl->setCurrentUserSet(Dahua::GenICam::IUserSetControl::userSet1);
    if (false == bRet)
    {
        printf("saveUserSet fail.\n");
        return -1;
    }

    return 0;
}
/*shark：不需要触发
            // 设置外触发延时时间
            // set external trigger delay time
              int32_t setTriggerDelay(Dahua::GenICam:: ICameraPtr& camera_sptr_, double dDelayTime)
            {
                bool bRet;
                Dahua::GenICam::IAcquisitionControlPtr sptAcquisitionControl = Dahua::GenICam::CSystem::getInstance().createAcquisitionControl(camera_sptr_);
                if (NULL == sptAcquisitionControl)
                {
                    return -1;
                }

                Dahua::GenICam::CDoubleNode doubleNode = sptAcquisitionControl->triggerDelay();
                bRet = doubleNode.setValue(dDelayTime);
                if (false == bRet)
                {
                    printf("set triggerDelay fail.\n");
                    return -1;
                }

                return 0;
            }

            // 获取外触发延时时间
            // get external trigger delay time
              int32_t getTriggerDelay(Dahua::GenICam:: ICameraPtr& camera_sptr_, double &dDelayTime)
            {
                bool bRet;
                Dahua::GenICam::IAcquisitionControlPtr sptAcquisitionControl = Dahua::GenICam::CSystem::getInstance().createAcquisitionControl(camera_sptr_);
                if (NULL == sptAcquisitionControl)
                {
                    return -1;
                }

                Dahua::GenICam::CDoubleNode doubleNode = sptAcquisitionControl->triggerDelay();
                bRet = doubleNode.getValue(dDelayTime);
                if (false == bRet)
                {
                    printf("set triggerDelay fail.\n");
                    return -1;
                }

                return 0;
            }

            // 设置外触发模式（上升沿触发、下降沿触发）
            // Set external trigger mode (rising edge trigger, falling edge trigger)
              int32_t setLineTriggerMode(Dahua::GenICam:: ICameraPtr& camera_sptr_, bool bRisingEdge)
            {
                bool bRet;
                Dahua::GenICam::IAcquisitionControlPtr sptAcquisitionControl = Dahua::GenICam::CSystem::getInstance().createAcquisitionControl(camera_sptr_);
                if (NULL == sptAcquisitionControl)
                {
                    return -1;
                }

                Dahua::GenICam::CEnumNode enumNode = sptAcquisitionControl->triggerSelector();
                if (false == enumNode.setValueBySymbol("FrameStart"))
                {
                    printf("set triggerSelector fail.\n");
                    return -1;
                }

                enumNode = sptAcquisitionControl->triggerMode();
                if (false == enumNode.setValueBySymbol("On"))
                {
                    printf("set triggerMode fail.\n");
                    return -1;
                }

                enumNode = sptAcquisitionControl->triggerSource();
                if (false == enumNode.setValueBySymbol("Line1"))
                {
                    printf("set triggerSource fail.\n");
                    return -1;
                }

                enumNode = sptAcquisitionControl->triggerActivation();
                if (true == bRisingEdge)
                {
                    bRet = enumNode.setValueBySymbol("RisingEdge");
                }
                else
                {
                    bRet = enumNode.setValueBySymbol("FallingEdge");
                }

                return 0;
            }

            // 获取外触发模式（上升沿触发、下降沿触发）
            // Get external trigger mode (rising edge trigger, falling edge trigger)
              int32_t getLineTriggerMode(Dahua::GenICam:: ICameraPtr& camera_sptr_, bool &bRisingEdge)
            {
                bool bRet;
                Dahua::GenICam::IAcquisitionControlPtr sptAcquisitionControl = Dahua::GenICam::CSystem::getInstance().createAcquisitionControl(camera_sptr_);
                if (NULL == sptAcquisitionControl)
                {
                    return -1;
                }

                Dahua::GenICam::CEnumNode enumNode = sptAcquisitionControl->triggerSelector();
                if (false == enumNode.setValueBySymbol("FrameStart"))
                {
                    printf("set triggerSelector fail.\n");
                    return -1;
                }

                Dahua::Infra:: CString strValue;
                enumNode = sptAcquisitionControl->triggerActivation();
                if (true == bRisingEdge)
                {
                    bRet = enumNode.getValueSymbol(strValue);
                }
                else
                {
                    bRet = enumNode.getValueSymbol(strValue);
                }

                if (false == bRet)
                {
                    printf("get triggerActivation fail.\n");
                    return -1;
                }

                if (strValue == "RisingEdge")
                {
                    bRisingEdge = true;
                }
                else if (strValue == "FallingEdge")
                {
                    bRisingEdge = false;
                }
                else
                {
                    printf("get triggerActivation fail.\n");
                    return -1;
                }

                return 0;
            }

            // 设置外触发信号滤波时间
            // Set filtering time of external trigger signal
              int32_t setLineDebouncerTimeAbs(Dahua::GenICam:: ICameraPtr& camera_sptr_, double dLineDebouncerTimeAbs)
            {
                IDigitalIOControlPtr sptDigitalIOControl = Dahua::GenICam::CSystem::getInstance().createDigitalIOControl(camera_sptr_);
                if (NULL == sptDigitalIOControl)
                {
                    return -1;
                }

                Dahua::GenICam::CEnumNode enumNode = sptDigitalIOControl->lineSelector();
                if (false == enumNode.setValueBySymbol("Line1"))
                {
                    printf("set lineSelector fail.\n");
                    return -1;
                }

                Dahua::GenICam::CDoubleNode doubleNode = sptDigitalIOControl->lineDebouncerTimeAbs();
                if (false == doubleNode.setValue(dLineDebouncerTimeAbs))
                {
                    printf("set lineDebouncerTimeAbs fail.\n");
                    return -1;
                }

                return 0;
            }

            // 获取外触发信号滤波时间
            // Acquisition of filtering time of external trigger signal
              int32_t getLineDebouncerTimeAbs(Dahua::GenICam:: ICameraPtr& camera_sptr_, double &dLineDebouncerTimeAbs)
            {
                IDigitalIOControlPtr sptDigitalIOControl = Dahua::GenICam::CSystem::getInstance().createDigitalIOControl(camera_sptr_);
                if (NULL == sptDigitalIOControl)
                {
                    return -1;
                }

                Dahua::GenICam::CEnumNode enumNode = sptDigitalIOControl->lineSelector();
                if (false == enumNode.setValueBySymbol("Line1"))
                {
                    printf("set lineSelector fail.\n");
                    return -1;
                }

                Dahua::GenICam::CDoubleNode doubleNode = sptDigitalIOControl->lineDebouncerTimeAbs();
                if (false == doubleNode.getValue(dLineDebouncerTimeAbs))
                {
                    printf("get lineDebouncerTimeAbs fail.\n");
                    return -1;
                }

                return 0;
            }

            // 设置外触发脉冲宽度（不支持）  | Set external trigger width (not supported)
            // 获取外触发脉冲宽度（不支持）  | Get external trigger width (not supported)
            // 设置输出信号线（控制光源用）（面阵相机是Line0） | Set the output signal line (for controlling the light source) (the area array camera is line0)
            // 获取输出信号线（面阵相机是Line0） | get the output signal line (the area array camera is line0)
            // 设置外部光源曝光时间（设置输出值为TRUE的时间） | Set the exposure time of the external light source (set the time when the output value is true)
              int32_t setOutputTime(Dahua::GenICam:: ICameraPtr& camera_sptr_, int nTimeMS)
            {
                IDigitalIOControlPtr sptDigitalIOControl = Dahua::GenICam::CSystem::getInstance().createDigitalIOControl(camera_sptr_);
                if (NULL == sptDigitalIOControl)
                {
                    return -1;
                }

                Dahua::GenICam::CEnumNode paramLineSource(camera_sptr_, "LineSource");
                if (false == paramLineSource.setValueBySymbol("UserOutput1"))
                {
                    printf("set LineSource fail.");
                    return -1;
                }

                // 将输出信号拉高然后拉低
                // Pull the output signal up and down
                Dahua::GenICam::CBoolNode booleanNode = sptDigitalIOControl->userOutputValue();
                if (false == booleanNode.setValue(true))
                {
                    printf("set userOutputValue fail.\n");
                    return -1;
                }

                CThread::sleep(nTimeMS);

                if (false == booleanNode.setValue(false))
                {
                    printf("set userOutputValue fail.\n");
                    return -1;
                }

                return 0;
            }
            */
/*shark：不需要翻转
            //  获取外部光源曝光时间（输出信号的时间由软件侧控制） | get the exposure time of external light source (the time of output signal is controlled by the software side)
            //  设置X轴翻转  | Set X-axis flip
              int32_t setReverseX(Dahua::GenICam:: ICameraPtr& camera_sptr_, bool flag)
            {
                Dahua::GenICam:: IImageFormatControlPtr sptrImageFormatControl = Dahua::GenICam::CSystem::getInstance().createImageFormatControl(camera_sptr_);

                Dahua::GenICam::CBoolNode boolNodeReverseX = sptrImageFormatControl->reverseX();
                if(!boolNodeReverseX.setValue(flag))
                {
                    printf("set reverseX fail.\n");
                    return -1;
                }

                return 0;
            }

            // 设置Y轴翻转
            // Set X-axis flip
              int32_t setReverseY(Dahua::GenICam:: ICameraPtr& camera_sptr_, bool flag)
            {
                Dahua::GenICam:: IImageFormatControlPtr sptrImageFormatControl = Dahua::GenICam::CSystem::getInstance().createImageFormatControl(camera_sptr_);

                Dahua::GenICam::CBoolNode boolNodeReverseY = sptrImageFormatControl->reverseY();
                if(!boolNodeReverseY.setValue(flag))
                {
                    printf("set reverseY fail.\n");
                    return -1;
                }

                return 0;
            }
            */

// 设置相机IP （与相机连接之前调用）
// Set up camera IP (before calling with camera)
int32_t DHUA::setCameraIp(Dahua::GenICam:: ICameraPtr& camera_sptr_, char* ipAddress, char* subnetMask, char* gateway)
{
    Dahua::GenICam::IGigECameraPtr gigeCameraPtr = Dahua::GenICam::IGigECamera::getInstance(camera_sptr_);
    if(NULL == gigeCameraPtr)
    {
        return -1;
    }

    if(!gigeCameraPtr->forceIpAddress(ipAddress, subnetMask, gateway))
    {
        printf("Set device ip failed.\n");
        return -1;
    }

    return 0;
}

// 设置相机静态IP （与相机连接之后调用）
// Set camera   IP (after calling with camera)
int32_t DHUA::setCameraPersistentIP(Dahua::GenICam:: ICameraPtr& camera_sptr_)
{
    Dahua::GenICam::IGigECameraPtr gigeCameraPtr = Dahua::GenICam::IGigECamera::getInstance(camera_sptr_);
    if(NULL == gigeCameraPtr)
    {
        printf("gigeCameraPtr is null.\n");
        return -1;
    }

    Dahua::GenICam::ITransportLayerControlPtr transportLayerControlPtr= Dahua::GenICam::CSystem::getInstance().createTransportLayerControl(camera_sptr_);

    if(NULL == transportLayerControlPtr)
    {
        printf("transportLayerControlPtr is null.\n");
        return -1;
    }

    transportLayerControlPtr->gevCurrentIPConfigurationPersistentIP().setValue(true);
    transportLayerControlPtr->gevPersistentDefaultGateway().setValue(gigeCameraPtr->getGateway().c_str());
    transportLayerControlPtr->gevPersistentIPAddress().setValue(gigeCameraPtr->getIpAddress().c_str());
    transportLayerControlPtr->gevPersistentSubnetMask().setValue(gigeCameraPtr->getSubnetMask().c_str());

    return 0;
}

/* shark:官方这句没什么用，就每次都会曝光加2罢了，似乎。
             * 修改曝光时间 （与相机连接之后调用）
            // Modify exposure time (after calling connect camera)
              void modifyCamralExposureTime(Dahua::GenICam::CSystem &systemObj, Dahua::GenICam:: ICameraPtr& camera_sptr_)
            {
                Dahua::GenICam::IAcquisitionControlPtr sptrAcquisitionControl = systemObj.createAcquisitionControl(camera_sptr_);
                if (NULL == sptrAcquisitionControl)
                {
                    return;
                }

                double exposureTimeValue = 0.0;
                Dahua::GenICam::CDoubleNode exposureTime = sptrAcquisitionControl->exposureTime();

                exposureTime.getValue(exposureTimeValue);
                printf("before change ,exposureTime is %f. thread ID :%d\n", exposureTimeValue, CThread::getCurrentThreadID());
                exposureTime.setValue(exposureTimeValue + 2);
                exposureTime.getValue(exposureTimeValue);
                printf("after change ,exposureTime is %f. thread ID :%d\n", exposureTimeValue, CThread::getCurrentThreadID());
            }
            */
void DHUA::LogPrinterFunc(const char* log)
{
    return;
}

// ********************** 这部分处理与SDK操作相机无关，用于显示设备列表 begin*****************************
// ***********BEGIN: These functions are not related to API call and used to display device info***********
void DHUA::displayDeviceInfo(Dahua::Infra::TVector<Dahua::GenICam:: ICameraPtr>& vCameraPtrList)
{
    Dahua::GenICam:: ICameraPtr camera_sptr_;
    // 打印Title行
    // Print title lineusing namespace Dahua::GenICam;
    using namespace Dahua::Infra;

    printf("\nIdx Type Vendor     Model      S/N             DeviceUserID    IP Address    \n");
    printf("------------------------------------------------------------------------------\n");
    for (int cameraIndex = 0; cameraIndex < vCameraPtrList.size(); cameraIndex++)
    {
        camera_sptr_ = vCameraPtrList[cameraIndex];
        // Idx 设备列表的相机索引 最大表示字数：3
        // Camera index in device list, display in 3 characters
        printf("%-3d", cameraIndex + 1);

        // Type 相机的设备类型（GigE，U3V，CL，PCIe
        // Camera type (eg:GigE，U3V，CL，PCIe)
        switch (camera_sptr_->getType())
        {
        case Dahua::GenICam::ICamera::typeGige:
            printf(" GigE");
            break;
        case Dahua::GenICam::ICamera::typeU3v:
            printf(" U3V ");
            break;
        case Dahua::GenICam::ICamera::typeCL:
            printf(" CL  ");
            break;
        case Dahua::GenICam::ICamera::typePCIe:
            printf(" PCIe");
            break;
        default:
            printf("     ");
            break;
        }

        // VendorName 制造商信息 最大表示字数：10
        // Camera vendor name, display in 10 characters
        const char* vendorName = camera_sptr_->getVendorName();
        char vendorNameCat[11];
        if (strlen(vendorName) > 10)
        {
            strncpy(vendorNameCat, vendorName, 7);
            vendorNameCat[7] = '\0';
            strcat(vendorNameCat, "...");
            printf(" %-10.10s", vendorNameCat);
        }
        else
        {
            printf(" %-10.10s", vendorName);
        }

        // ModeName 相机的型号信息 最大表示字数：10
        // Camera model name, display in 10 characters
        printf(" %-10.10s", camera_sptr_->getModelName());

        // Serial Number 相机的序列号 最大表示字数：15
        // Camera serial number, display in 15 characters
        printf(" %-15.15s", camera_sptr_->getSerialNumber());

        // deviceUserID 自定义用户ID 最大表示字数：15
        // Camera user id, display in 15 characters
        const char* deviceUserID = camera_sptr_->getName();
        char deviceUserIDCat[16] = {0};
        if (strlen(deviceUserID) > 15)
        {
            strncpy(deviceUserIDCat, deviceUserID, 12);
            deviceUserIDCat[12] = '\0';
            strcat(deviceUserIDCat, "...");
            printf(" %-15.15s", deviceUserIDCat);
        }
        else
        {
            // 防止console显示乱码,UTF8转换成ANSI进行显示
            // Prevent console from displaying garbled code and convert utf8 to ANSI for display
            memcpy(deviceUserIDCat, deviceUserID, sizeof(deviceUserIDCat));
            printf(" %-15.15s", deviceUserIDCat);
        }

        // IPAddress GigE相机时获取IP地址
        // IP address of GigE camera
        Dahua::GenICam::IGigECameraPtr gigeCameraPtr = Dahua::GenICam::IGigECamera::getInstance(camera_sptr_);
        if (NULL != gigeCameraPtr.get())
        {
            Dahua::Infra:: CString ip = gigeCameraPtr->getIpAddress();
            printf(" %s", ip.c_str());
        }
        printf("\n");

    }
}

}
