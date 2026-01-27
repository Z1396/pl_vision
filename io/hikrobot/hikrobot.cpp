#include "hikrobot.hpp"

#include <libusb-1.0/libusb.h>

#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace io
{
/**
 * @brief HikRobot相机类的构造函数，初始化相机参数并启动守护线程
 * @param exposure_ms 曝光时间（毫秒）
 * @param gain 增益值
 * @param vid_pid 相机的VID:PID字符串（如"04B4:00F9"）
 */
HikRobot::HikRobot(double exposure_ms, double gain, const std::string & vid_pid)
// 初始化列表：初始化类成员变量
: exposure_us_(exposure_ms * 1e3),  // 将曝光时间从毫秒转换为微秒（1ms = 1000us）
  gain_(gain),                      // 初始化增益值
  queue_(1),                        // 初始化数据队列，最大容量为1（推测为线程安全队列）
  daemon_quit_(false),              // 守护线程退出标志初始化为false（不退出）
  vid_(-1),                         // 厂商ID初始化为-1（无效值）
  pid_(-1)                          // 产品ID初始化为-1（无效值）
{
  // 解析vid_pid字符串，设置vid_和pid_（若格式正确）
  set_vid_pid(vid_pid);
  
  // 初始化libusb库（USB设备通信库），若失败则输出警告日志
  /*返回 0：初始化成功；
  返回非 0：初始化失败（如系统不支持 USB 驱动、权限不足等），具体错误码可通过 libusb_strerror() 解析。*/
  if (libusb_init(NULL)) tools::logger()->warn("Unable to init libusb!");

  // 启动守护线程，负责相机的持续采集和异常恢复
  daemon_thread_ = std::thread{[this] 
  {
    tools::logger()->info("HikRobot's daemon thread started.");

    // 开始图像采集
    capture_start();

    // 守护线程主循环：在退出标志为false时持续运行
    while (!daemon_quit_) 
    {
      // 休眠100毫秒，降低CPU占用
      std::this_thread::sleep_for(100ms);

      // 如果正在采集，则跳过后续检查（正常工作状态）
      if (capturing_) continue;

      // 若未在采集状态（异常），执行恢复流程：
      capture_stop();    // 停止当前采集（确保状态干净）
      reset_usb();       // 重置USB连接（尝试恢复设备通信）
      capture_start();   // 重新开始采集
    }

    // 退出循环后，停止采集
    capture_stop();

    tools::logger()->info("HikRobot's daemon thread stopped.");
  }};
}

HikRobot::~HikRobot()
{
  daemon_quit_ = true;
  if (daemon_thread_.joinable()) daemon_thread_.join();
  tools::logger()->info("HikRobot destructed.");
}

void HikRobot::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  CameraData data;
  queue_.pop(data);

  img = data.img;
  timestamp = data.timestamp;
}

/**
 * @brief 启动相机采集流程，初始化设备并启动图像采集线程
 * 功能：枚举设备、创建句柄、配置参数、启动采集，并创建线程循环获取图像
 */
void HikRobot::capture_start()
{
    // 初始化采集状态标志：当前未在采集，采集线程不退出
    capturing_ = false;
    capture_quit_ = false;

    unsigned int ret; // 存储海康SDK函数的返回值（用于错误判断）

    // 设备列表结构体，用于存储枚举到的相机设备信息
    MV_CC_DEVICE_INFO_LIST device_list;
    // 枚举USB接口的相机设备（MV_USB_DEVICE表示USB设备类型）
    ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    if (ret != MV_OK) 
    {
        tools::logger()->warn("MV_CC_EnumDevices failed: {:#x}", ret); // 枚举失败，输出十六进制错误码
        return;
    }

    // 检查是否枚举到设备
    if (device_list.nDeviceNum == 0) 
    {
        tools::logger()->warn("Not found camera!"); // 未找到相机
        return;
    }

    // 创建相机设备句柄（handle_用于后续操作相机），使用第一个枚举到的设备
    ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
    if (ret != MV_OK) 
    {
        tools::logger()->warn("MV_CC_CreateHandle failed: {:#x}", ret); // 创建句柄失败
        return;
    }

    // 打开相机设备（建立与相机的连接）
    ret = MV_CC_OpenDevice(handle_);
    if (ret != MV_OK) 
    {
        tools::logger()->warn("MV_CC_OpenDevice failed: {:#x}", ret); // 打开设备失败
        return;
    }

    // 配置相机参数（通过海康SDK接口设置）
    set_enum_value("BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS); // 白平衡自动模式
    set_enum_value("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);           // 关闭自动曝光（使用手动设置）
    set_enum_value("GainAuto", MV_GAIN_MODE_OFF);                         // 关闭自动增益（使用手动设置）
    set_float_value("ExposureTime", exposure_us_);                        // 设置曝光时间（微秒）
    set_float_value("Gain", gain_);                                        // 设置增益值
    MV_CC_SetFrameRate(handle_, 150);                                     // 设置帧率为150fps

    // 开始图像采集（相机进入持续输出图像状态）
    ret = MV_CC_StartGrabbing(handle_);
    if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_StartGrabbing failed: {:#x}", ret); // 启动采集失败
        return;
    }

    // 创建图像采集线程，循环获取并处理图像
    capture_thread_ = std::thread{[this] {
        tools::logger()->info("HikRobot's capture thread started."); // 采集线程启动日志

        capturing_ = true; // 标记为正在采集状态

        MV_FRAME_OUT raw; // 原始图像数据结构体（来自相机SDK）
        MV_CC_PIXEL_CONVERT_PARAM cvt_param; // 像素格式转换参数（未实际使用）

        // 采集循环：在退出标志为false时持续运行
        while (!capture_quit_) 
        {
            std::this_thread::sleep_for(1ms); // 短暂休眠，降低CPU占用

            unsigned int ret;
            unsigned int nMsec = 100; // 超时时间（100毫秒）

            // 从相机获取一帧图像数据（超时返回错误）
            ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);
            if (ret != MV_OK) 
            {
                tools::logger()->warn("MV_CC_GetImageBuffer failed: {:#x}", ret); // 获取图像失败
                break; // 退出循环（通常触发守护线程的恢复机制）
            }

            // 记录当前时间戳（用于图像的时间标记）
            auto timestamp = std::chrono::steady_clock::now();
            // 根据原始图像信息创建OpenCV矩阵（初始为单通道，后续会转换）
            cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), CV_8U, raw.pBufAddr);

            // 以下为像素格式转换配置（实际未使用SDK转换，改用OpenCV转换）
            cvt_param.nWidth = raw.stFrameInfo.nWidth;
            cvt_param.nHeight = raw.stFrameInfo.nHeight;
            cvt_param.pSrcData = raw.pBufAddr;
            cvt_param.nSrcDataLen = raw.stFrameInfo.nFrameLen;
            cvt_param.enSrcPixelType = raw.stFrameInfo.enPixelType;
            cvt_param.pDstBuffer = img.data;
            cvt_param.nDstBufferSize = img.total() * img.elemSize();
            cvt_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

            // 注释：未使用海康SDK的格式转换函数，改用OpenCV转换
            // ret = MV_CC_ConvertPixelType(handle_, &cvt_param);
            
            // 获取原始图像的像素类型（通常为Bayer格式）
            const auto & frame_info = raw.stFrameInfo;
            auto pixel_type = frame_info.enPixelType;
            cv::Mat dst_image; // 转换后的图像

            // 映射海康像素格式到OpenCV的颜色转换码（Bayer转RGB）
            const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
                {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
                {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
                {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
                {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}
            };

            // 执行格式转换（Bayer模式转RGB彩色图像）
            cv::cvtColor(img, dst_image, type_map.at(pixel_type));
            img = dst_image; // 更新img为转换后的彩色图像

            // 将图像和时间戳打包，推入线程安全队列（供其他模块消费）
            queue_.push({img, timestamp});

            // 释放原始图像缓冲区（必须调用，否则内存泄漏）
            ret = MV_CC_FreeImageBuffer(handle_, &raw);
            if (ret != MV_OK) 
            {
                tools::logger()->warn("MV_CC_FreeImageBuffer failed: {:#x}", ret); // 释放缓冲区失败
                break; // 退出循环
            }
        }

        capturing_ = false; // 标记为停止采集状态
        tools::logger()->info("HikRobot's capture thread stopped."); // 采集线程停止日志
    }};
}

// 停止海康机器人相机的图像采集，并并释放相关资源
void HikRobot::capture_stop()
{
  // 1. 设置设置退出线程退出标记，通知通知采集线程终止循环
  capture_quit_ = true;
  
  // 2. 等待采集采集线程执行完毕（若线程可join）
  //    确保线程安全退出，避免资源竞争或未定义行为
  if (capture_thread_.joinable()) 
    capture_thread_.join();

  unsigned int ret;  // 用于接收SDK函数的返回值

  // 3. 停止相机采集
  //    通知相机停止输出图像帧，结束图像抓取流程
  ret = MV_CC_StopGrabbing(handle_);
  if (ret != MV_OK) 
  {
    tools::logger()->warn("MV_CC_StopGrabbing failed: {:#x}", ret);
    return;
  }

  // 4. 关闭相机设备
  //    断开与相机的连接，释放设备占用的硬件资源
  ret = MV_CC_CloseDevice(handle_);
  if (ret != MV_OK) 
  {
    tools::logger()->warn("MV_CC_CloseDevice failed: {:#x}", ret);
    return;
  }

  // 5. 销毁相机句柄
  //    释放相机句柄占用的内存资源，彻底清理初始化时创建的对象
  ret = MV_CC_DestroyHandle(handle_);
  if (ret != MV_OK) 
  {
    tools::logger()->warn("MV_CC_DestroyHandle failed: {:#x}", ret);
    return;
  }
}

void HikRobot::set_float_value(const std::string & name, double value)
{
  unsigned int ret;

  ret = MV_CC_SetFloatValue(handle_, name.c_str(), value);

  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetFloatValue(\"{}\", {}) failed: {:#x}", name, value, ret);
    return;
  }
}

void HikRobot::set_enum_value(const std::string & name, unsigned int value)
{
  unsigned int ret;

  ret = MV_CC_SetEnumValue(handle_, name.c_str(), value);

  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetEnumValue(\"{}\", {}) failed: {:#x}", name, value, ret);
    return;
  }
}

/**
 * @brief 设置相机的VID（厂商ID）和PID（产品ID）
 * @param vid_pid 字符串格式为"VID:PID"，其中VID和PID为十六进制数（如"04B4:00F9"）
 */
void HikRobot::set_vid_pid(const std::string & vid_pid)
{
    // 查找字符串中VID和PID的分隔符':'的位置，返回其索引（从 0 开始）。
    auto index = vid_pid.find(':');
    
    // 如果未找到':'，说明格式无效，输出警告日志并返回
    if (index == std::string::npos) 
    {
        tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
        return;
    }

    // 截取从 0 开始、长度为index的子串，即:前面的部分（VID字符串）。（十六进制）
    auto vid_str = vid_pid.substr(0, index);
    // 截取从index + 1开始到字符串末尾的子串，即:后面的部分（PID字符串）。（十六进制）
    auto pid_str = vid_pid.substr(index + 1);

    try 
    {
        // 将十六进制的VID字符串转换为整数，存储到成员变量vid_
        // 第三个参数16表示按十六进制解析
        vid_ = std::stoi(vid_str, 0, 16);
        // 将十六进制的PID字符串转换为整数，存储到成员变量pid_
        pid_ = std::stoi(pid_str, 0, 16);
    } catch (const std::exception &) 
    {
        // 若转换失败（如包含非十六进制字符），输出警告日志
        tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
    }
}

void HikRobot::reset_usb() const
{
  if (vid_ == -1 || pid_ == -1) return;

  // https://github.com/ralight/usb-reset/blob/master/usb-reset.c
  auto handle = libusb_open_device_with_vid_pid(NULL, vid_, pid_);
  if (!handle) {
    tools::logger()->warn("Unable to open usb!");
    return;
  }

  if (libusb_reset_device(handle))
    tools::logger()->warn("Unable to reset usb!");
  else
    tools::logger()->info("Reset usb successfully :)");

  libusb_close(handle);
}

}  // namespace io