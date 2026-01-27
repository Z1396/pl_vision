#ifndef IO__DAHUA_HPP
#define IO__DAHUA_HPP

#include <atomic>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <mutex>          // 互斥锁头文件
#include <condition_variable>  // 条件变量头文件（若用线程就绪等待）

// 引入大华SDK核心类型定义（需根据SDK安装目录调整头文件路径）
#include "GenICam/System.h"
#include "GenICam/Camera.h"
#include "GenICam/StreamSource.h"
#include "GenICam/Frame.h"
#include "GenICam/GigE/GigECamera.h"
#include "GenICam/GigE/GigEInterface.h"
#include "Infra/PrintLog.h"

#include "StreamRetrieve.h"

#include "Memory/SharedPtr.h"
#include "Infra/Thread.h"

#include "io/camera.hpp"
#include "tools/thread_safe_queue.hpp"

// 引入大华SDK命名空间（简化类型声明）
using namespace Dahua::GenICam;
using namespace Dahua::Infra;

namespace io
{
class DHUA : public CameraBase
{
public:

  Dahua::Memory::TSharedPtr<StreamRetrieve> streamThreadSptr;//线程指针
  /**
   * @brief 构造函数：初始化相机参数
   * @param exposure_ms 曝光时间（毫秒）
   * @param gain 增益值
   * @param vid_pid 相机VID:PID字符串（如"04B4:00F9"）
   */
  DHUA(double exposure_ms, double gain, const std::string & vid_pid);
  
  /**
   * @brief 析构函数：释放SDK资源与线程
   */
  ~DHUA() override;
  
  /**
   * @brief 读取图像数据（实现CameraBase纯虚函数）
   * @param img 输出图像（OpenCV Mat格式）
   * @param timestamp 输出图像采集时间戳
   */
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

    /**
   * @brief 辅助函数：大华CFrame转OpenCV Mat（适配手册3.5.4节“获取图像裸数据”与4.2节“OpenCV对象转换”）
   * @param frame 大华SDK图像帧对象（CFrame，手册3.5.4节）
   * @param img_rgb 输出RGB格式图像（OpenCV Mat）
   * @return 转换成功标志（true=成功，false=失败）
   */
  bool convert_dahua_frame_to_opencv(const CFrame & frame, cv::Mat & img_rgb);

  // 设置相机曝光时间
  void setCameraExposureTime(Dahua::GenICam:: ICameraPtr &cameraSptr, double exposureTimeSet, bool addFlag);

  
private:

    // ---------------------- 新增：互斥锁与条件变量成员 ----------------------
  std::mutex stream_mtx_;           // 保护 streamThreadSptr 跨线程访问的互斥锁
  std::mutex queue_mtx_;            // 保护 queue_ 读写的互斥锁
  std::mutex ready_mtx_;            // 线程就绪条件变量的互斥锁（若用条件变量）
  std::condition_variable ready_cv_;// 线程就绪条件变量（若用条件变量，替代固定sleep）
  /**
   * @brief 相机数据结构体：存储图像与时间戳（供线程安全队列使用）
   */
  struct CameraData
  {
    cv::Mat img;  // 图像数据（OpenCV格式）
    std::chrono::steady_clock::time_point timestamp;  // 采集时间戳
  };

  // -------------------------- 相机参数 --------------------------
  double exposure_us_;  // 曝光时间（微秒，对应手册3.4节属性配置）
  double gain_;         // 增益值（对应手册3.4节Gain属性）

  // -------------------------- 线程相关 --------------------------
  std::thread daemon_thread_;    // 守护线程（负责异常检测与恢复）
  std::atomic<bool> daemon_quit_;// 守护线程退出标志（原子变量确保线程安全）
  std::thread capture_thread_;   // 图像采集线程（负责从SDK获取图像）
  std::atomic<bool> capturing_;  // 采集状态标志（true=正在采集，对应手册3.5节采集流程）
  std::atomic<bool> capture_quit_;// 采集线程退出标志

  // -------------------------- SDK核心对象 --------------------------
  CSystem &system_sptr_;          // 大华SDK系统单例（手册3.1节，核心入口）
  ICameraPtr camera_sptr_;        // 相机智能指针（手册3.2-3.3节，设备发现与连接）
  IStreamSourcePtr stream_sptr_;  // 流对象智能指针（手册3.5.1节，图像采集通道）

  // -------------------------- 数据缓存 --------------------------
  tools::ThreadSafeQueue<CameraData> queue_;  // 线程安全队列（缓存采集到的图像数据）
  int vid_, pid_;                             // 相机VID/PID（用于USB重置，与SDK无关）

  // -------------------------- 私有成员函数 --------------------------
  /**
   * @brief 启动采集流程（对应手册3.5节“采集图像”完整流程）
   * 功能：设备发现→连接→参数配置→流对象创建→启动采集线程
   */
  void capture_start();
  
  /**
   * @brief 停止采集流程（对应手册3.5.3节“停止采集图像”）
   * 功能：停止采集线程→停止流采集→断开相机连接→释放SDK资源
   */
  void capture_stop();

  /**
   * @brief 设置浮点型属性（对应手册3.4节“读写属性-通用属性方法”）
   * @param name 属性名（如"ExposureTime"“Gain”，需与手册属性列表一致）
   * @param value 待设置的浮点值
   */
  void set_float_value(const std::string & name, double value);
  
  /**
   * @brief 设置枚举型属性（适配手册3.4.2节“枚举型属性”接口）
   * @param name 属性名（如"ExposureAuto"“GainAuto”）
   * @param value 属性值（字符串形式，如"Off""Continuous"，需与手册枚举值一致）
   */
  void set_enum_value(const std::string & name, const std::string & value);

  /**
   * @brief 解析VID:PID字符串（提取厂商ID与产品ID）
   * @param vid_pid 输入字符串（格式："XXXX:XXXX"，十六进制）
   */
  void set_vid_pid(const std::string & vid_pid);
  
  /**
   * @brief 重置USB设备（基于libusb，用于异常恢复）
   */
  void reset_usb() const;



  int32_t setResolution(Dahua::GenICam:: ICameraPtr& cameraSptr, int nWidth, int nHeight);
  //shark：自己写的设置相机增益

  static int32_t setGrabMode(Dahua::GenICam::ICameraPtr cameraSptr, bool bContious);
    // 获取相机采图模式
    // Get camera acquisition mode
    static int32_t getGrabMode(Dahua::GenICam:: ICameraPtr& cameraSptr, bool &bContious);
    // 软件触发
    // software trigger
    static int32_t triggerSoftware(Dahua::GenICam:: ICameraPtr& cameraSptr);
    // 设置传感器采样率（采集分辨率）
    // Set sensor sampling rate (acquisition resolution)
    
    // 获取传感器采样率
    // Get sensor sample rate
    static int32_t getResolution(Dahua::GenICam:: ICameraPtr& cameraSptr, int64_t &nWidth, int64_t &nHeight);
    // 设置binning (Off X Y XY)
    // set binning (Off X Y XY)
    static int32_t setBinning(Dahua::GenICam:: ICameraPtr& cameraSptr);
    // 获取传感器最大分辩率
    // get the maximum resolution of the sensor
    static int32_t getMaxResolution(Dahua::GenICam:: ICameraPtr& cameraSptr, int64_t &nWidthMax, int64_t &nHeightMax);
    // 设置图像ROI
    // set image ROI
    static int32_t setROI(Dahua::GenICam:: ICameraPtr& cameraSptr, int64_t nX, int64_t nY, int64_t nWidth, int64_t nHeight);
    // 获取图像ROI
    // get image ROI
    static int32_t getROI(Dahua::GenICam:: ICameraPtr& cameraSptr, int64_t &nX, int64_t &nY, int64_t &nWidth, int64_t &nHeight);
    // 获取采图图像宽度
    // Get the width of the image
    static int32_t getWidth(Dahua::GenICam:: ICameraPtr& cameraSptr, int64_t &nWidth);
    // 获取采图图像高度
    // Get the height of the image
    static int32_t getHeight(Dahua::GenICam:: ICameraPtr& cameraSptr, int64_t &nHeight);
    // 改动的设置曝光值(曝光、自动曝光/手动曝光)shark:这只有连续曝光和手动曝光，没有一次曝光
    // Set exposure value (exposure, auto exposure / manual exposure)
    static int32_t setExposureMODE(Dahua::GenICam:: ICameraPtr& cameraSptr, bool bAutoExposure );
    //shark:自己写的设置曝光值(addFlag：true添加模式)

    static void setGainRawValue(Dahua::GenICam:: ICameraPtr &cameraSptr,double gainRawSet ,bool addFlag);
    //shark：自己写的gamma设置
    void SetGamma(Dahua::GenICam:: ICameraPtr &cameraSptr, double gammaSet, bool addFlag);
    // 设置白平衡值（有三个白平衡值）
    // Set the white balance value ( three white balance values)
    static int32_t setBalanceRatio(Dahua::GenICam:: ICameraPtr& cameraSptr, double dRedBalanceRatio, double dGreenBalanceRatio, double dBlueBalanceRatio);
    // 获取白平衡值（有三个白平衡值)
    // Get white balance value (three white balance values)
    static int32_t getBalanceRatio(Dahua::GenICam:: ICameraPtr& cameraSptr, double &dRedBalanceRatio, double &dGreenBalanceRatio, double &dBlueBalanceRatio);
    // 获取白平衡值范围
    // Get white balance value range
    static int32_t getBalanceRatioMinMaxValue(Dahua::GenICam:: ICameraPtr& cameraSptr, double &dMinValue, double &dMaxValue);
    // 设置采图速度（秒帧数）
    // Set the acquisition speed (seconds\frames)
    static int32_t setAcquisitionFrameRate(Dahua::GenICam:: ICameraPtr& cameraSptr, double dFrameRate);
    // 获取采图速度（秒帧数）
    // Get the acquisition speed (seconds and frames)
    static int32_t getAcquisitionFrameRate(Dahua::GenICam:: ICameraPtr& cameraSptr, double &dFrameRate);
    // 保存参数
    // Save parameters
    static int32_t userSetSave(Dahua::GenICam:: ICameraPtr& cameraSptr);
    // 加载参数
    // Load parameters
    static int32_t loadUserSet(Dahua::GenICam:: ICameraPtr& cameraSptr);
    // 当相机与网卡处于不同网段时，自动设置相机IP与网卡处于同一网段 （与相机连接之前调用）
    // When the camera and the network card are in different network segments, automatically set the camera IP and the network card in the same network segment (before calling the camera).
    //static int32_t autoSetCameraIP(Dahua::GenICam:: ICameraPtr& cameraSptr);
    // 设置相机IP （与相机连接之前调用）
    // Set up camera IP (before calling with camera)
    static int32_t setCameraIp(Dahua::GenICam:: ICameraPtr& cameraSptr, char* ipAddress, char* subnetMask, char* gateway);
    // 设置相机静态IP （与相机连接之后调用）
    // Set camera static IP (after calling with camera)
    static int32_t setCameraPersistentIP(Dahua::GenICam:: ICameraPtr& cameraSptr);
    //shark:似乎是打印日志功能，但官方似乎只是给了个模板，让用户自己编写日志功能，这部分可以参考桂林理工中的日志部分（RMLOG.h）。后辈们们有空的话尽量完善，毕竟对维护代码，排除问题很有帮助
    void LogPrinterFunc(const char* log);
    // ********************** 这部分处理与SDK操作相机无关，用于显示设备列表 begin*****************************
    // ***********BEGIN: These functions are not related to API call and used to display device info***********
    static void displayDeviceInfo(Dahua::Infra::TVector<Dahua::GenICam:: ICameraPtr>& vCameraPtrList);
};

}  // namespace io

#endif  // IO__DAHUA_HPP