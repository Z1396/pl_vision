/**
 * @file io/hikrobot.hpp
 * @brief 海康机器人USB相机驱动实现类头文件
 * @details 继承自相机基类CameraBase，基于海康机器人MvCameraControl SDK实现USB相机的图像采集、参数配置、线程安全数据缓存功能
 * @note 依赖海康机器人MvCameraControl.h SDK头文件，编译时需链接对应库文件
 */
// 防止头文件重复包含的宏定义（双下划线分隔命名空间/类，避免命名冲突）
#ifndef IO__HIKROBOT_HPP
#define IO__HIKROBOT_HPP

// 标准库头文件包含
#include <atomic>           // 原子变量，用于多线程无锁同步（线程安全的布尔标志）
#include <chrono>           // 时间戳相关，用于记录图像采集时间
#include <opencv2/opencv.hpp> // OpenCV库，用于图像数据存储和处理
#include <string>           // 字符串处理，用于设备VID/PID、参数名等
#include <thread>           // 多线程，用于相机采集线程、守护线程

// 第三方/项目内部头文件包含
#include "MvCameraControl.h"    // 海康机器人相机SDK核心头文件，提供相机控制API
#include "io/camera.hpp"        // 项目相机基类头文件，定义相机通用接口
#include "tools/thread_safe_queue.hpp" // 项目线程安全队列工具，用于采集图像的缓存队列

// 项目命名空间，封装IO相关模块（输入输出设备：相机、串口、网口等）
namespace io
{
/**
 * @class HikRobot
 * @brief 海康机器人USB相机驱动实现类
 * @details 继承自抽象基类CameraBase，实现基类定义的相机通用接口，基于海康SDK完成
 *          相机初始化、参数配置（曝光、增益、VID/PID）、图像采集、线程管理等功能
 * @note 采用多线程采集+线程安全队列缓存设计，保证图像采集的实时性和线程安全
 */
class HikRobot : public CameraBase
{
public:
  /**
   * @brief 构造函数，相机初始化核心接口
   * @param exposure_ms 曝光时间，单位：毫秒(ms)
   * @param gain 相机增益，无单位（海康SDK通用增益值，通常1.0~32.0）
   * @param vid_pid USB设备的VID/PID组合字符串，格式如"0x1234:0x5678"，用于匹配指定相机
   * @details 完成相机设备查找、打开、句柄获取、曝光/增益/VID/PID等参数配置
   *          初始化采集线程、守护线程相关的原子标志，创建线程安全队列
   */
  HikRobot(double exposure_ms, double gain, const std::string & vid_pid);

  /**
   * @brief 析构函数，重写基类虚析构
   * @override 重写CameraBase的虚析构函数
   * @details 保证多态析构时子类资源被完整释放，停止图像采集、销毁采集线程/守护线程
   *          关闭相机句柄、释放SDK相关资源，清空线程安全队列
   */
  ~HikRobot() override;

  /**
   * @brief 读取相机图像接口，重写基类纯虚函数
   * @override 重写CameraBase的纯虚read函数，实现具体的图像读取逻辑
   * @param[out] img 输出的图像数据，OpenCV的Mat格式（与相机采集格式匹配，通常为BGR8/GRAY8）
   * @param[out] timestamp 图像采集的时间戳，基于系统稳态时钟（steady_clock），不受系统时间修改影响
   * @details 从线程安全队列中取出最新的采集图像和对应时间戳，为上层业务提供图像数据
   *          若队列为空，可能会阻塞或返回空图像（具体实现由队列特性决定）
   */
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

private:
  /**
   * @struct CameraData
   * @brief 相机采集数据结构体，封装图像数据和对应采集时间戳
   * @details 作为线程安全队列的存储单元，实现图像和时间戳的绑定传输，保证数据一致性
   */
  struct CameraData
  {
    cv::Mat img;  ///< 采集到的图像数据，OpenCV Mat格式
    std::chrono::steady_clock::time_point timestamp; ///< 图像采集的精确时间戳
  };

  // 相机参数成员变量（私有，仅内部函数修改，保证参数封装性）
  double exposure_us_;  ///< 曝光时间，单位：微秒(us)（SDK底层以微秒为单位，构造函数将ms转换为us存储）
  double gain_;         ///< 相机增益值，与SDK接口匹配，由构造函数传入初始化

  // 线程管理相关成员变量
  std::thread daemon_thread_;    ///< 相机守护线程，用于监控相机状态、处理异常重连等
  std::atomic<bool> daemon_quit_;///< 守护线程退出标志，原子布尔型，多线程安全控制线程退出

  // 相机核心控制成员变量
  void * handle_;                ///< 相机设备句柄，海康SDK的核心标识，所有相机操作均基于此句柄
  std::thread capture_thread_;   ///< 相机图像采集线程，独立线程循环采集图像，避免阻塞主线程
  std::atomic<bool> capturing_;  ///< 采集状态标志，原子布尔型，表示当前是否正在采集图像
  std::atomic<bool> capture_quit_;///< 采集线程退出标志，原子布尔型，多线程安全控制采集线程退出
  tools::ThreadSafeQueue<CameraData> queue_; ///< 线程安全图像队列，缓存采集到的CameraData数据
                                             ///< 实现采集线程（生产者）和read接口（消费者）的解耦

  // USB设备标识成员变量
  int vid_;  ///< USB设备的VID（厂商ID），由vid_pid字符串解析而来，用于匹配指定相机
  int pid_;  ///< USB设备的PID（产品ID），由vid_pid字符串解析而来，用于匹配指定相机

  /**
   * @brief 启动图像采集线程，私有成员函数
   * @details 创建并启动capture_thread_采集线程，设置capturing_为true，
   *          线程内循环调用海康SDK采集接口，将图像和时间戳封装为CameraData后推入队列
   */
  void capture_start();

  /**
   * @brief 停止图像采集线程，私有成员函数
   * @details 设置capture_quit_为true，等待采集线程退出，设置capturing_为false，
   *          清空队列中的残留图像数据，保证资源干净释放
   */
  void capture_stop();

  /**
   * @brief 设置相机浮点型参数，私有工具函数
   * @param name 海康SDK对应的参数名（如"ExposureTime"、"Gain"等）
   * @param value 要设置的浮点型参数值
   * @details 封装海康SDK的浮点参数设置API，处理参数设置的异常情况，保证参数配置的安全性
   */
  void set_float_value(const std::string & name, double value);

  /**
   * @brief 设置相机枚举型参数，私有工具函数
   * @param name 海康SDK对应的枚举参数名（如"PixelFormat"、"TriggerMode"等）
   * @param value 要设置的枚举值（无符号整型，与SDK枚举定义匹配）
   * @details 封装海康SDK的枚举参数设置API，处理参数设置的异常情况
   */
  void set_enum_value(const std::string & name, unsigned int value);

  /**
   * @brief 解析并设置USB设备的VID/PID，私有工具函数
   * @param vid_pid 格式如"0x1234:0x5678"的字符串
   * @details 解析字符串中的VID和PID值，转换为整型后赋值给vid_和pid_，
   *          用于海康SDK的设备查找接口，精准匹配指定USB相机
   */
  void set_vid_pid(const std::string & vid_pid);

  /**
   * @brief 重置USB相机设备，私有const成员函数
   * @details 当相机采集异常时，调用该函数重置USB设备，尝试恢复相机正常工作，
   *          const修饰表示函数不修改类的任何成员变量
   */
  void reset_usb() const;
};

}  // namespace io

#endif  // IO__HIKROBOT_HPP  // 头文件保护宏结束
/**
 * @file tools/thread_safe_queue.hpp
 * @brief 线程安全队列类模板头文件
 * @details 提供通用型线程安全队列实现，支持多线程环境下的生产者-消费者模型
 *          基于std::queue结合互斥锁和条件变量，保证队列操作的原子性和线程安全
 */