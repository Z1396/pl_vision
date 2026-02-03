// 头文件保护宏：防止重复包含（替代#pragma once，跨编译器兼容性更好）
// 命名规范：IO__GIMBAL_HPP，对应命名空间io+文件名gimbal.hpp，避免命名冲突
#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

// 引入核心依赖库：Eigen几何库、C++多线程/同步/时间库、串口通信库、自定义线程安全队列
#include <Eigen/Geometry>          // Eigen几何库：四元数（Eigen::Quaterniond）、位姿运算，处理云台旋转
#include <atomic>                   // 原子变量：保证多线程间布尔变量（如quit_）的无锁安全操作
#include <chrono>                   // 高精度时间库：时间戳记录、时间点同步，解决视觉-云台时间差
#include <mutex>                    // 互斥锁：保护多线程共享数据（如rx_data_、state_），避免数据竞争
#include <string>                   // 字符串：配置文件路径、串口名称等
#include <thread>                   // 多线程：创建独立的串口数据读取线程，不阻塞主线程
#include <tuple>                    // 元组：封装四元数+时间戳，存入线程安全队列

#include "serial/serial.h"          // 第三方串口通信库：实现跨平台串口读写（Windows/Linux/嵌入式）
#include "tools/thread_safe_queue.hpp"  // 自定义线程安全队列：缓存云台位姿+时间戳，供视觉端查询

// 命名空间：io，封装所有IO相关模块（相机、云台、串口等），避免全局命名污染
namespace io
{
// 云台→视觉 数据结构体：云台硬件发送给视觉端的状态数据，__attribute__((packed)) 强制紧凑内存布局
// 作用：定长二进制数据，直接与云台硬件的通信协议匹配，无内存对齐填充，保证串口字节流解析准确
struct __attribute__((packed)) GimbalToVision
{
  uint8_t head[2] = {'S', 'P'};    // 帧头标识：固定为'S'+'P'，用于串口数据帧同步、帧头校验
  uint8_t mode;                    // 云台当前工作模式：0-空闲, 1-自瞄, 2-小符, 3-大符（对应GimbalMode枚举）
  float q[4];                      // 云台当前姿态四元数：wxyz顺序（w实部，x/y/z虚部），表示云台旋转状态
  float yaw;                       // 云台横滚角（单位：弧度/度，由硬件协议定义）
  float yaw_vel;                   // 云台横滚角速度（单位：弧度/秒/度/秒）
  float pitch;                     // 云台俯仰角
  float pitch_vel;                 // 云台俯仰角速度
  float bullet_speed;              // 当前子弹速度（硬件测量/配置值，用于弹道补偿、预瞄计算）
  uint16_t bullet_count;           // 子弹累计发射次数：用于判断是否实际开火、统计发射量
  uint16_t crc16;                  // CRC16校验码：对整帧数据做校验，防止串口传输过程中数据丢失/篡改
};

// 静态断言：编译期检查结构体大小是否≤64字节
// 作用：1. 匹配云台硬件的通信协议帧长限制；2. 保证__attribute__((packed))生效，无内存对齐；3. 编译期发现结构体定义错误
static_assert(sizeof(GimbalToVision) <= 64);

// 视觉→云台 数据结构体：视觉端发送给云台硬件的控制指令，同样强制紧凑内存布局
struct __attribute__((packed)) VisionToGimbal
{
  uint8_t head[2] = {'S', 'P'};    // 帧头标识：与云台端一致，保证协议互通
  uint8_t mode;                    // 视觉端控制模式：0-不控制, 1-控制云台但不开火, 2-控制云台且开火
  float yaw;                       // 云台目标横滚角：视觉算法规划的目标角度
  float yaw_vel;                   // 云台目标横滚角速度：规划的角速度指令
  float yaw_acc;                   // 云台目标横滚角加速度：规划的角加速度指令，提升云台运动平滑性
  float pitch;                     // 云台目标俯仰角
  float pitch_vel;                 // 云台目标俯仰角速度
  float pitch_acc;                 // 云台目标俯仰角加速度
  uint16_t crc16;                  // CRC16校验码：云台端用于校验控制指令的完整性
};

// 静态断言：编译期校验控制指令结构体大小，匹配硬件协议
static_assert(sizeof(VisionToGimbal) <= 64);

// 云台工作模式枚举类：强类型封装云台模式，替代魔法数字，提升代码可读性和类型安全性
// 与GimbalToVision中的mode字段一一对应（0→IDLE，1→AUTO_AIM，2→SMALL_BUFF，3→BIG_BUFF）
enum class GimbalMode
{
  IDLE,        // 0：空闲模式，云台无自主运动，等待指令
  AUTO_AIM,    // 1：自瞄模式，云台跟随视觉检测的目标运动（核心自动瞄准模式）
  SMALL_BUFF,  // 2：小符模式，针对机器人比赛“小能量机关”的专用模式
  BIG_BUFF     // 3：大符模式，针对机器人比赛“大能量机关”的专用模式
};

// 云台状态结构体：封装云台核心状态数据，简化状态读取接口
// 作用：屏蔽GimbalToVision的协议细节，对外提供简洁、易读的状态数据，解耦协议层与业务层
struct GimbalState
{
  float yaw;            // 横滚角
  float yaw_vel;        // 横滚角速度
  float pitch;          // 俯仰角
  float pitch_vel;      // 俯仰角速度
  float bullet_speed;   // 子弹速度
  uint16_t bullet_count;// 累计发射次数
};

// 云台通信核心类：封装所有云台串口通信、状态管理、指令发送逻辑，对外提供简洁的业务接口
class Gimbal
{
public:
  // 构造函数：初始化云台通信
  // 参数：config_path - 配置文件路径（从yaml读取串口参数：串口号、波特率、超时时间等）
  Gimbal(const std::string & config_path);

  // 析构函数：释放云台通信资源，保证优雅退出
  ~Gimbal();

  // 公共接口：获取云台当前工作模式（const表示该函数不修改类成员变量）
  GimbalMode mode() const;
  // 公共接口：获取云台当前核心状态（返回GimbalState结构体，解耦协议细节）
  GimbalState state() const;
  // 公共接口：将GimbalMode枚举转换为字符串，用于日志打印、可视化显示
  std::string str(GimbalMode mode) const;
  // 公共接口：根据指定时间戳t，查询云台在该时刻的位姿四元数
  // 核心作用：时间戳同步，解决相机采集图像与云台状态的时间差，保证坐标解算准确性
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  // 公共接口：发送云台控制指令（重载1，业务层友好接口）
  // 参数：control-是否使能云台控制，fire-是否开火，yaw/pitch-目标角度/角速度/角加速度
  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc);

  // 公共接口：发送云台控制指令（重载2，底层协议接口）
  // 参数：已封装好的VisionToGimbal结构体，直接发送原始协议数据，适用于底层调试
  void send(io::VisionToGimbal VisionToGimbal);

private:
  // 私有成员：串口通信对象，封装串口打开、读写、波特率设置等底层操作
  serial::Serial serial_;

  // 私有成员：多线程与同步相关
  std::thread thread_;          // 串口数据读取线程：独立运行，持续读取云台发送的状态数据，不阻塞主线程
  std::atomic<bool> quit_ = false;  // 线程退出标志：原子变量，多线程安全控制读取线程退出
  mutable std::mutex mutex_;    // 互斥锁：保护多线程共享的成员变量（rx_data_、state_、mode_等），mutable表示const函数中也可加锁

  // 私有成员：协议数据缓存，与硬件通信的原始二进制数据
  GimbalToVision rx_data_;      // 接收数据缓存：存储从云台读取的原始状态数据
  VisionToGimbal tx_data_;      // 发送数据缓存：存储待发送给云台的原始控制指令

  // 私有成员：云台状态与模式缓存，业务层可直接使用的解析后数据
  GimbalMode mode_ = GimbalMode::IDLE;  // 云台当前工作模式，解析自rx_data_.mode
  GimbalState state_;           // 云台当前核心状态，解析自rx_data_，供外部通过state()接口获取
  // 私有成员：云台位姿时间戳队列，缓存历史位姿+对应时间戳
  // 模板参数：存储元组（四元数+时间点），队列容量1000，用于时间戳同步时的历史位姿查询
  tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
    queue_{1000};

  // 私有成员函数：底层串口读取，读取指定长度的二进制数据到缓冲区
  // 返回值：bool - 读取成功/失败，参数：buffer-数据缓冲区，size-要读取的字节数
  bool read(uint8_t * buffer, size_t size);
  // 私有成员函数：串口读取线程的主函数，持续执行「读取数据→解析状态→更新缓存」逻辑
  void read_thread();
  // 私有成员函数：串口重连机制，当串口断开时（如硬件掉线），自动尝试重新连接云台，提升通信可靠性
  void reconnect();
};

}  // 命名空间io结束

#endif  // 头文件保护宏结束

/*2. 关键设计亮点：面向工业级实时通信
（1）二进制定长协议 + 紧凑内存布局

    采用__attribute__((packed))强制结构体无内存对齐，保证二进制数据与云台硬件协议一字节不差；
    固定帧头SP+CRC16 校验，解决串口通信的帧同步、数据校验问题，防止传输丢包、篡改；
    static_assert编译期校验结构体大小，提前发现协议定义错误，避免运行时解析异常。

（2）独立读取线程，保证实时性

    创建专属的read_thread串口读取线程，与主线程（视觉检测）并行运行，不会因串口阻塞导致视觉算法卡顿；
    线程退出由原子变量quit_控制，无锁且多线程安全，避免退出时的资源竞争。

（3）完善的多线程同步机制

    互斥锁mutex_保护所有共享数据（状态、模式、协议缓存），避免多线程读写冲突；
    原子变量处理轻量级布尔标志（quit_），互斥锁处理复杂数据读写，兼顾性能与安全性；
    自定义线程安全队列queue_缓存位姿时间戳，无需额外加锁，简化跨线程数据传递。

（4）协议层与业务层解耦

    底层用GimbalToVision/VisionToGimbal对接硬件协议，上层用GimbalMode/GimbalState提供业务接口；
    外部仅需调用state()、send(control, fire, ...)等简洁接口，无需关心串口协议、数据解析细节，降低上层算法的集成成本。

（5）高可靠性设计

    提供reconnect()自动重连函数，串口断开时自动尝试恢复通信，适应工业现场的复杂环境；
    双重载send接口，既提供业务层友好的参数化接口，也保留底层原始协议接口，兼顾开发效率与调试灵活性；
    位姿时间戳队列缓存 1000 条历史数据，满足视觉算法的时间戳同步需求，避免因时间差导致的解算误差。*/
