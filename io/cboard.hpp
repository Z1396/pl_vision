// 头文件保护宏：防止头文件被多次包含，避免类/枚举/函数重复定义导致的编译错误
// 命名规则：模块名__文件名_HPP（大写，双层下划线分隔，区分命名空间与文件名）
#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

// 引入Eigen几何模块：用于四元数、矩阵等几何运算，处理IMU姿态数据
#include <Eigen/Geometry>
// 引入chrono库：高精度时间戳，记录IMU数据采集时间、实现时间相关逻辑
#include <chrono>
// 引入数学库：基础数学运算（如三角函数、绝对值等）
#include <cmath>
// 引入函数包装器：支持可调用对象的存储与传递（若后续扩展回调逻辑使用）
#include <functional>
// 引入字符串库：处理设备名称、配置路径、模式字符串转换等
#include <string>
// 引入向量容器：存储模式名称列表、批量数据等
#include <vector>

// 引入项目内部依赖头文件（按模块划分，保证依赖最小化）
#include "io/command.hpp"    // IO模块命令类：定义需要发送给下位机的控制指令结构
#include "io/socketcan.hpp"  // IO模块SocketCAN类：封装CAN总线底层通信，实现CAN帧的收发
#include "tools/logger.hpp"  // 工具模块日志类：实现日志打印（INFO/WARN/ERROR），便于问题排查
#include "tools/thread_safe_queue.hpp"  // 工具模块线程安全队列：多线程下安全传递IMU数据，避免数据竞争

// IO模块命名空间：隔离IO相关类/枚举/函数，避免与项目其他模块（如算法、工具）命名冲突
namespace io
{
// 定义系统工作模式枚举：表征整机的核心工作状态，由上位机决策或人工设定
// 枚举值为全小写命名，符合C++枚举命名规范，与下方MODES字符串列表一一对应
enum Mode
{
    idle,        // 空闲模式：设备上电初始化完成，无任务执行，等待指令
    auto_aim,    // 自动瞄准模式：核心作战模式，自动识别目标并完成瞄准跟踪
    small_buff,  // 小能量机关模式：针对小能量机关目标的专用作战模式
    big_buff,    // 大能量机关模式：针对大能量机关目标的专用作战模式
    outpost      // 前哨模式：针对前哨站目标的专用作战模式
};
// 工作模式字符串映射表：将枚举值转换为可读字符串，便于日志打印、调试输出、用户交互
// 顺序与Mode枚举完全一致，通过枚举值作为下标可直接获取对应字符串
const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff", "big_buff", "outpost"};

// 哨兵机器人专有射击模式枚举：表征哨兵机器人的发射机构工作方式（单/双管发射）
// 仅适用于哨兵机型，其他机型可忽略该枚举
enum ShootMode
{
  left_shoot,   // 左管单独射击：仅控制左侧发射机构工作
  right_shoot,  // 右管单独射击：仅控制右侧发射机构工作
  both_shoot    // 双管同时射击：左右两侧发射机构同步工作
};
// 射击模式字符串映射表：将ShootMode枚举转换为可读字符串，便于日志与调试
// 顺序与ShootMode枚举完全一致，一一映射
const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot", "both_shoot"};

/**
 * @brief CAN板卡通信核心类（CBoard = CAN Board）
 * @details 封装与下位机CAN板卡的所有通信逻辑，实现「上位机<->CAN板卡」的数据交互：
 *          1. 接收CAN板卡上传的IMU姿态数据、弹速数据等传感器信息；
 *          2. 发送上位机的控制指令（工作模式、射击指令、云台角度等）到CAN板卡；
 *          3. 对IMU数据进行缓存与时间戳匹配，提供姿态数据的查询接口；
 *          4. 封装CAN总线底层细节，为上层业务提供简洁的调用接口。
 * @note 该类是IO模块与下位机通信的核心入口，所有与CAN板卡的交互均通过此类完成
 * @note 支持多线程场景：IMU数据接收与查询分离，通过线程安全队列保证数据安全
 */
class CBoard
{
public:
  // 公有成员变量：需上层业务修改/读取的核心状态参数（对外提供操作接口）
  double bullet_speed;  // 当前弹丸速度（单位：m/s），由CAN板卡上传，上层算法（如弹道解算）需使用
  Mode mode;            // 当前系统工作模式，由上位机决策设置，下发至CAN板卡
  ShootMode shoot_mode; // 哨兵专有射击模式，由上位机控制，下发至CAN板卡发射机构
  double ft_angle;      // 无人机专有云台角度（ft = Fuselage/Turret），无人机机型的云台姿态控制参数

  /**
   * @brief 构造函数：初始化CAN板卡通信对象
   * @param config_path CAN板卡配置文件路径（yaml格式），包含CAN设备名、各帧ID、波特率等配置
   * @note 初始化流程：读取yaml配置 → 初始化SocketCAN通信 → 注册CAN帧回调函数 → 初始化成员变量
   * @note 成员初始化顺序严格遵循类内私有成员声明顺序（避免未定义行为）
   */
  CBoard(const std::string & config_path);

  /**
   * @brief 根据时间戳查询对应时刻的IMU姿态数据（四元数）
   * @param timestamp 目标查询时间戳（稳态时钟），由上层业务传入（如图像采集时间戳）
   * @return Eigen::Quaterniond 对应时刻的IMU四元数，表征设备的三维姿态（旋转）
   * @note Eigen::Quaterniond：Eigen库双精度浮点型四元数，用于无万向节锁的姿态表示，优于欧拉角
   * @note 核心功能：实现「时间戳同步」，匹配图像/其他传感器与IMU的采集时间，保证数据时序一致性
   * @note 线程安全：内部通过线程安全队列读取IMU数据，支持多线程调用
   */
  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  /**
   * @brief 发送控制指令到CAN板卡
   * @param command 待发送的控制指令对象（Command类实例），包含模式、角度、射击等控制信息
   * @note const成员函数：仅发送数据，不修改当前CBoard对象的任何成员变量，保证const正确性
   * @note 底层调用SocketCAN的发送接口，将Command对象封装为CAN帧后下发至下位机
   * @note 线程安全：SocketCAN内部保证CAN帧发送的原子性，支持多线程调用
   */
  void send(Command command) const;

private:
  /**
   * @brief IMU数据结构体：封装单帧IMU的姿态数据与采集时间戳
   * @details 作为IMU数据的最小存储单元，用于线程安全队列的缓存与传递
   */
  struct IMUData
  {
    Eigen::Quaterniond q;   // IMU四元数：表征设备当前的三维姿态（滚转/俯仰/偏航）
    std::chrono::steady_clock::time_point timestamp;  // 数据采集时间戳：稳态时钟（不受系统时间修改影响），保证时间一致性
  };

  // 私有成员变量：类内部使用的核心数据与通信对象，禁止上层业务直接修改（封装性）
  tools::ThreadSafeQueue<IMUData> queue_;  // IMU数据线程安全队列：缓存CAN板卡上传的IMU数据，实现「接收线程<->查询线程」的安全数据传递
                                           // 初始化顺序：必须在SocketCAN can_之前初始化！否则CAN接收回调中访问未初始化的队列会导致死锁/程序崩溃
  SocketCAN can_;                          // CAN总线通信对象：封装CAN底层收发逻辑，是与CAN板卡通信的核心句柄
  IMUData data_ahead_;                     // 前向IMU数据缓存：存储最新的IMU数据（时间戳较新），用于时间戳匹配的前向插值
  IMUData data_behind_;                    // 后向IMU数据缓存：存储上一帧IMU数据（时间戳较旧），用于时间戳匹配的后向插值

  // CAN帧ID配置：从yaml配置文件读取，表征不同类型数据的CAN帧标识，用于解析CAN板卡上传的数据
  int quaternion_canid_;   // IMU四元数数据对应的CAN帧ID，解析姿态数据时匹配该ID
  int bullet_speed_canid_; // 弹丸速度数据对应的CAN帧ID，解析弹速数据时匹配该ID
  int send_canid_;         // 上位机下发控制指令的CAN帧ID，发送Command时使用该ID

  /**
   * @brief CAN帧接收回调函数：SocketCAN接收到CAN帧后自动调用此函数
   * @param frame 接收到的原始CAN帧数据（can_frame为Linux SocketCAN原生结构体）
   * @details 核心功能：解析CAN帧数据 → 按帧ID分类处理（IMU四元数/弹速）→ 封装为对应数据结构 → 存入线程安全队列/更新成员变量
   * @note 回调函数运行在SocketCAN的接收线程中，需保证执行效率，避免耗时操作
   * @note 线程安全：对共享资源的访问通过线程安全队列或加锁保证，避免数据竞争
   */
  void callback(const can_frame & frame);

  /**
   * @brief 读取YAML配置文件：解析CAN板卡的相关配置参数
   * @param config_path YAML配置文件的绝对/相对路径
   * @return std::string 解析后的CAN设备名（如"can0"），用于初始化SocketCAN
   * @details 解析内容：CAN设备名、各类型数据的CAN帧ID、波特率等配置，并赋值给类内对应的成员变量（如quaternion_canid_）
   * @note 若配置文件读取失败/解析错误，会通过logger打印ERROR日志并终止程序
   */
  std::string read_yaml(const std::string & config_path);
};

}  // namespace io

#endif  // IO__CBOARD_HPP
