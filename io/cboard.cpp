#include "cboard.hpp"

#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
/**
 * @brief CBoard类的构造函数，初始化控制板相关资源和状态
 * @param config_path YAML配置文件路径，用于加载CAN总线参数等配置
 */
CBoard::CBoard(const std::string & config_path)
// 初始化列表：按顺序初始化类成员变量
: mode(Mode::idle),                  // 初始化工作模式为空闲状态（Mode是自定义枚举）
  shoot_mode(ShootMode::left_shoot), // 初始化射击模式为左侧射击（ShootMode是自定义枚举）
  bullet_speed(0),                   // 初始化子弹速度为0
  queue_(5000),                      // 初始化数据队列，最大容量为5000（推测为线程安全队列）
  // 初始化CAN总线通信对象：
  // 1. 第一个参数：通过read_yaml函数读取配置文件，获取CAN初始化参数
  // 2. 第二个参数：绑定回调函数，当CAN接收数据时自动调用CBoard::callback
  can_(read_yaml(config_path), std::bind(&CBoard::callback, this, std::placeholders::_1))
// 注意: CAN回调函数可能在CBoard构造函数完全执行完毕前就被调用
// （因为can_在初始化列表中先于构造函数体初始化，若CAN立即收到数据会触发回调）
{
  // 输出日志：提示正在等待队列数据（"q"推测为queue_的简写）
  tools::logger()->info("[Cboard] Waiting for q...");
  
  // // 从队列中取出第一个数据，存入data_ahead_（推测为前方/前置传感器数据）
  queue_.pop(data_ahead_);
  // 从队列中取出第二个数据，存入data_behind_（推测为后方/后置传感器传感器数据）
  queue_.pop(data_behind_);
  
  // 输出日志：表示CBoard初始化完成，已成功启动
  tools::logger()->info("[Cboard] Opened.");
}

/**
 * @brief 根据指定时间戳，通过插值计算该时刻的IMU姿态（四元数）
 * @param timestamp 目标时间戳，需要计算该时刻的IMU姿态
 * @return 插值得到的目标时刻姿态（四元数，已归一化）
 * 说明：利用时间戳前后的两个IMU数据，通过球面线性插值（SLERP）计算目标时刻的姿态
 */
Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
    // 如果当前"后方数据"的时间戳早于目标时间戳，将其更新为"前方数据"
    // （确保data_ahead_始终是早于或等于目标时间戳的最新数据）
    if (data_behind_.timestamp < timestamp) 
        data_ahead_ = data_behind_;

    // 从队列中持续获取数据，直到找到时间戳晚于目标时间戳的"后方数据"
    while (true) 
    {
        queue_.pop(data_behind_);  // 从队列弹出一个数据，作为新的"后方数据"
        if (data_behind_.timestamp > timestamp)  // 找到晚于目标时间的第一个数据
            break;
        data_ahead_ = data_behind_;  // 若仍早于目标时间，则更新"前方数据"
    }

    // 获取前后两个时刻的姿态四元数，并确保其已归一化（四元数必须归一化才能正确表示姿态）
    Eigen::Quaterniond q_a = data_ahead_.q.normalized();   // 目标时间之前的姿态（较早时刻）
    Eigen::Quaterniond q_b = data_behind_.q.normalized();  // 目标时间之后的姿态（较晚时刻）
    
    // 提取三个关键时间点
    auto t_a = data_ahead_.timestamp;   // 前方数据的时间戳（t_a ≤ timestamp）
    auto t_b = data_behind_.timestamp;  // 后方数据的时间戳（t_b ≥ timestamp）
    auto t_c = timestamp;               // 目标时间戳

    // 计算时间间隔（转换为秒为单位的浮点数，用于插值比例计算）
    std::chrono::duration<double> t_ab = t_b - t_a;  // 前后数据的时间差（t_b - t_a）
    std::chrono::duration<double> t_ac = t_c - t_a;  // 目标时间与前方数据的时间差（t_c - t_a）

    // 计算插值比例k：目标时间在[t_a, t_b]区间内的相对位置（0 ≤ k ≤ 1）
    auto k = t_ac / t_ab;
    
    // 对前后姿态进行球面线性插值（SLERP），并归一化结果
    // SLERP是四元数插值的最优方法，能保持旋转的均匀性（避免欧拉角插值的万向锁问题）
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

    // 返回目标时间戳的插值姿态
    return q_c;
}

void CBoard::send(Command command) const
{
  can_frame frame;
  frame.can_id = send_canid_;
  frame.can_dlc = 8;
  frame.data[0] = (command.control) ? 1 : 0;
  frame.data[1] = (command.shoot) ? 1 : 0;
  frame.data[2] = (int16_t)(command.yaw * 1e4) >> 8;
  frame.data[3] = (int16_t)(command.yaw * 1e4);
  frame.data[4] = (int16_t)(command.pitch * 1e4) >> 8;
  frame.data[5] = (int16_t)(command.pitch * 1e4);
  frame.data[6] = (int16_t)(command.horizon_distance * 1e4) >> 8;
  frame.data[7] = (int16_t)(command.horizon_distance * 1e4);

  try 
  {
    can_.write(&frame);
  } catch (const std::exception & e) 
  {
    tools::logger()->warn("{}", e.what());
  }
}

/**
 * @brief CAN总线数据接收回调函数，处理特定ID的CAN帧数据
 * 
 * 该函数是CAN总线通信的核心处理接口，当接收到CAN帧时被触发，根据CAN帧的ID区分数据类型（四元数(quaternion)、子弹速度等），
 * 解析数据并进行合法性校验，最终将有效数据存入队列或更新成员变量，同时控制日志输出频率避免刷屏。
 * 
 * @param frame 接收到的CAN帧数据（包含can_id和8字节数据域data）
 */
void CBoard::callback(const can_frame & frame)
{
    // 记录当前数据接收的时间戳（高精度时钟，用于后续时间相关计算）
    auto timestamp = std::chrono::steady_clock::now();

    // 1. 处理四元数数据帧（CAN ID匹配quaternion_canid_）
    if (frame.can_id == quaternion_canid_) 
    {
        // 1.1 解析CAN帧数据域中的四元数（w, x, y, z）
        // CAN数据为8字节，每2字节表示一个16位有符号整数，通过移位和或运算拼接  
        // 除以1e4：将整数表示的固定点小数转换为浮点数（如传感器输出的四元数通常以1e4为缩放因子）       
        auto x = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e4;  // x分量（数据0-1字节）
        auto y = (int16_t)(frame.data[2] << 8 | frame.data[3]) / 1e4;  // y分量（数据2-3字节）
        auto z = (int16_t)(frame.data[4] << 8 | frame.data[5]) / 1e4;  // z分量（数据4-5字节）
        auto w = (int16_t)(frame.data[6] << 8 | frame.data[7]) / 1e4;  // w分量（数据6-7字节）

        // 1.2 四元数合法性校验：四元数模长应接近1（w² + x² + y² + z² ≈ 1）
        // 若偏差超过0.01，则判定为无效数据，记录警告日志并返回
        if (std::abs(x * x + y * y + z * z + w * w - 1) > 1e-2) 
        {
            tools::logger()->warn("Invalid q: {} {} {} {}", w, x, y, z);  // 输出无效四元数
            return;
        }

        // 1.3 将有效四元数和时间戳存入队列（供后续姿态解算等模块使用）
        queue_.push({{w, x, y, z}, timestamp});  // 假设队列元素为{四元数, 时间戳}的结构体
    }

    // 2. 处理子弹速度及模式数据帧（CAN ID匹配bullet_speed_canid_）
    else if (frame.can_id == bullet_speed_canid_) 
    {
        // 2.1 解析子弹速度（数据0-1字节，16位有符号整数，除以1e2转换为m/s）
        bullet_speed = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e2;
        // 解析当前模式（数据2字节，转换为Mode枚举类型）
        mode = Mode(frame.data[2]);
        // 解析射击模式（数据3字节，转换为ShootMode枚举类型）
        shoot_mode = ShootMode(frame.data[3]);
        // 解析云台反馈角度（数据4-5字节，16位有符号整数，除以1e4转换为弧度）
        ft_angle = (int16_t)(frame.data[4] << 8 | frame.data[5]) / 1e4;

        // 2.2 限制日志输出频率为1Hz（避免高频数据刷屏，仅每秒输出一次有效信息）
        static auto last_log_time = std::chrono::steady_clock::time_point::min();  // 静态变量，记录上次日志时间
        auto now = std::chrono::steady_clock::now();  // 当前时间

        // 若子弹速度有效（>0）且距离上次日志已超过1秒，则输出信息
        if (bullet_speed > 0 && tools::delta_time(now, last_log_time) >= 1.0) 
        {
            tools::logger()->info(
                "[CBoard] Bullet speed: {:.2f} m/s, Mode: {}, Shoot mode: {}, FT angle: {:.2f} rad",
                bullet_speed,          // 子弹速度（保留2位小数）
                MODES[mode],           // 模式枚举对应的字符串（假设MODES是Mode到字符串的映射数组）
                SHOOT_MODES[shoot_mode],  // 射击模式枚举对应的字符串
                ft_angle               // 云台反馈角度（保留2位小数）
            );
            last_log_time = now;  // 更新上次日志时间为当前时间
        }
    }

    // （注：可根据需要添加更多else if分支，处理其他CAN ID的帧数据）
}

// 实现方式有待改进
std::string CBoard::read_yaml(const std::string & config_path)
{
  auto yaml = tools::load(config_path);

  quaternion_canid_ = tools::read<int>(yaml, "quaternion_canid");
  bullet_speed_canid_ = tools::read<int>(yaml, "bullet_speed_canid");
  send_canid_ = tools::read<int>(yaml, "send_canid");

  if (!yaml["can_interface"]) 
  {
    throw std::runtime_error("Missing 'can_interface' in YAML configuration.");
  }

  return yaml["can_interface"].as<std::string>();
}

}  // namespace io