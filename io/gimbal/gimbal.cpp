// 引入云台模块头文件（类/结构体/接口声明）
#include "gimbal.hpp"

// 引入项目自定义工具库：CRC校验、日志、数学工具、YAML配置解析
#include "tools/crc.hpp"        // CRC16校验工具：帧数据完整性校验，防止传输篡改/丢包
#include "tools/logger.hpp"     // 日志工具：分级打印（INFO/WARN/ERROR/DEBUG），便于问题排查
#include "tools/math_tools.hpp" // 数学工具：时间差计算、四元数插值等通用数学操作
#include "tools/yaml.hpp"       // YAML配置解析：从配置文件读取串口参数（串口号、波特率等）

// 命名空间：io，与头文件保持一致，封装IO相关实现
namespace io
{
// 构造函数：初始化云台串口通信、启动读取线程、完成初始位姿获取
// 参数：config_path - YAML配置文件路径，包含串口、云台相关配置项
Gimbal::Gimbal(const std::string & config_path)
{
  // 加载YAML配置文件，解析为可读取的yaml对象
  auto yaml = tools::load(config_path);
  // 从配置文件读取串口号（如"/dev/ttyUSB0"、"COM3"），跨平台兼容
  auto com_port = tools::read<std::string>(yaml, "com_port");

  try 
  {
    // 配置串口参数并尝试打开串口
    serial_.setPort(com_port);  // 设置串口号
    serial_.open();             // 打开串口（默认使用配置文件/库默认的波特率/数据位/校验位）
  } catch (const std::exception & e) 
  {
    // 串口打开失败：打印ERROR级日志，直接退出程序（云台是核心硬件，不可缺少）
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  // 启动云台串口读取线程：将Gimbal类的read_thread成员函数作为线程入口，this为类实例指针
  // 独立线程持续读取云台数据，不阻塞主线程（视觉检测/规划），保证实时性
  thread_ = std::thread(&Gimbal::read_thread, this);

  // 阻塞等待并弹出队列中第一个云台位姿数据，确保程序启动后已获取有效云台位姿
  queue_.pop();
  // 打印INFO级日志，标识云台初始化完成，已获取首个位姿数据
  tools::logger()->info("[Gimbal] First q received.");
}

// 析构函数：优雅释放云台通信资源，保证程序退出时无资源泄漏
Gimbal::~Gimbal()
{
  quit_ = true;  // 设置原子退出标志，通知读取线程退出循环
  if (thread_.joinable()) 
  {  // 检查线程是否可连接（未退出）
    thread_.join();          // 阻塞等待读取线程执行完毕，避免线程泄漏
  }
  serial_.close();  // 关闭串口，释放硬件资源
}

// 公共接口实现：获取云台当前工作模式
// const修饰：函数不修改类成员变量；mutable互斥锁可在const函数中使用
GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);  // 加锁保护共享变量mode_，防止多线程读写冲突
  return mode_;                              // 返回当前云台模式
}

// 公共接口实现：获取云台当前核心状态（位姿、速度、子弹信息等）
GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);  // 加锁保护共享变量state_，多线程安全
  return state_;                             // 返回拷贝后的云台状态，避免外部直接修改成员变量
}

// 公共接口实现：将云台模式枚举转换为字符串，便于日志打印/可视化显示
std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) 
  {
    case GimbalMode::IDLE:        return "IDLE";        // 空闲模式
    case GimbalMode::AUTO_AIM:    return "AUTO_AIM";    // 自瞄模式（核心）
    case GimbalMode::SMALL_BUFF:  return "SMALL_BUFF";  // 小符模式
    case GimbalMode::BIG_BUFF:    return "BIG_BUFF";    // 大符模式
    default:                      return "INVALID";     // 无效模式，容错处理
  }
}

// 核心公共接口实现：根据指定时间戳t，插值获取云台在该时刻的位姿四元数
// 解决相机采集图像与云台状态的「时间差问题」，保证坐标解算的准确性
// 参数：t - 相机采集图像的高精度时间戳
// 返回：t时刻云台的位姿四元数（单位化，保证旋转有效性）
Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) 
  {  // 循环插值，直到找到匹配的时间区间
    // 弹出队列中前一个位姿数据：q_a-前一时刻四元数，t_a-前一时刻时间戳
    auto [q_a, t_a] = queue_.pop();
    // 获取队列中后一个位姿数据（不弹出）：q_b-后一时刻四元数，t_b-后一时刻时间戳
    auto [q_b, t_b] = queue_.front();
    
    // 计算时间差：t_ab-前后两个云台位姿的时间间隔；t_ac-前一云台时刻到目标图像时刻的时间间隔
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    // 计算插值系数k：0~1之间，表示目标时刻在前后两个云台时刻的时间占比
    auto k = t_ac / t_ab;
    
    // 四元数球面线性插值（slerp）：在q_a和q_b之间按比例k插值，得到t时刻的位姿
    // normalized()：单位化四元数，避免插值后模长偏离1导致旋转错误
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    
    // 时间戳匹配判断：
    if (t < t_a) return q_c;  // 目标时刻早于前一云台时刻，直接返回插值结果
    if (!(t_a < t && t <= t_b)) continue;  // 目标时刻不在当前时间区间，继续循环查找
    
    return q_c;  // 找到匹配区间，返回插值后的t时刻云台位姿
  }
}

// 控制指令发送接口实现（重载1）：直接发送已封装的VisionToGimbal协议结构体
// 适用于底层调试/自定义协议封装，直接对接硬件协议格式
// 参数：VisionToGimbal - 已填充的视觉→云台控制指令结构体
void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  // 将传入的指令数据拷贝到类内发送缓存tx_data_
  tx_data_.mode = VisionToGimbal.mode;
  tx_data_.yaw = VisionToGimbal.yaw;
  tx_data_.yaw_vel = VisionToGimbal.yaw_vel;
  tx_data_.yaw_acc = VisionToGimbal.yaw_acc;
  tx_data_.pitch = VisionToGimbal.pitch;
  tx_data_.pitch_vel = VisionToGimbal.pitch_vel;
  tx_data_.pitch_acc = VisionToGimbal.pitch_acc;
  
  // 计算并设置CRC16校验码：校验范围为除crc16字段外的所有数据
  // reinterpret_cast：将结构体指针转为uint8_t*，按字节遍历计算CRC
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try 
  {
    // 将tx_data_按二进制字节流写入串口，发送给云台硬件
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) 
  {
    // 串口写入失败：打印WARN级日志，不退出程序（尝试重发/后续恢复）
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

// 控制指令发送接口实现（重载2）：业务层友好接口，按参数封装指令并发送
// 上层算法（MPC规划）直接调用，无需关心协议细节，解耦业务层与协议层
// 参数：control-是否使能云台控制；fire-是否开火；yaw/pitch-目标角度/角速度/角加速度
void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  // 根据control和fire参数，设置协议中的控制模式：
  // 0-不控制；1-控制云台不开火；2-控制云台并开火（与硬件协议定义一致）
  tx_data_.mode = control ? (fire ? 2 : 1) : 0;
  // 填充云台控制参数：目标角度、角速度、角加速度
  tx_data_.yaw = yaw;
  tx_data_.yaw_vel = yaw_vel;
  tx_data_.yaw_acc = yaw_acc;
  tx_data_.pitch = pitch;
  tx_data_.pitch_vel = pitch_vel;
  tx_data_.pitch_acc = pitch_acc;
  
  // 计算CRC16校验码，逻辑同重载1
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try 
  {
    // 二进制发送控制指令到云台
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) 
  {
    // 写入失败打日志，容错处理
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

// 私有成员函数：底层串口读取，读取指定长度的二进制数据到缓冲区
// 封装serial库的read接口，增加异常捕获和返回值判断，简化上层调用
// 参数：buffer-数据缓冲区指针；size-需要读取的字节数
// 返回：bool - 读取成功（实际读取字节数==指定size）/失败
bool Gimbal::read(uint8_t * buffer, size_t size)
{
  try 
  {
    // 调用serial库读取数据，返回实际读取字节数，判断是否等于期望长度
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) 
  {
    // 读取异常：静默处理（不打日志，避免错误过多刷屏），返回读取失败
    // tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    return false;
  }
}

// 核心私有成员函数：串口读取线程主函数，独立线程持续运行
// 功能：帧头校验→数据读取→CRC16校验→数据解析→状态更新→位姿缓存，实现云台状态的实时获取
void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");  // 线程启动日志
  int error_count = 0;  // 连续读取错误计数器，用于触发串口重连

  // 线程主循环：直到收到退出标志quit_=true
  while (!quit_) {
    // 连续错误超过5000次，判定为串口通信异常，触发重连机制
    if (error_count > 5000) 
    {
      error_count = 0;  // 重置错误计数器
      tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();      // 调用重连函数，尝试恢复串口通信
      continue;         // 重连后继续循环，重新开始读取
    }

    // 第一步：读取帧头（2字节，固定为'S'+'P'），做帧同步
    if (!read(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_.head))) 
    {
      error_count++;  // 帧头读取失败，错误计数+1
      continue;       // 跳过本次循环，重新尝试读取
    }

    // 帧头校验：不是'SP'则为无效帧，直接跳过，继续读取下一组数据
    if (rx_data_.head[0] != 'S' || rx_data_.head[1] != 'P') continue;

    // 帧头校验通过，记录当前数据接收的高精度时间戳（用于时间同步）
    auto t = std::chrono::steady_clock::now();

    // 第二步：读取帧头之外的剩余数据，完成整帧数据的读取
    if (!read(
          reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.head),  // 读取起始地址：帧头后
          sizeof(rx_data_) - sizeof(rx_data_.head))) 
    {                     // 读取长度：剩余数据字节数
      error_count++;  // 数据读取失败，错误计数+1
      continue;
    }

    // 第三步：CRC16完整性校验，防止串口传输过程中数据篡改、丢包、字节错位
    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) 
    {
      tools::logger()->debug("[Gimbal] CRC16 check failed.");  // 校验失败打DEBUG日志
      continue;                                                 // 跳过无效帧
    }

    // 整帧数据读取+校验成功，重置错误计数器
    error_count = 0;
    
    // 解析云台位姿四元数：从rx_data_中读取wxyz顺序的四元数数据，构造Eigen四元数对象
    Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
    // 将四元数+对应时间戳推入线程安全队列，供q(t)接口插值使用，实现时间同步
    queue_.push({q, t});

    // 第四步：解析并更新云台状态和模式（共享变量，需加锁保护）
    std::lock_guard<std::mutex> lock(mutex_);  // 加锁，保证多线程读写安全

    // 解析云台核心状态：将协议数据拷贝到state_结构体，供外部state()接口获取
    state_.yaw = rx_data_.yaw;
    state_.yaw_vel = rx_data_.yaw_vel;
    state_.pitch = rx_data_.pitch;
    state_.pitch_vel = rx_data_.pitch_vel;
    state_.bullet_speed = rx_data_.bullet_speed;
    state_.bullet_count = rx_data_.bullet_count;

    // 解析云台工作模式：将协议中的数字模式转换为GimbalMode枚举，增强类型安全性
    switch (rx_data_.mode) {
      case 0: mode_ = GimbalMode::IDLE; break;
      case 1: mode_ = GimbalMode::AUTO_AIM; break;
      case 2: mode_ = GimbalMode::SMALL_BUFF; break;
      case 3: mode_ = GimbalMode::BIG_BUFF; break;
      default:  // 无效模式容错：设为空闲模式，并打印WARN日志
        mode_ = GimbalMode::IDLE;
        tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
        break;
    }
  }

  // 线程退出循环，打印结束日志
  tools::logger()->info("[Gimbal] read_thread stopped.");
}

// 私有成员函数：串口自动重连机制，当通信异常时尝试恢复连接
// 提升系统鲁棒性，适应工业现场/比赛环境中的硬件临时掉线、接触不良等问题
void Gimbal::reconnect()
{
  int max_retry_count = 10;  // 最大重连尝试次数，避免无限重试
  // 循环重连：最多尝试10次，或收到退出标志则停止
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try 
    {
      serial_.close();  // 先尝试关闭串口，释放资源
      std::this_thread::sleep_for(std::chrono::seconds(1));  // 延时1秒，等待硬件释放
    } catch (...) 
    {
      // 关闭串口异常：静默处理，不影响后续重连
    }

    try 
    {
      serial_.open();  // 尝试重新打开串口
      queue_.clear();  // 清空历史位姿队列，避免旧数据干扰
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;  // 重连成功，退出循环
    } catch (const std::exception & e) 
    {
      // 重连失败：打印WARN日志，延时1秒后继续下一次尝试
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // 命名空间io结束
