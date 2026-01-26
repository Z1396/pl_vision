#include "commandgener.hpp"  // 包含当前类的头文件

#include "tools/math_tools.hpp"  // 包含数学工具函数（如平方、时间差计算）

namespace auto_aim
{
namespace multithread
{

/**
 * 构造函数：初始化命令生成器并启动独立线程
 * @param shooter 发射器对象引用（用于判断发射时机）
 * @param aimer 瞄准器对象引用（用于计算云台控制角度）
 * @param cboard 控制板对象引用（用于向下位机发送控制命令）
 * @param plotter 绘图工具引用（用于调试数据可视化）
 * @param debug 是否启用调试模式（决定是否输出调试数据）
 */
CommandGener::CommandGener(
  auto_aim::Shooter & shooter, auto_aim::Aimer & aimer, io::CBoard & cboard,
  tools::Plotter & plotter, bool debug)
// 初始化成员变量：绑定外部传入的对象引用，设置线程停止标志和调试模式
: shooter_(shooter), aimer_(aimer), cboard_(cboard), plotter_(plotter), stop_(false), debug_(debug)
{
  // 启动命令生成线程：绑定到generate_command成员函数，传入当前对象指针
  thread_ = std::thread(&CommandGener::generate_command, this);
}

/**
 * 析构函数：安全停止线程并释放资源
 * 避免线程在对象销毁后仍访问无效内存
 */
CommandGener::~CommandGener()
{
  {
    // 加锁修改线程停止标志（lock_guard自动释放锁）
    std::lock_guard<std::mutex> lock(mtx_);
    stop_ = true;  // 设置停止标志，让线程退出循环
  }
  cv_.notify_all();  // 唤醒可能阻塞在条件变量上的线程
  if (thread_.joinable())  // 若线程可join（未被join过）
    thread_.join();        // 等待线程执行完毕，避免僵尸线程
}

/**
 * 推送目标数据到命令生成器
 * 供外部（如主线程）调用，将最新的目标信息、时间戳等传入
 * @param targets 追踪到的目标列表（包含目标状态，如位置、速度）
 * @param t 目标数据对应的时间戳（用于时间同步）
 * @param bullet_speed 当前子弹速度（用于弹道解算）
 * @param gimbal_pos 云台当前姿态（欧拉角，用于瞄准补偿）
 */
void CommandGener::push(
  const std::list<auto_aim::Target> & targets, const std::chrono::steady_clock::time_point & t,
  double bullet_speed, const Eigen::Vector3d & gimbal_pos)
{
  // 加锁保护共享数据latest_（避免多线程读写冲突）
  std::lock_guard<std::mutex> lock(mtx_);
  // 将传入的参数封装为Input结构体，更新最新数据
  latest_ = {targets, t, bullet_speed, gimbal_pos};
  cv_.notify_one();  // 唤醒命令生成线程，通知有新数据待处理
}

/**
 * 命令生成线程主函数：循环处理目标数据，生成并发送控制命令
 * 独立于主线程执行，避免阻塞主逻辑（如图像采集、目标检测）
 */
void CommandGener::generate_command()
{
  auto t0 = std::chrono::steady_clock::now();  // 记录线程启动时间（用于调试计时）
  while (!stop_) 
  {  // 线程主循环：未收到停止信号则持续运行
    std::optional<Input> input;  // 存储待处理的输入数据（optional支持空值）
    {
      // 加锁读取最新数据（避免与push函数的写操作冲突）
      std::lock_guard<std::mutex> lock(mtx_);
      // 检查是否有最新数据，且数据未过期（时间差<0.2秒，避免使用过时数据）
      if (latest_ && tools::delta_time(std::chrono::steady_clock::now(), latest_->t) < 0.2) 
      {
        input = latest_;  // 复制最新数据到input
      } else 
      {
        input = std::nullopt;  // 无有效数据，置为空
      }
    }

    // 若有有效输入数据，开始生成控制命令
    if (input) 
    {
      // 1. 调用瞄准器计算云台控制命令（yaw偏航角、pitch俯仰角）
      // 输入：目标列表、时间戳、子弹速度（用于弹道解算和延迟补偿）
      auto command = aimer_.aim(input->targets_, input->t, input->bullet_speed);

      // 2. 调用发射器判断是否需要发射（shoot为true/false）
      // 输入：控制命令、瞄准器、目标列表、云台当前姿态（用于发射条件判断，如偏差是否在允许范围）
      command.shoot = shooter_.shoot(command, aimer_, input->targets_, input->gimbal_pos);

      // 3. 计算目标水平距离（用于调试和发射逻辑辅助）
      // 若目标列表非空，取第一个目标的x/z坐标计算水平距离（sqrt(x² + z²)）
      command.horizon_distance = input->targets_.empty()
                                   ? 0  // 无目标时距离为0
                                   : std::sqrt(
                                       tools::square(input->targets_.front().ekf_x()[0]) +  // x坐标平方
                                       tools::square(input->targets_.front().ekf_x()[2])); // z坐标平方

      // 4. 向下位机发送控制命令（包含yaw、pitch、shoot等信息）
      cboard_.send(command);

      // 5. 若启用调试模式，记录并绘制调试数据
      if (debug_) 
      {
        nlohmann::json data;  // 用JSON格式存储调试数据
        data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);  // 线程运行时间
        data["cmd_yaw"] = command.yaw * 57.3;  // 偏航角命令（弧度转角度，57.3≈180/π）
        data["cmd_pitch"] = command.pitch * 57.3;  // 俯仰角命令（弧度转角度）
        data["shoot"] = command.shoot;  // 发射标志（true=发射，false=不发射）
        data["horizon_distance"] = command.horizon_distance;  // 目标水平距离
        plotter_.plot(data);  // 调用绘图工具绘制数据（如实时曲线）
      }
    }

    // 线程休眠2毫秒，控制循环频率约500Hz（避免CPU占用过高）
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

}  // namespace multithread

}  // namespace auto_aim