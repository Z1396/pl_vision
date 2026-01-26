#include "buff_aimer.hpp"

#include "tools/logger.hpp"       // 日志工具
#include "tools/math_tools.hpp"   // 数学工具函数
#include "tools/trajectory.hpp"   // 弹道计算工具

namespace auto_buff
{
/**
 * @brief Aimer类构造函数，从配置文件加载参数
 * @param config_path 配置文件路径
 */
Aimer::Aimer(const std::string & config_path)
{
  // 加载YAML配置文件
  auto yaml = YAML::LoadFile(config_path);
  // 加载偏航角偏移量(度转弧度)
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;      // degree to rad
  // 加载俯仰角偏移量(度转弧度)
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;  // degree to rad
  // 加载射击间隔时间
  fire_gap_time_ = yaml["fire_gap_time"].as<double>();
  // 加载预测时间
  predict_time_ = yaml["predict_time"].as<double>();

  // 初始化最后射击时间为当前时间
  last_fire_t_ = std::chrono::steady_clock::now();
}

/**
 * @brief 生成瞄准指令
 * @param target 目标对象
 * @param timestamp 目标检测时间戳
 * @param bullet_speed 子弹速度
 * @param to_now 是否基于当前时间预测
 * @return 瞄准和射击指令
 */
io::Command Aimer::aim(
  auto_buff::Target & target, std::chrono::steady_clock::time_point & timestamp,
  double bullet_speed, bool to_now)
{
  // 初始化指令：默认不射击、不控制、角度为0
  io::Command command = {false, false, 0, 0};
  // 如果目标无法解算，直接返回空指令
  if (target.is_unsolve()) return command;

  // 如果子弹速度小于10，将其设为默认值24
  if (bullet_speed < 10) bullet_speed = 24;

  // 获取当前时间
  auto now = std::chrono::steady_clock::now();

  // 计算检测时间到当前时间的间隔
  auto detect_now_gap = tools::delta_time(now, timestamp);
  // 计算预测时间：如果to_now为true，则基于当前时间差+预设预测时间；否则使用固定值
  auto future = to_now ? (detect_now_gap + predict_time_) : 0.1 + predict_time_;
  double yaw, pitch;  // 偏航角和俯仰角

  // 检查角度变化是否超过5度(已转为弧度)
  bool angle_changed =
    std::abs(last_yaw_ - yaw) > 5 / 57.3 || std::abs(last_pitch_ - pitch) > 5 / 57.3;
  
  // 计算发送角度，如果成功
  if (get_send_angle(target, future, bullet_speed, to_now, yaw, pitch)) {
    command.yaw = yaw;                  // 设置偏航角指令
    command.pitch = -pitch;             // 设置俯仰角指令(世界坐标系下向上为负)
    
    // 错误计数超过3次
    if (mistake_count_ > 3) {
      switch_fanblade_ = true;          // 标记需要切换扇叶
      mistake_count_ = 0;               // 重置错误计数
      command.control = true;           // 允许控制
    } 
    // 角度变化超过阈值
    else if (std::abs(last_yaw_ - yaw) > 5 / 57.3 || std::abs(last_pitch_ - pitch) > 5 / 57.3) {
      switch_fanblade_ = true;          // 标记需要切换扇叶
      mistake_count_++;                 // 增加错误计数
      command.control = false;          // 禁止控制
    } 
    // 角度稳定
    else {
      switch_fanblade_ = false;         // 不需要切换扇叶
      mistake_count_ = 0;               // 重置错误计数
      command.control = true;           // 允许控制
    }
    // 更新最后一次的角度值
    last_yaw_ = yaw;
    last_pitch_ = pitch;
  }

  // 如果需要切换扇叶，不射击并更新最后射击时间
  if (switch_fanblade_) 
  {
    command.shoot = false;
    last_fire_t_ = now;
  } 
  // 如果不需要切换扇叶且超过射击间隔时间，允许射击并更新最后射击时间
  else if (!switch_fanblade_ && tools::delta_time(now, last_fire_t_) > fire_gap_time_) 
  {
    command.shoot = true;
    last_fire_t_ = now;
  }

  return command;
}

/**
 * @brief 基于模型预测控制(MPC)的瞄准方法，生成更精细的控制计划
 * @param target 目标对象
 * @param timestamp 目标检测时间戳
 * @param gs 云台状态
 * @param to_now 是否基于当前时间预测
 * @return 包含速度、加速度的控制计划
 */
auto_aim::Plan Aimer::mpc_aim(
  auto_buff::Target & target, std::chrono::steady_clock::time_point & timestamp, io::GimbalState gs,
  bool to_now)
{
  // 初始化计划：默认不射击、不控制、各项参数为0
  auto_aim::Plan plan = {false, false, 0, 0, 0, 0, 0, 0, 0, 0};
  // 如果目标无法解算，直接返回空计划
  if (target.is_unsolve()) return plan;

  double bullet_speed;
  // 如果子弹速度小于10，将其设为默认值24
  if (gs.bullet_speed < 10)
    bullet_speed = 24;
  else
    bullet_speed = gs.bullet_speed;

  // 获取当前时间
  auto now = std::chrono::steady_clock::now();

  // 计算检测时间到当前时间的间隔
  auto detect_now_gap = tools::delta_time(now, timestamp);
  // 计算预测时间
  auto future = to_now ? (detect_now_gap + predict_time_) : 0.1 + predict_time_;
  double yaw, pitch;  // 偏航角和俯仰角

  // 检查角度变化是否超过5度
  bool angle_changed =
    std::abs(last_yaw_ - yaw) > 5 / 57.3 || std::abs(last_pitch_ - pitch) > 5 / 57.3;
  
  // 计算发送角度，如果成功
  if (get_send_angle(target, future, bullet_speed, to_now, yaw, pitch)) {
    plan.yaw = yaw;                 // 设置偏航角
    plan.pitch = -pitch;            // 设置俯仰角(世界坐标系下向上为负)
    
    // 错误计数超过3次
    if (mistake_count_ > 3) {
      switch_fanblade_ = true;      // 标记需要切换扇叶
      mistake_count_ = 0;           // 重置错误计数
      plan.control = true;          // 允许控制
      first_in_aimer_ = true;       // 标记首次进入瞄准
    } 
    // 角度变化超过阈值
    else if (std::abs(last_yaw_ - yaw) > 5 / 57.3 || std::abs(last_pitch_ - pitch) > 5 / 57.3) {
      switch_fanblade_ = true;      // 标记需要切换扇叶
      mistake_count_++;             // 增加错误计数
      plan.control = false;         // 禁止控制
      first_in_aimer_ = true;       // 标记首次进入瞄准
    } 
    // 角度稳定
    else {
      switch_fanblade_ = false;     // 不需要切换扇叶
      mistake_count_ = 0;           // 重置错误计数
      plan.control = true;          // 允许控制
    }
    // 更新最后一次的角度值
    last_yaw_ = yaw;
    last_pitch_ = pitch;

    // 如果允许控制
    if (plan.control) {
      // 如果是首次进入瞄准
      if (first_in_aimer_) {
        // 初始速度和加速度设为0
        plan.yaw_vel = 0;
        plan.yaw_acc = 0;
        plan.pitch_vel = 0;
        plan.pitch_acc = 0;
        first_in_aimer_ = false;    // 清除首次标记
      } else {
        // 计算时间间隔
        auto dt = predict_time_;
        double last_yaw_mpc, last_pitch_mpc;
        
        // 获取上一时刻的角度
        get_send_angle(
          target, predict_time_ * -1, bullet_speed, to_now, last_yaw_mpc, last_pitch_mpc);
        
        // 计算偏航角速度(限制在弧度范围内)
        plan.yaw_vel = tools::limit_rad(yaw - last_yaw_mpc) / (2 * dt);
        // 计算偏航角加速度
        plan.yaw_acc = (tools::limit_rad(yaw - gs.yaw) - tools::limit_rad(gs.yaw - last_yaw_mpc)) /
                       std::pow(dt, 2);

        // 计算俯仰角速度
        plan.pitch_vel = tools::limit_rad(-pitch + last_pitch_mpc) / (2 * dt);
        // 计算俯仰角加速度
        plan.pitch_acc = (-pitch - gs.pitch - (gs.pitch + last_pitch_mpc)) / std::pow(dt, 2);
      }
    }
  }

  // 如果需要切换扇叶，不射击并更新最后射击时间
  if (switch_fanblade_) {
    plan.fire = false;
    last_fire_t_ = now;
  } 
  // 如果不需要切换扇叶且超过射击间隔时间，允许射击并更新最后射击时间
  else if (!switch_fanblade_ && tools::delta_time(now, last_fire_t_) > fire_gap_time_) {
    plan.fire = true;
    last_fire_t_ = now;
  }

  return plan;
}

/**
 * @brief 计算发送给执行器的角度
 * @param target 目标对象
 * @param predict_time 预测时间
 * @param bullet_speed 子弹速度
 * @param to_now 是否基于当前时间
 * @param yaw 输出的偏航角
 * @param pitch 输出的俯仰角
 * @return 是否成功计算角度
 */
bool Aimer::get_send_angle(
  auto_buff::Target & target, const double predict_time, const double bullet_speed,
  const bool to_now, double & yaw, double & pitch)
{
  // 预测目标在未来predict_time时刻的位置
  target.predict(predict_time);
  // 获取目标的角度状态(可能是扇叶角度)
  angle = target.ekf_x()[5];

  // 计算目标点在世界坐标系下的空间坐标
  // 这里的Eigen::Vector3d(0.0, 0.0, 0.7)可能是目标上的特定点(如中心)
  auto aim_in_world = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.7));
  // 计算水平距离
  double d = std::sqrt(aim_in_world[0] * aim_in_world[0] + aim_in_world[1] * aim_in_world[1]);
  // 计算高度差
  double h = aim_in_world[2];

  // 创建弹道对象，计算子弹轨迹
  tools::Trajectory trajectory0(bullet_speed, d, h);
  // 如果弹道无法解算，返回失败
  if (trajectory0.unsolvable) {  
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d, h);
    return false;
  }

  // 根据第一个弹道的飞行时间再次预测目标位置
  target.predict(trajectory0.fly_time);
  angle = target.ekf_x()[5];

  // 重新计算目标点在世界坐标系下的空间坐标
  aim_in_world = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.7));
  d = std::sqrt(aim_in_world[0] * aim_in_world[0] + aim_in_world[1] * aim_in_world[1]);
  h = aim_in_world[2];
  
  // 再次计算弹道
  tools::Trajectory trajectory1(bullet_speed, d, h);
  // 如果弹道无法解算，返回失败
  if (trajectory1.unsolvable) {  
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory1: {:.2f} {:.2f} {:.2f}", bullet_speed, d, h);
    return false;
  }

  // 计算两次弹道飞行时间的误差
  auto time_error = trajectory1.fly_time - trajectory0.fly_time;
  // 如果时间误差过大(超过0.01秒)，返回失败
  if (std::abs(time_error) > 0.01) {  
    tools::logger()->debug("[Aimer] Large time error: {:.3f}", time_error);
    return false;
  }

  // 计算最终的偏航角(加上偏移量)
  yaw = std::atan2(aim_in_world[1], aim_in_world[0]) + yaw_offset_;
  // 计算最终的俯仰角(加上偏移量)
  pitch = trajectory1.pitch + pitch_offset_;
  return true;
};

}  // namespace auto_buff