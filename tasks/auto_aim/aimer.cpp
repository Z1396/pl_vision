#include "aimer.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_aim
{
Aimer::Aimer(const std::string & config_path)
: left_yaw_offset_(std::nullopt), right_yaw_offset_(std::nullopt)
{
  auto yaml = YAML::LoadFile(config_path);
  // 偏航角偏移量（角度转弧度：除以180/π≈57.3）
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;    

  // 俯仰角偏移量（同上）
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;    

  // 入射角度（目标进入视野的角度）
  comming_angle_ = yaml["comming_angle"].as<double>() / 57.3;  

  // 出射角度（目标离开视野的角度）
  leaving_angle_ = yaml["leaving_angle"].as<double>() / 57.3;  

  // 高速目标延迟时间（补偿高速运动的时间差）
  high_speed_delay_time_ = yaml["high_speed_delay_time"].as<double>();

  // 低速目标延迟时间
  low_speed_delay_time_ = yaml["low_speed_delay_time"].as<double>();

  // 速度阈值（用于区分高速/低速目标）
  decision_speed_ = yaml["decision_speed"].as<double>();

  // 检查YAML中是否定义了left_yaw_offset和right_yaw_offset
  if (yaml["left_yaw_offset"].IsDefined() && yaml["right_yaw_offset"].IsDefined()) 
  {
      // 若定义，则加载并转换为弧度，赋值给optional变量（从无值→有值）
      left_yaw_offset_ = yaml["left_yaw_offset"].as<double>() / 57.3;    
      right_yaw_offset_ = yaml["right_yaw_offset"].as<double>() / 57.3;  
      // 日志输出：提示加载成功
      tools::logger()->info("[Aimer] successfully loading shootmode");
  }
}

/**
 * @brief 瞄准函数，根据目标列表计算瞄准角度，考虑子弹飞行时间和目标运动预测
 * 
 * @param targets 目标列表（通常是跟踪到的敌方装甲板）
 * @param timestamp 时间戳（与目标检测相关的时间戳）
 * @param bullet_speed 子弹速度
 * @param to_now 是否基于当前时间进行预测
 * @return io::Command 包含瞄准指令的结构体（是否有效、角度等信息）
 */
io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  bool to_now)
{
  // 如果没有目标，返回空指令
  if (targets.empty()) return {false, false, 0, 0};
  
  // 取第一个目标作为瞄准对象（默认优先级最高的目标在列表前端）
  auto target = targets.front();

  // 获取目标的卡尔曼滤波器实例，用于状态预测
  auto ekf = target.ekf();
  
  // 根据目标速度（通过EKF状态向量第7个元素判断）选择延迟时间
  // 高速目标使用高速延迟时间，否则使用低速延迟时间
  double delay_time =
    target.ekf_x()[7] > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;

  // 子弹速度校验：如果速度低于14，则使用默认值23（可能是最小有效速度）
  if (bullet_speed < 14) bullet_speed = 23;

  // 计算未来时间点（考虑系统延迟和子弹飞行时间）
  auto future = timestamp;
  if (to_now) 
  {
    // 若需要基于当前时间预测：计算从timestamp到现在的时间差，加上延迟时间
    double dt;
    dt = tools::delta_time(std::chrono::steady_clock::now(), timestamp) + delay_time;
    // 将未来时间点偏移dt微秒
    future += std::chrono::microseconds(int(dt * 1e6));
    // 预测目标在未来时间点的状态
    target.predict(future);
  }
  else 
  {
    // 否则使用固定时间差：0.005秒（detector到aimer的处理延迟）+ 发射延迟
    auto dt = 0.005 + delay_time;  
    future += std::chrono::microseconds(int(dt * 1e6));
    target.predict(future);
  }

  // 选择目标上的瞄准点（可能是装甲板的某个特定位置）
  auto aim_point0 = choose_aim_point(target);
  debug_aim_point = aim_point0;  // 保存调试用的瞄准点
  
  // 如果瞄准点无效，返回空指令
  if (!aim_point0.valid) 
  {
    return {false, false, 0, 0};
  }

  // 提取瞄准点的三维坐标 (x, y, z)
  Eigen::Vector3d xyz0 = aim_point0.xyza.head(3);
  // 计算目标距离（xy平面内的距离）
  auto d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
  
  // 计算初始弹道（基于当前瞄准点和子弹速度）
  tools::Trajectory trajectory0(bullet_speed, d0, xyz0[2]);
  if (trajectory0.unsolvable) 
  {
    // 若弹道不可解（如距离过远），记录日志并返回空指令
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d0, xyz0[2]);
    debug_aim_point.valid = false;
    return {false, false, 0, 0};
  }

  // 迭代求解飞行时间（考虑目标移动，最多10次迭代）
  bool converged = false;
  double prev_fly_time = trajectory0.fly_time;  // 初始飞行时间
  tools::Trajectory current_traj = trajectory0;
  // 创建10个目标副本用于迭代预测（每次迭代使用独立副本避免相互影响）
  std::vector<Target> iteration_target(10, target);

  for (int iter = 0; iter < 10; ++iter) 
  {
    // 计算目标在未来时间 + 上一次飞行时间后的位置（子弹飞到目标所需时间）
    auto predict_time = future + std::chrono::microseconds(static_cast<int>(prev_fly_time * 1e6));
    iteration_target[iter].predict(predict_time);

    // 计算该时间点的瞄准点
    auto aim_point = choose_aim_point(iteration_target[iter]);
    debug_aim_point = aim_point;
    if (!aim_point.valid) 
    {
      return {false, false, 0, 0};
    }

    // 计算新的弹道参数
    Eigen::Vector3d xyz = aim_point.xyza.head(3);
    double d = std::sqrt(xyz.x() * xyz.x() + xyz.y() * xyz.y());  // 新距离
    current_traj = tools::Trajectory(bullet_speed, d, xyz.z());

    // 检查新弹道是否可解
    if (current_traj.unsolvable) 
    {
      tools::logger()->debug(
        "[Aimer] Unsolvable trajectory in iter {}: speed={:.2f}, d={:.2f}, z={:.2f}", iter + 1,
        bullet_speed, d, xyz.z());
      debug_aim_point.valid = false;
      return {false, false, 0, 0};
    }

    // 检查收敛条件：相邻两次飞行时间差小于0.001秒
    if (std::abs(current_traj.fly_time - prev_fly_time) < 0.001) 
    {
      converged = true;
      break;
    }
    prev_fly_time = current_traj.fly_time;
  }

  // 计算最终瞄准角度
  Eigen::Vector3d final_xyz = debug_aim_point.xyza.head(3);
  // 计算偏航角（yaw），加上偏移量校准
  double yaw = std::atan2(final_xyz.y(), final_xyz.x()) + yaw_offset_;
  // 计算俯仰角（pitch），加上偏移量校准（注意符号：世界坐标系中向上为负）
  double pitch = -(current_traj.pitch + pitch_offset_);
  
  // 返回有效的瞄准指令
  return {true, false, yaw, pitch};
}

io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  io::ShootMode shoot_mode, bool to_now)
{
  double yaw_offset;
  if (shoot_mode == io::left_shoot && left_yaw_offset_.has_value()) {
    yaw_offset = left_yaw_offset_.value();
  } else if (shoot_mode == io::right_shoot && right_yaw_offset_.has_value()) {
    yaw_offset = right_yaw_offset_.value();
  } else {
    yaw_offset = yaw_offset_;
  }

  auto command = aim(targets, timestamp, bullet_speed, to_now);
  command.yaw = command.yaw - yaw_offset_ + yaw_offset;

  return command;
}

// 选择瞄准点的函数，返回一个AimPoint结构体（包含是否有效和瞄准点坐标）
// 参数：target - 目标对象，包含目标的状态信息和装甲板列表
AimPoint Aimer::choose_aim_point(const Target & target)
{
  // 获取目标的EKF（扩展卡尔曼滤波）估计状态向量（包含位置、速度等信息）
  Eigen::VectorXd ekf_x = target.ekf_x();
  // 获取目标所有装甲板的坐标信息列表（x,y,z,a角度）
  std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
  // 获取装甲板的数量
  auto armor_num = armor_xyza_list.size();
  
  // 如果目标装甲板未发生跳变（识别稳定），直接返回第一个装甲板作为瞄准点
  // （通常此时只有一个有效装甲板被识别）
  if (!target.jumped) return {true, armor_xyza_list[0]};

  // 计算目标整车旋转中心相对原点的偏航角（yaw）
  // ekf_x[0]是x坐标，ekf_x[2]是y坐标，atan2(y,x)得到方位角（弧度）
  auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  // 计算每个装甲板相对整车旋转中心的角度差（已归一化到[-π, π]）
  // delta_angle = 装甲板自身角度 - 整车旋转中心角度
  std::vector<double> delta_angle_list;
  for (int i = 0; i < armor_num; i++) 
  {
    auto delta_angle = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);
    delta_angle_list.emplace_back(delta_angle);
  }

  // 非小陀螺模式判断：旋转角速度较小（ekf_x[8]为旋转角速度）且不是基地目标
  // （小陀螺指目标快速旋转的战术动作）
  if (std::abs(target.ekf_x()[8]) <= 2 && target.name != ArmorName::outpost) 
  {
    // 筛选出在可射击角度范围内的装甲板（角度小于60度，60/57.3是弧度转换）
    std::vector<int> id_list;
    for (int i = 0; i < armor_num; i++) 
    {
      if (std::abs(delta_angle_list[i]) > 60 / 57.3) continue;  // 超过范围则跳过
      id_list.push_back(i);  // 记录符合条件的装甲板索引
    }
    
    // 如果没有符合条件的装甲板，返回警告并默认第一个装甲板
    if (id_list.empty()) 
    {
      tools::logger()->warn("Empty id list!");
      return {false, armor_xyza_list[0]};
    }

    // 当有多个装甲板在可射击范围内时（通常是两个对称装甲板）
    if (id_list.size() > 1) 
    {
      int id0 = id_list[0], id1 = id_list[1];

      // 未处于锁定模式时，选择角度差绝对值较小的装甲板（更接近瞄准方向）
      // 并进入锁定模式，防止在两个装甲板间频繁切换
      if (lock_id_ != id0 && lock_id_ != id1)
        lock_id_ = (std::abs(delta_angle_list[id0]) < std::abs(delta_angle_list[id1])) ? id0 : id1;

      // 返回锁定的装甲板作为瞄准点
      return {true, armor_xyza_list[lock_id_]};
    }

    // 只有一个装甲板在可射击范围内时，退出锁定模式并返回该装甲板
    lock_id_ = -1;
    return {true, armor_xyza_list[id_list[0]]};
  }

  // 小陀螺模式处理：设置装甲板"进入"和"离开"的角度阈值
  double coming_angle, leaving_angle;
  if (target.name == ArmorName::outpost)  // 如果是基地目标，使用特定角度阈值
  {
    coming_angle = 70 / 57.3;   // 70度（转换为弧度）
    leaving_angle = 30 / 57.3;  // 30度（转换为弧度）
  } else  // 其他目标使用成员变量中存储的角度阈值
  {
    coming_angle = comming_angle_;  // 注意：原代码可能是拼写错误（应为coming）
    leaving_angle = leaving_angle_;
  }

  // 小陀螺模式下的瞄准策略：
  // 优先选择正处于"进入"视野且未"离开"的装甲板（被击中概率更高）
  for (int i = 0; i < armor_num; i++) 
  {
    // 跳过角度超过"进入"阈值的装甲板
    if (std::abs(delta_angle_list[i]) > coming_angle) continue;
    
    // 根据目标旋转方向（ekf_x[7]为旋转方向相关速度）选择合适的装甲板：
    // 1. 正向旋转时，选择角度差小于"离开"阈值的装甲板
    // 2. 反向旋转时，选择角度差大于-"离开"阈值的装甲板
    if (ekf_x[7] > 0 && delta_angle_list[i] < leaving_angle) return {true, armor_xyza_list[i]};
    if (ekf_x[7] < 0 && delta_angle_list[i] > -leaving_angle) return {true, armor_xyza_list[i]};
  }

  // 如果没有符合条件的装甲板，返回默认值
  return {false, armor_xyza_list[0]};
}

}  // namespace auto_aim