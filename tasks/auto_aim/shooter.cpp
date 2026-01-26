#include "shooter.hpp"

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim  // 自动瞄准相关功能的命名空间
{
// 构造函数：从配置文件初始化Shooter类的参数
// 参数：config_path - 配置文件的路径
Shooter::Shooter(const std::string & config_path) : last_command_{false, false, 0, 0}
{
  // 加载YAML配置文件
  auto yaml = YAML::LoadFile(config_path);
  
  // 从配置文件读取第一级 tolerance（允许的误差范围），并从度转换为弧度
  first_tolerance_ = yaml["first_tolerance"].as<double>() / 57.3;    // degree to rad
  
  // 从配置文件读取第二级 tolerance，同样从度转换为弧度
  second_tolerance_ = yaml["second_tolerance"].as<double>() / 57.3;  // degree to rad
  
  // 从配置文件读取判断距离（用于切换不同tolerance的阈值）
  judge_distance_ = yaml["judge_distance"].as<double>();
  
  // 从配置文件读取自动开火开关状态
  auto_fire_ = yaml["auto_fire"].as<bool>();
}

/**
 * @brief 射击判断函数，决定是否执行射击操作
 * 
 * @param command 控制命令（包含是否允许控制等信息）
 * @param aimer 瞄准器对象（包含瞄准点等信息）
 * @param targets 目标列表（检测到的目标）
 * @param gimbal_pos 云台当前位置（可能包含yaw/pitch等角度信息）
 * @return true 如果满足射击条件，返回true（执行射击）
 * @return false 不满足射击条件，返回false（不射击）
 */
bool Shooter::shoot(
  const io::Command & command, const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos)
{
  // 射击条件前置判断：保留原逻辑
  if (!command.control || targets.empty() || !auto_fire_) return false;

  auto target_x = targets.front().ekf_x()[0];
  auto target_y = targets.front().ekf_x()[2];
  
  // 1. 放宽 tolerance 基础值：在原阈值基础上放大（例如1.5倍，可根据需求调整）
  auto tolerance = std::sqrt(tools::square(target_x) + tools::square(target_y)) > judge_distance_
                     ? second_tolerance_ * 1  // 远距离阈值放大
                     : first_tolerance_ * 1;   // 近距离阈值放大（比远距离略小，保持精度）
  
  // 2. 放宽命令突变判断：原倍数是2倍，改为3倍（允许更大的命令波动）
  // 3. 放宽瞄准精度判断：原是1倍tolerance，改为1.5倍（允许更大的瞄准偏差）
  if (
    std::abs(last_command_.yaw - command.yaw) < tolerance * 2.0 &&  // 原2倍 → 3倍
    std::abs(gimbal_pos[0] - last_command_.yaw) < tolerance * 1.0 &&  // 原1倍 → 1.5倍
    aimer.debug_aim_point.valid) {
    
    last_command_ = command;
    return true;
  }

  last_command_ = command;
  return false;
}

}  // namespace auto_aim