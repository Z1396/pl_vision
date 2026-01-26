#ifndef AUTO_AIM__SHOOTER_HPP
#define AUTO_AIM__SHOOTER_HPP

#include <string>  // 用于处理字符串（如配置文件路径）

// 包含命令结构体和自瞄瞄准器类的头文件
#include "io/command.hpp"
#include "tasks/auto_aim/aimer.hpp"

namespace auto_aim
{
/**
 * 发射器类（Shooter）
 * 功能：根据目标状态、瞄准命令和云台姿态判断是否满足发射条件，控制发射逻辑
 */
class Shooter
{
public:
  /**
   * 构造函数
   * @param config_path 配置文件路径，用于加载发射相关参数（如距离阈值、角度容忍度等）
   */
  Shooter(const std::string & config_path);

  /**
   * 判断是否发射
   * 核心逻辑：根据瞄准命令、目标状态和云台当前姿态，决定是否触发发射
   * @param command 瞄准器生成的控制命令（包含期望角度等信息）
   * @param aimer 瞄准器对象（提供瞄准相关的状态信息）
   * @param targets 追踪到的目标列表（包含目标位置、速度等信息）
   * @param gimbal_pos 云台当前姿态（欧拉角，用于计算实际角度偏差）
   * @return 是否发射（true表示发射，false表示不发射）
   */
  bool shoot(
    const io::Command & command, const auto_aim::Aimer & aimer,
    const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos);

private:
  io::Command last_command_;       // 上一次的控制命令（用于状态跟踪或滤波）
  double judge_distance_;          // 发射判断的距离阈值（超出此距离不发射）
  double first_tolerance_;         // 一级角度容忍度（高优先级目标的角度偏差允许范围）
  double second_tolerance_;        // 二级角度容忍度（低优先级目标的角度偏差允许范围）
  bool auto_fire_;                 // 自动发射使能标志（是否允许自动发射）
};
}  // namespace auto_aim

#endif  // AUTO_AIM__SHOOTER_HPP