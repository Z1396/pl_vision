#ifndef AUTO_AIM__AIMER_HPP
#define AUTO_AIM__AIMER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>

#include "io/cboard.hpp"
#include "io/command.hpp"
#include "target.hpp"

namespace auto_aim
{

// 定义一个名为 AimPoint 的结构体（struct），用于存储瞄准点的核心信息
// 通常在射击类机器人、目标跟踪系统中使用，用于传递“是否可瞄准”及“瞄准点具体坐标”的关键数据
struct AimPoint
{
  // 1. 瞄准点有效性标志：标记当前瞄准点是否合法、可用于射击
  // - true：表示该瞄准点有效（例如在可射击角度内、目标识别稳定）
  // - false：表示该瞄准点无效（例如无符合条件的装甲板、目标信息异常）
  // 作用：上层逻辑可通过该标志判断是否使用当前 xyza 数据，避免无效瞄准
  bool valid;

  // 2. 瞄准点的四维度坐标与角度信息（Eigen库的Vector4d类型，存储4个double值）
  // 各维度含义由业务场景约定，结合前序代码（装甲板瞄准），通常定义为：
  // - xyza[0]：瞄准点在世界坐标系/相机坐标系下的 X 轴坐标
  // - xyza[1]：瞄准点在世界坐标系/相机坐标系下的 Y 轴坐标
  // - xyza[2]：瞄准点在世界坐标系/相机坐标系下的 Z 轴坐标（深度信息）
  // - xyza[3]：瞄准点的角度信息（通常是偏航角 yaw，单位为弧度，用于姿态对准）
  // 作用：精确描述瞄准点的空间位置和姿态，为后续射击控制提供数据基础
  Eigen::Vector4d xyza;
};

class Aimer
{
public:
  AimPoint debug_aim_point;
  explicit Aimer(const std::string & config_path);
  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    bool to_now = true);

  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    io::ShootMode shoot_mode, bool to_now = true);

private:
// 基础偏航角偏移（弧度制）：补偿瞄准系统水平方向固定偏差（如相机安装倾斜、炮管水平偏移），确保瞄准基准与射击基准对齐
double yaw_offset_;

// 左/右侧射击模式的可选偏航角偏移（弧度制，std::optional类型）：
// 仅双炮管等多射击单元场景使用，补偿单侧单元额外水平偏差；未配置时为nullopt（无值），不启用单侧补偿
std::optional<double> left_yaw_offset_, right_yaw_offset_;

// 基础俯仰角偏移（弧度制）：补偿瞄准系统垂直方向固定偏差（如炮管重力下垂、相机垂直安装误差），避免瞄准点上下偏移
double pitch_offset_;

// 目标进入视野角度（弧度制）：定义目标从哪个水平角度进入有效瞄准范围（如机身右侧30°），超出则不启动瞄准逻辑
double comming_angle_;

// 目标离开视野角度（弧度制）：定义目标从哪个水平角度离开有效瞄准范围（如机身左侧30°），超出则停止瞄准跟踪
double leaving_angle_;

// 当前锁定目标的ID（初始值-1表示未锁定任何目标）：用于持续跟踪特定目标，防止多目标干扰导致的频繁切换
double lock_id_ = -1;

// 高速目标射击延迟时间（单位：秒）：目标速度超过阈值时，补偿子弹飞行时间（高速目标位移快，需更长延迟预测未来位置）
double high_speed_delay_time_;

// 低速目标射击延迟时间（单位：秒）：目标速度低于阈值时，补偿子弹飞行时间（低速目标位移慢，需更短延迟）
double low_speed_delay_time_;

// 高速/低速目标区分阈值（单位：米/秒）：判断目标速度等级的标准，决定使用对应的射击延迟参数
double decision_speed_;

  AimPoint choose_aim_point(const Target & target);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIMER_HPP