#ifndef TOOLS__TRAJECTORY_HPP
#define TOOLS__TRAJECTORY_HPP

namespace tools
{
// 定义一个名为 Trajectory 的结构体，用于存储弹道计算结果
// 主要应用于需要计算子弹/抛射物飞行轨迹的场景（如射击类机器人、弹道模拟等）
struct Trajectory
{
  // 1. 弹道是否可解标志
  // - true：表示根据给定参数（初速度、距离、高度）无法计算出有效弹道（例如超出射程）
  // - false：表示弹道计算成功，得到了有效的飞行时间和俯仰角
  bool unsolvable;

  // 2. 子弹飞行时间
  // 单位：秒（s）
  // 含义：子弹从发射到命中目标所需的时间，用于预测目标未来位置（考虑目标移动）
  double fly_time;

  // 3. 发射俯仰角
  // 单位：通常为弧度（rad）
  // 含义：发射装置与水平面的夹角，正值表示抬头（向上倾斜），负值表示低头（向下倾斜）
  // 用于控制发射机构的角度，确保子弹能命中目标
  double pitch;  // 抬头为正

  // 4. 构造函数声明：用于初始化弹道参数并计算弹道
  // 参数说明：
  // - v0：子弹初速度大小，单位：米/秒（m/s）
  // - d：目标水平距离，单位：米（m）（发射点与目标在水平面上的直线距离）
  // - h：目标竖直高度，单位：米（m）（目标相对发射点的高度差，可为正可为负）
  // 注：该构造函数不考虑空气阻力对弹道的影响，基于理想抛物线运动模型计算
  Trajectory(const double v0, const double d, const double h);
};

}  // namespace tools

#endif  // TOOLS__TRAJECTORY_HPP