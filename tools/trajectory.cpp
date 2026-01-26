#include "trajectory.hpp"

#include <cmath>

namespace tools
{
constexpr double g = 9.7833;

// Trajectory结构体的构造函数实现
// 功能：根据子弹初速度、目标水平距离和高度，计算弹道参数（俯仰角和飞行时间）
// 参数：
//   v0 - 子弹初速度大小（m/s）
//   d  - 目标水平距离（m）
//   h  - 目标竖直高度（m，相对发射点）
Trajectory::Trajectory(const double v0, const double d, const double h)
{
  // 计算二次方程系数a（基于物理运动学公式推导）
  // g为重力加速度（约9.8m/s²），此处为全局变量或宏定义
  // 推导依据：理想弹道模型中竖直方向位移公式
  auto a = g * d * d / (2 * v0 * v0);
  
  // 计算二次方程系数b
  auto b = -d;
  
  // 计算二次方程系数c
  auto c = a + h;
  
  // 计算二次方程判别式Δ = b² - 4ac
  // 用于判断方程是否有实数解（即弹道是否可行）
  auto delta = b * b - 4 * a * c;

  // 若判别式小于0，方程无实数解，说明无法命中目标
  if (delta < 0) 
  {
    unsolvable = true;  // 标记弹道不可解
    return;
  }

  // 弹道可解，标记状态
  unsolvable = false;
  
  // 计算二次方程的两个解（正切值形式）
  // 对应两种可能的发射角度：高抛弹道和低平弹道
  auto tan_pitch_1 = (-b + std::sqrt(delta)) / (2 * a);
  auto tan_pitch_2 = (-b - std::sqrt(delta)) / (2 * a);
  
  // 将正切值转换为角度（弧度），得到两个可能的俯仰角
  auto pitch_1 = std::atan(tan_pitch_1);
  auto pitch_2 = std::atan(tan_pitch_2);
  
  // 计算两种弹道对应的飞行时间
  // 推导依据：水平方向匀速运动，时间 = 水平距离 / 水平分速度
  // 水平分速度 = v0 * cos(俯仰角)
  auto t_1 = d / (v0 * std::cos(pitch_1));
  auto t_2 = d / (v0 * std::cos(pitch_2));

  // 选择飞行时间较短的弹道参数（通常低平弹道，命中更快，受目标移动影响小）
  pitch = (t_1 < t_2) ? pitch_1 : pitch_2;    // 确定最终俯仰角
  fly_time = (t_1 < t_2) ? t_1 : t_2;        // 确定最终飞行时间
}

}  // namespace tools