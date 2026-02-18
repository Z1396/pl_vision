#ifndef IO__COMMAND_HPP
#define IO__COMMAND_HPP

namespace io
{
struct Command
{
  bool control;
  bool shoot;
  double yaw;
  double pitch;
  double horizon_distance = 0;  //无人机专有
};

struct FD_Command
{
  bool control;
  bool shoot;
  float yaw;                       // 云台目标横滚角：视觉算法规划的目标角度
  float yaw_vel;                   // 云台目标横滚角速度：规划的角速度指令
  float yaw_acc;                   // 云台目标横滚角加速度：规划的角加速度指令，提升云台运动平滑性
  float pitch;                     // 云台目标俯仰角
  float pitch_vel;                 // 云台目标俯仰角速度
  float pitch_acc;                 // 云台目标俯仰角加速度
};


}  // namespace io

#endif  // IO__COMMAND_HPP