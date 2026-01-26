#ifndef AUTO_AIM__PLANNER_HPP
#define AUTO_AIM__PLANNER_HPP

#include <Eigen/Dense>
#include <list>
#include <optional>

#include "tasks/auto_aim/target.hpp"  // 目标相关数据结构头文件（包含目标位置、速度等信息）
#include "tinympc/tiny_api.hpp"       // TinyMPC模型预测控制库接口头文件

namespace auto_aim
{
// 控制周期：10ms（系统的离散控制步长）
constexpr double DT = 0.01;
// 半预测时域长度（用于对称预测或简化计算）
constexpr int HALF_HORIZON = 50;
// 完整预测时域长度（MPC优化的时间步数，共100步=1秒预测）
constexpr int HORIZON = HALF_HORIZON * 2;

/**
 * @brief 轨迹数据类型定义
 * @details Eigen矩阵维度：4行（状态量）× HORIZON列（时间步）
 * 状态量顺序：
 * 第0行：yaw（偏航角，单位：rad）
 * 第1行：yaw_vel（偏航角速度，单位：rad/s）
 * 第2行：pitch（俯仰角，单位：rad）
 * 第3行：pitch_vel（俯仰角速度，单位：rad/s）
 */
using Trajectory = Eigen::Matrix<double, 4, HORIZON>;

/**
 * @brief 规划输出结果结构体
 * @details 包含控制器所需的目标指令和当前状态信息
 */
struct Plan
{
  bool control;          // 控制使能标志（true：执行控制，false：停止控制）
  bool fire;             // 发射使能标志（true：允许发射，false：禁止发射）
  float target_yaw;      // 目标偏航角（期望最终达到的偏航角）
  float target_pitch;    // 目标俯仰角（期望最终达到的俯仰角）
  float yaw;             // 当前时刻偏航角（规划轨迹的起始偏航角）
  float yaw_vel;         // 当前时刻偏航角速度
  float yaw_acc;         // 当前时刻偏航角加速度
  float pitch;           // 当前时刻俯仰角（规划轨迹的起始俯仰角）
  float pitch_vel;       // 当前时刻俯仰角速度
  float pitch_acc;       // 当前时刻俯仰角加速度
};

/**
 * @brief 自动瞄准规划器类
 * @details 基于模型预测控制（MPC）实现目标跟踪的轨迹规划，输出云台控制指令和发射决策
 */
class Planner
{
public:
  Eigen::Vector4d debug_xyza;  // 调试用数据（可能存储目标x/y/z坐标及附加信息a）

  /**
   * @brief 构造函数
   * @param config_path MPC控制器配置文件路径（包含权重、约束等参数）
   */
  Planner(const std::string & config_path);

  /**
   * @brief 规划接口（带确定目标）
   * @param target 目标对象（包含目标位置、速度、尺寸等信息）
   * @param bullet_speed 子弹速度（单位：m/s，用于弹道补偿计算）
   * @return 规划结果（控制指令+状态信息）
   */
  Plan plan(Target target, double bullet_speed);

  /**
   * @brief 规划接口（带可选目标）
   * @param target 可选目标对象（std::optional：可能为空，表示未检测到目标）
   * @param bullet_speed 子弹速度（单位：m/s）
   * @return 规划结果（未检测到目标时返回默认控制指令）
   */
  Plan plan(std::optional<Target> target, double bullet_speed);

private:
  // 机械安装偏移量（用于校准云台零位与实际瞄准方向的偏差）
  double yaw_offset_;    // 偏航角偏移量（单位：rad）
  double pitch_offset_;  // 俯仰角偏移量（单位：rad）

  double fire_thresh_;  // 发射阈值（例如：瞄准误差小于该值时允许发射）

  // 发射延迟参数（根据子弹速度动态调整发射时机）
  double low_speed_delay_time_;  // 低速子弹发射延迟时间（单位：s）
  double high_speed_delay_time_; // 高速子弹发射延迟时间（单位：s）
  double decision_speed_;        // 速度分界值（区分高低速子弹的阈值，单位：m/s）

  TinySolver * yaw_solver_;   // 偏航角MPC求解器（负责偏航通道轨迹优化）
  TinySolver * pitch_solver_; // 俯仰角MPC求解器（负责俯仰通道轨迹优化）

  /**
   * @brief 初始化偏航角MPC求解器
   * @param config_path 配置文件路径（读取偏航通道的MPC参数）
   */
  void setup_yaw_solver(const std::string & config_path);

  /**
   * @brief 初始化俯仰角MPC求解器
   * @param config_path 配置文件路径（读取俯仰通道的MPC参数）
   */
  void setup_pitch_solver(const std::string & config_path);

  /**
   * @brief 计算瞄准角（含弹道补偿）
   * @param target 目标对象
   * @param bullet_speed 子弹速度
   * @return 2维向量（[补偿后的偏航角, 补偿后的俯仰角]，单位：rad）
   */
  Eigen::Matrix<double, 2, 1> aim(const Target & target, double bullet_speed);

  /**
   * @brief 生成最优轨迹（MPC核心计算）
   * @param target 目标对象
   * @param yaw0 偏航角初始状态（当前云台偏航角，单位：rad）
   * @param bullet_speed 子弹速度
   * @return 优化后的轨迹（4维状态×100时间步）
   */
  Trajectory get_trajectory(Target & target, double yaw0, double bullet_speed);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__PLANNER_HPP