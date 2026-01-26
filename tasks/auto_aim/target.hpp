#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{
/**
 * @brief 目标装甲板类，用于表示和跟踪识别到的装甲板目标
 * 
 * 该类包含目标装甲板的属性信息和状态估计功能，
 * 使用扩展卡尔曼滤波器(EKF)进行状态预测和更新，
 * 实现对装甲板目标的持续跟踪。
 */
class Target
{
public:
  // 装甲板名称（可能表示不同类型的装甲板，如英雄、步兵等）
  ArmorName name;
  // 装甲板类型（可能表示装甲板的形状或规格）
  ArmorType armor_type;
  // 装甲板优先级（用于多目标时的决策）
  ArmorPriority priority;
  // 标记是否是否跳跃变（可能表示目标状态是否发生突变）
  bool jumped;
  // 上一帧的ID（仅用于调试）
  int last_id;  

  // 默认构造函数
  Target() = default;

  /**
   * @brief 构造函数，从装甲板检测结果初始化目标
   * @param armor 装甲板检测数据
   * @param t 时间戳
   * @param radius 半径参数（可能是装甲板相关尺寸）
   * @param armor_num 装甲板数量
   * @param P0_dig 初始协方差矩阵的对角线元素
   */
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig);

  /**
   * @brief 构造函数，从运动学参数初始化目标
   * @param x x坐标
   * @param vyaw 偏航角速度
   * @param radius 半径参数
   * @param h 高度参数
   */
  Target(double x, double vyaw, double radius, double h);

  /**
   * @brief 根据时间戳预测目标状态
   * @param t 预测的目标时间戳
   */
  void predict(std::chrono::steady_clock::time_point t);

  /**
   * @brief 根据时间间隔预测目标状态
   * @param dt 时间间隔（秒）
   */
  void predict(double dt);

  /**
   * @brief 使用新检测到的装甲板数据更新目标状态
   * @param armor 新检测到的装甲板数据
   */
  void update(const Armor & armor);

  /**
   * @brief 获取EKF的状态向量
   * @return 状态向量（包含位置、速度等信息）
   */
  Eigen::VectorXd ekf_x() const;

  /**
   * @brief 获取扩展卡尔曼滤波器实例
   * @return 常量引用的EKF对象
   */
  const tools::ExtendedKalmanFilter & ekf() const;

  /**
   * @brief 获取装甲板的坐标和角度列表
   * @return 包含多个装甲板的x、y、z坐标和角度的向量列表
   */
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  /**
   * @brief 检查目标状态是否发散
   * @return 若状态发散返回true，否则返回false
   */
  bool diverged() const;

  /**
   * @brief 检查目标状态是否收敛
   * @return 若状态收敛返回true，否则返回false
   */
  bool convergened();

  // 目标是否已初始化
  bool isinit = false;

  /**
   * @brief 检查目标是否已初始化
   * @return 若已初始化返回true，否则返回false
   */
  bool checkinit();

  std::string outpost_state() const;

private:

  mutable std::string outpost_state_;
  // 装甲板数量
  int armor_num_;
  // 切换计数（可能用于统计目标切换次数）
  int switch_count_;
  // 更新计数（统计状态更新次数）
  int update_count_;

  // 是否发生切换，是否已收敛
  bool is_switch_, is_converged_;

  // 扩展卡尔曼滤波器实例，用于状态估计
  tools::ExtendedKalmanFilter ekf_;
  // 上一次更新的时间戳
  std::chrono::steady_clock::time_point t_;

  /**
   * @brief 使用YPDA（Yaw-Pitch-Distance-Angle）方法更新目标
   * @param armor 装甲板检测数据
   * @param id 装甲板ID
   */
  void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle

  /**
   * @brief 计算装甲板在世界坐标系中的坐标
   * @param x 状态向量
   * @param id 装甲板ID
   * @return 装甲板的x、y、z坐标
   */
  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;

  /**
   * @brief 计算观测模型的雅可比矩阵
   * @param x 状态向量
   * @param id 装甲板ID
   * @return 雅可比矩阵
   */
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP