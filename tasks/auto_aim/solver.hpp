#ifndef AUTO_AIM__SOLVER_HPP
#define AUTO_AIM__SOLVER_HPP

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

#include "armor.hpp"

namespace auto_aim
{
class Solver
{
public:
  explicit Solver(const std::string & config_path);  /*在 C++ 中，explicit 是一个关键字，主要用于修饰类的构造函数，
                                                      其核心作用是禁止隐式类型转换，只能进行显式类型转换*/

  Eigen::Matrix3d R_gimbal2world() const;
  Eigen::Matrix3d T_gimbal2world() const;

  void set_R_gimbal2world(const Eigen::Quaterniond & q);

  void solve(Armor & armor) const;

  std::vector<cv::Point2f> reproject_armor(
    const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const;

  double oupost_reprojection_error(Armor armor, const double & picth);

  std::vector<cv::Point2f> world2pixel(const std::vector<cv::Point3f> & worldPoints);

private:
// 相机内参矩阵，用于描述相机的光学特性（焦距、主点坐标等）
// 类型：3x3 cv::Mat矩阵，结构为 [fx, 0, cx; 0, fy, cy; 0, 0, 1]
// 作用：将三维相机坐标系下的点投影到二维图像平面
cv::Mat camera_matrix_;

// 相机畸变系数，用于校正镜头光学畸变
// 类型：1x5或1x8 cv::Mat向量，通常为[k1, k2, p1, p2, k3]
// 作用：描述径向畸变和切向畸变，配合内参矩阵使用可消除图像畸变
cv::Mat distort_coeffs_;

// 云台坐标系到IMU机体坐标系的旋转矩阵
// 类型：3x3 Eigen双精度矩阵
// 作用：表示云台与IMU之间的相对旋转关系，用于将云台坐标系下的点转换到IMU机体坐标系
Eigen::Matrix3d R_gimbal2imubody_;

Eigen::Vector3d T_gimbal2imubody_;


// 相机坐标系到云台坐标系的旋转矩阵
// 类型：3x3 Eigen双精度矩阵
// 作用：描述相机相对于云台的安装姿态，用于坐标从相机坐标系到云台坐标系的旋转转换
Eigen::Matrix3d R_camera2gimbal_;

// 相机坐标系到云台坐标系的平移向量
// 类型：3维Eigen双精度向量
// 作用：表示相机坐标系原点在云台坐标系下的位置，配合旋转旋转矩阵完成坐标转换
Eigen::Vector3d t_camera2gimbal_;

// 云台坐标系到世界坐标系的旋转矩阵
// 类型：3x3 Eigen双精度矩阵
// 作用：描述云台在世界坐标系中的朝向，用于将云台坐标系下的点转换到世界坐标系
Eigen::Matrix3d R_gimbal2world_;

Eigen::Vector3d T_gimbal2world_;                                // 云台→世界平移（新增，用于输出）

  void optimize_yaw(Armor & armor) const;

  double armor_reprojection_error(const Armor & armor, double yaw, const double & inclined) const;
  double SJTU_cost(
    const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
    const double & inclined) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__SOLVER_HPP