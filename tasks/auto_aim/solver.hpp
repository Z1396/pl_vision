/**
 * @file solver.hpp
 * @brief 自动瞄准系统核心坐标解算类头文件
 * @details 实现装甲板目标的**三维坐标解算、云台角度优化、坐标系统转换、目标重投影**等核心功能，
 *          是连接视觉检测与云台控制的关键模块——将检测到的二维装甲板特征转换为三维世界坐标，
 *          并优化云台瞄准角度，为云台运动控制提供精准的目标位置参数。
 * @note 核心依赖：Eigen3（矩阵/向量计算、坐标转换）、OpenCV（相机投影、畸变校正）、自定义Armor数据结构
 * @note 核心功能：多坐标系转换、PnP解算三维位置、云台偏航角优化、目标重投影误差计算、世界坐标转像素坐标
 */
#ifndef AUTO_AIM__SOLVER_HPP  // 头文件保护宏，避免重复包含（命名规范：模块名__类名_HPP）
#define AUTO_AIM__SOLVER_HPP

// 必须优先包含Eigen头文件（避免与OpenCV的Eigen接口冲突）
#include <Eigen/Dense>        // Eigen核心：矩阵、向量、线性代数计算
#include <Eigen/Geometry>     // Eigen几何模块：四元数、旋转矩阵、变换

// OpenCV-Eigen互转接口：实现cv::Mat与Eigen::Matrix的无缝转换
#include <opencv2/core/eigen.hpp>

// 项目自定义头文件：依赖装甲板核心数据结构
#include "armor.hpp"

// 自动瞄准模块命名空间，隔离模块内代码，避免命名冲突
namespace auto_aim
{

/**
 * @class Solver
 * @brief 自动瞄准系统坐标解算与云台角度优化核心类
 * @details 封装多坐标系转换、装甲板三维位置解算、云台瞄准角度优化、目标重投影等核心逻辑，
 *          基于相机内参/畸变系数、硬件安装外参（相机-云台、云台-IMU），将视觉检测的二维装甲板特征，
 *          解算为世界坐标系下的三维坐标，并优化云台偏航角以提升瞄准精度，为云台控制提供直接可用的参数。
 * @note 核心设计：基于**多坐标系链式转换**实现坐标解算，所有外参通过配置文件加载，适配不同硬件安装方式
 * @note 显式构造函数：禁止隐式类型转换，确保配置文件路径必传，避免未初始化使用
 * @note 坐标系统：相机坐标系→云台坐标系→IMU机体坐标系→世界坐标系，全程通过Eigen实现高精度矩阵计算
 */
class Solver
{
public:
  /**
   * @brief 显式构造函数：初始化解算器，加载配置参数和硬件外参
   * @param config_path 配置文件路径（YAML/JSON格式），包含相机内参、畸变系数、硬件安装外参
   * @details 1. 从配置文件加载相机内参（camera_matrix_）、畸变系数（distort_coeffs_）；
   *          2. 加载硬件安装外参：相机-云台的旋转/平移、云台-IMU机体的旋转/平移；
   *          3. 初始化云台→世界的转换矩阵（初始为单位矩阵，后续通过IMU数据更新）；
   *          4. 校验配置参数的有效性（如矩阵维度、非空性），避免解算错误。
   * @note explicit关键字：禁止隐式转换（如直接用字符串初始化Solver），强制显式传参，提升代码健壮性
   * @throw 若配置文件不存在、参数格式错误、矩阵维度不匹配，会抛出异常（需上层捕获）
   */
  explicit Solver(const std::string & config_path);

  /**
   * @brief 获取云台坐标系到世界坐标系的旋转矩阵
   * @return Eigen::Matrix3d 3x3旋转矩阵（正交单位矩阵）
   * @details 只读接口，返回当前的云台→世界旋转关系，用于外部模块的坐标转换或状态查询
   * @const 方法不修改类内任何成员变量，仅做只读返回
   */
  Eigen::Matrix3d R_gimbal2world() const;

  /**
   * @brief 获取云台坐标系到世界坐标系的变换矩阵（注：原命名为T，实际返回旋转矩阵，与R_gimbal2world功能一致）
   * @return Eigen::Matrix3d 3x3旋转矩阵
   * @details 兼容外部模块的接口命名习惯，实际返回R_gimbal2world_，与R_gimbal2world()功能完全相同
   * @const 方法不修改类内任何成员变量，仅做只读返回
   */
  Eigen::Matrix3d T_gimbal2world() const;

  /**
   * @brief 更新云台坐标系到世界坐标系的旋转矩阵（通过四元数）
   * @param q 云台在世界坐标系中的姿态四元数（Eigen::Quaterniond）
   * @details 从IMU/云台姿态传感器获取四元数，转换为旋转矩阵并更新R_gimbal2world_，
   *          是实现**实时坐标解算**的关键接口——随云台姿态变化动态更新坐标转换关系。
   * @note 四元数需保证单位化，否则会导致旋转矩阵非正交，解算结果失真
   */
  void set_R_gimbal2world(const Eigen::Quaterniond & q);

  /**
   * @brief 核心解算接口：对检测到的装甲板进行全流程解算与角度优化
   * @param armor 输入输出参数：传入检测到的装甲板（含二维特征、类型、名称），
   *              解算后更新**三维世界坐标、云台瞄准角度、重投影误差**等关键参数
   * @details 执行完整解算流程：1. 基于PnP算法解算装甲板在相机坐标系下的三维位置；
   *          2. 通过多坐标系链式转换，将相机坐标转换为世界坐标系坐标；
   *          3. 优化云台偏航角（optimize_yaw），最小化重投影误差；
   *          4. 计算最终的云台瞄准角度（俯仰/偏航），赋值给armor相关成员。
   * @const 方法不修改类内成员变量，仅修改输入的armor对象
   */
  void solve(Armor & armor) const;

  /**
   * @brief 装甲板重投影：将世界坐标系下的三维装甲板投影到二维图像平面
   * @param xyz_in_world 装甲板在世界坐标系下的三维中心坐标（x,y,z）
   * @param yaw 云台偏航角（弧度），用于确定装甲板的姿态
   * @param type 装甲板类型（大/小），对应不同的物理尺寸
   * @param name 装甲板名称（数字/基地），辅助确定物理尺寸
   * @return std::vector<cv::Point2f> 装甲板四个角点在图像平面的像素坐标
   * @details 1. 根据装甲板类型/名称获取实际物理尺寸，生成世界坐标系下的装甲板角点；
   *          2. 通过多坐标系转换，将世界角点转换为相机坐标系下的点；
   *          3. 调用相机投影函数，结合内参/畸变系数，将相机三维点投影为二维像素点；
   *          4. 实现**从三维世界到二维图像**的反向投影，用于重投影误差计算和目标验证。
   * @const 方法不修改类内任何成员变量
   */
  std::vector<cv::Point2f> reproject_armor(
    const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const;

  /**
   * @brief 前哨站装甲板重投影误差计算（专属方法）
   * @param armor 检测到的前哨站装甲板对象（含二维像素角点）
   * @param picth 云台俯仰角（弧度，注意：原代码拼写错误，正确为pitch）
   * @return double 重投影误差值（像素单位），值越小表示投影越精准
   * @details 针对前哨站装甲板的特殊姿态，计算其实际检测角点与重投影角点的误差，
   *          用于前哨站目标的有效性校验和角度优化，是普通重投影误差的专属优化版本。
   */
  double oupost_reprojection_error(Armor armor, const double & picth);

  /**
   * @brief 世界坐标系到像素坐标系的投影：通用接口
   * @param worldPoints 世界坐标系下的三维点集（任意数量）
   * @return std::vector<cv::Point2f> 对应的二维像素点集
   * @details 实现**通用的三维世界点→二维像素点**投影，步骤为：
   *          1. 世界坐标→云台坐标→IMU坐标→相机坐标；
   *          2. 结合相机内参/畸变系数，将相机三维点投影为二维像素点；
   *          3. 是reproject_armor的底层通用实现，支持任意三维点的投影。
   * @const 方法不修改类内任何成员变量
   */
  std::vector<cv::Point2f> world2pixel(const std::vector<cv::Point3f> & worldPoints);

private:
  // ************************** 相机内参与畸变参数 **************************
  cv::Mat camera_matrix_;    // 相机内参矩阵（3x3）：[fx,0,cx; 0,fy,cy; 0,0,1]
                             // fx/fy：x/y方向焦距（像素）；cx/cy：主点坐标（像素）
                             // 作用：描述相机光学特性，实现相机三维点→二维像素点的投影
  cv::Mat distort_coeffs_;   // 相机畸变系数（1x5/1x8）：[k1,k2,p1,p2,k3(,k4,k5,k6)]
                             // k1-k6：径向畸变系数；p1-p2：切向畸变系数
                             // 作用：校正镜头光学畸变，提升投影/解算精度

  // ************************** 硬件安装外参（云台-IMU机体） **************************
  Eigen::Matrix3d R_gimbal2imubody_;  // 云台→IMU机体旋转矩阵（3x3）
                                      // 作用：描述云台相对于IMU机体的安装姿态，固定硬件参数（配置文件加载）
  Eigen::Vector3d T_gimbal2imubody_;  // 云台→IMU机体平移向量（3x1）
                                      // 作用：描述云台坐标系原点在IMU机体坐标系下的位置，固定硬件参数

  // ************************** 硬件安装外参（相机-云台） **************************
  Eigen::Matrix3d R_camera2gimbal_;   // 相机→云台旋转矩阵（3x3）
                                      // 作用：描述相机相对于云台的安装姿态，固定硬件参数（如相机向下倾斜30°）
  Eigen::Vector3d t_camera2gimbal_;   // 相机→云台平移向量（3x1）
                                      // 作用：描述相机光心在云台坐标系下的位置，固定硬件参数（如相机向前偏移5cm）

  // ************************** 实时姿态参数（云台-世界） **************************
  Eigen::Matrix3d R_gimbal2world_;    // 云台→世界旋转矩阵（3x3）：实时更新（通过IMU/云台姿态）
                                      // 作用：描述云台在世界坐标系中的实时朝向，动态坐标转换核心
  Eigen::Vector3d T_gimbal2world_;    // 云台→世界平移向量（3x1）：世界坐标系下云台原点的位置
                                      // 作用：实现坐标系的平移转换，配合旋转矩阵完成完整的刚体变换

  // ************************** 私有核心方法 **************************
  /**
   * @brief 云台偏航角优化：最小化装甲板重投影误差，提升瞄准精度
   * @param armor 输入输出参数：传入解算后的装甲板，优化后更新最优偏航角
   * @details 基于**非线性优化**（如梯度下降、二分法）寻找最优偏航角，
   *          使装甲板实际检测像素角点与重投影像素角点的误差最小，
   *          解决因云台姿态误差、检测误差导致的瞄准偏差，是提升瞄准精度的关键步骤。
   * @const 方法不修改类内成员变量，仅修改输入的armor对象
   */
  void optimize_yaw(Armor & armor) const;

  /**
   * @brief 通用装甲板重投影误差计算
   * @param armor 检测到的装甲板对象（含实际检测像素角点）
   * @param yaw 待评估的云台偏航角（弧度）
   * @param inclined 装甲板倾斜角（弧度）：描述装甲板相对于水平面的姿态
   * @return double 重投影误差值（像素单位）：误差越小，偏航角越优
   * @details 1. 根据传入的yaw和inclined，重投影装甲板得到理论像素角点；
   *          2. 计算理论角点与实际检测角点的欧氏距离均值/和，作为重投影误差；
   *          3. 是optimize_yaw的核心评估函数，为偏航角优化提供误差指标。
   * @const 方法不修改类内任何成员变量
   */
  double armor_reprojection_error(const Armor & armor, double yaw, const double & inclined) const;

  /**
   * @brief SJTU成本函数（上海交通大学定制版误差计算）
   * @param cv_refs 实际检测到的装甲板像素角点（参考点）
   * @param cv_pts 重投影得到的装甲板像素角点（评估点）
   * @param inclined 装甲板倾斜角（弧度）
   * @return double 成本函数值：值越小表示参考点与评估点越匹配
   * @details 针对装甲板检测的实际场景，对传统重投影误差做**加权优化**，
   *          对装甲板不同位置的角点赋予不同权重（如灯条角点权重更高），
   *          提升误差计算的鲁棒性，使优化结果更贴合实际瞄准需求。
   * @const 方法不修改类内任何成员变量
   */
  double SJTU_cost(
    const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
    const double & inclined) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__SOLVER_HPP
