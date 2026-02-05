// 头文件保护指令：替代传统#ifndef/#define/#endif，编译效率更高，防止重复包含导致的类/结构体多重定义错误
#pragma once

// 引入Eigen稠密矩阵库：用于构建线性方程组，执行SVD分解实现最小二乘参数求解
#include <Eigen/Dense>
// 引入C++双端队列容器：存储时序拟合数据，支持高效的头部弹出（适配滑动窗口）和尾部插入
#include <deque>
// 引入C++标准输入输出库：基础控制台打印（预留调试使用）
#include <iostream>
// 引入C++随机数库：用于RANSAC随机采样和角频率随机生成，实现鲁棒拟合
#include <random>
// 引入C++向量容器：存储RANSAC随机采样的最小数据集、数据索引等
#include <vector>

// 工具模块命名空间：隔离通用算法工具类，避免与业务模块（如auto_aim、omniperception）命名冲突，
// 本类为通用型正弦曲线拟合工具，可被各模块跨场景调用
namespace tools
{

/**
 * @brief 基于RANSAC的正弦曲线鲁棒拟合类
 * @details 实现**随机采样一致性（RANSAC）**算法的正弦曲线拟合，针对含噪声/异常值的时序数据，
 *          鲁棒拟合出正弦曲线通用模型：y = A·sin(ωt + φ) + C，能有效剔除外点干扰，
 *          支持增量式数据添加、时序连续性校验、滑动窗口缓存，适配传感器实时数据处理等工业场景；
 *          核心设计将非线性正弦拟合转化为线性最小二乘问题，降低求解复杂度，提升拟合效率与稳定性。
 * @note 1. 鲁棒性：基于RANSAC算法，对异常值（外点）不敏感，即使数据中存在大量噪声仍能精准拟合；
 *       2. 实时性：支持增量式数据输入，拟合完成后自动维护滑动窗口，避免内存堆积；
 *       3. 通用性：拟合参数（迭代次数、阈值、角频率范围）可配置，适配不同频率、不同噪声水平的正弦曲线拟合。
 */
class RansacSineFitter
{
public:
  /**
   * @brief 正弦曲线拟合结果结构体
   * @details 封装拟合得到的正弦曲线核心参数及模型评价指标，所有参数均具有明确物理意义，
   *          为拟合结果的统一输出格式，便于上层模块直接使用拟合参数做后续计算/分析。
   */
  struct Result
  {
    double A = 0.0;        // 正弦曲线振幅（物理意义：振动/信号的幅值大小，A>0）
    double omega = 0.0;    // 角频率（ω=2πf，f为实际频率，单位：rad/s）
    double phi = 0.0;      // 初相位（单位：rad，范围[-π, π]，决定曲线的左右偏移）
    double C = 0.0;        // 直流偏置/基线值（决定曲线的上下偏移，拟合非零均值正弦曲线）
    int inliers = 0;       // 最优模型的内点数量（评价指标：数量越多，模型拟合效果越好，鲁棒性越强）
  };
  Result best_result_;     ///< 全局最优拟合结果，fit()执行后更新，上层可直接访问

  /**
   * @brief 构造函数：初始化RANSAC正弦拟合核心超参数
   * @details 配置RANSAC迭代策略、内点判定阈值、角频率拟合范围，初始化随机数生成器，
   *          所有核心超参数一次性配置，后续拟合过程中保持不变，适配固定场景的持续拟合。
   * @param max_iterations RANSAC最大迭代次数（正整数）：迭代越多拟合精度越高，but耗时越长，需平衡精度与实时性
   * @param threshold 内点判定残差阈值（非负数）：数据点真实值与模型预测值的残差绝对值小于该值，即为内点
   * @param min_omega 待拟合正弦曲线最小角频率（非负数）：限定ω的下限，避免拟合出无物理意义的低频值
   * @param max_omega 待拟合正弦曲线最大角频率（非负数，大于min_omega）：限定ω的上限，缩小拟合搜索范围
   * @note 随机数生成器采用std::mt19937（梅森旋转算法），伪随机数质量高，适合RANSAC随机采样场景
   */
  RansacSineFitter(int max_iterations, double threshold, double min_omega, double max_omega);

  /**
   * @brief 增量式数据添加接口
   * @details 向拟合器添加时序数据点 (t, v)，支持传感器实时数据的持续输入，
   *          内置时序连续性校验逻辑，自动维护拟合数据的有效性，是拟合的前置数据准备接口。
   * @param t 时序自变量（如时间、采样步长，要求单调递增），作为正弦曲线的自变量
   * @param v 时序因变量（如传感器测量值、采集信号值），作为待拟合的原始数据
   * @note 数据存储于std::deque，尾部插入效率为O(1)，适配高频数据添加场景。
   */
  void add_data(double t, double v);

  /**
   * @brief RANSAC核心拟合执行接口
   * @details 执行完整的RANSAC正弦曲线拟合流程：随机采样→局部模型求解→内点评估→最优模型选择，
   *          拟合完成后更新best_result_为全局最优结果，同时维护滑动窗口缓存，避免内存溢出；
   *          若拟合数据量不足（小于3个），直接返回不执行拟合（正弦曲线拟合至少需3个点）。
   * @note 本接口为无返回值设计，拟合结果通过公有成员best_result_向外暴露，简化调用逻辑。
   */
  void fit();

  /**
   * @brief 正弦曲线计算公式（内联成员函数）
   * @details 实现正弦曲线通用模型的计算，根据输入的参数计算指定自变量t对应的预测值，
   *          用于模型内点评估、后续预测等场景，是拟合模型的直接体现。
   * @param t 自变量（如时间）
   * @param A 振幅
   * @param omega 角频率
   * @param phi 初相位
   * @param C 直流偏置
   * @return double 正弦曲线在t处的预测值，计算公式：A·sin(ωt + φ) + C
   * @note inline特性（编译器自动优化），高频调用时无函数调用开销，提升计算效率。
   */
  double sine_function(double t, double A, double omega, double phi, double C)
  {
    return A * std::sin(omega * t + phi) + C;
  }

private:
  int max_iterations_;     ///< RANSAC最大迭代次数，构造函数初始化，拟合过程中只读
  double threshold_;       ///< 内点判定残差阈值，构造函数初始化，内点评估时使用
  double min_omega_;       ///< 拟合角频率下限，构造函数初始化，角频率随机生成时限定范围
  double max_omega_;       ///< 拟合角频率上限，构造函数初始化，角频率随机生成时限定范围
  std::mt19937 gen_;       ///< RANSAC随机数生成器（梅森旋转），用于随机采样和角频率生成，保证采样随机性
  std::deque<std::pair<double, double>> fit_data_;  ///< 拟合数据缓存（t, v），双端队列适配滑动窗口操作

  /**
   * @brief 局部模型线性最小二乘求解函数（私有核心实现）
   * @details RANSAC内部调用，将正弦曲线拟合转化为线性最小二乘问题，固定角频率ω后，
   *          求解线性参数[A1, A2, C]（对应变换：A·sin(ωt+φ)+C = A1·sin(ωt)+A2·cos(ωt)+C），
   *          通过Eigen的SVD分解求解超定方程组，保证求解稳定性。
   * @param sample RANSAC随机采样的最小数据集（3个数据点），满足正弦曲线拟合的最小条件
   * @param omega 本次迭代的固定角频率（随机生成）
   * @param params 输出参数：Eigen::Vector3d([A1, A2, C])，线性拟合的中间参数
   * @return bool 求解是否成功：true=成功（矩阵非奇异，求解有效），false=失败（如矩阵奇异、数值溢出）
   * @note 私有函数，仅被fit()调用，对外隐藏线性化求解的实现细节。
   */
  bool fit_partial_model(
    const std::vector<std::pair<double, double>> & sample, double omega, Eigen::Vector3d & params);

  /**
   * @brief 模型内点数量评估函数（私有核心实现）
   * @details RANSAC内部调用，遍历所有拟合数据，计算每个数据点与当前模型的残差，
   *          统计残差绝对值小于阈值的内点数量，为最优模型选择提供依据。
   * @param A 正弦曲线振幅
   * @param omega 角频率
   * @param phi 初相位
   * @param C 直流偏置
   * @return int 当前模型的内点总数，数量越多表示模型与数据的契合度越高
   * @note 私有函数，仅被fit()调用，对外隐藏内点评估的细节。
   */
  int evaluate_inliers(double A, double omega, double phi, double C);
};

}  // namespace tools
