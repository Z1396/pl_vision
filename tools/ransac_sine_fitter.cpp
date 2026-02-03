// 引入RANSAC正弦拟合类头文件，包含类声明、成员函数原型、结果结构体及参数定义
#include "ransac_sine_fitter.hpp"

// 引入Eigen稠密矩阵库：用于构建超定方程组、执行SVD分解实现最小二乘求解
#include <Eigen/Dense>
// 引入C++算法库：用于随机洗牌、序列生成等操作
#include <algorithm>
// 引入C++数学库：用于三角函数、开方、绝对值等核心计算
#include <cmath>
// 引入C++随机数库：用于RANSAC的随机采样和角频率随机生成
#include <random>

// 工具模块命名空间，隔离通用工具类，避免与业务模块命名冲突
namespace tools
{

/**
 * @brief 构造函数：初始化RANSAC正弦拟合核心参数
 * @details 配置RANSAC迭代策略、拟合阈值、角频率范围，初始化随机数生成器，
 *          是拟合器的入口初始化逻辑，所有核心超参数一次性配置完成
 * @param max_iterations RANSAC最大迭代次数，迭代越多拟合精度越高但耗时越长
 * @param threshold 拟合内点判定阈值（残差绝对值），小于该值则为有效内点
 * @param min_omega 待拟合正弦曲线最小角频率（ω），限定拟合频率下限
 * @param max_omega 待拟合正弦曲线最大角频率（ω），限定拟合频率上限
 * @note 随机数生成器采用std::random_device获取真随机种子，保证采样随机性
 */
RansacSineFitter::RansacSineFitter(
  int max_iterations, double threshold, double min_omega, double max_omega)
: max_iterations_(max_iterations),    // RANSAC迭代次数
  threshold_(threshold),              // 内点判定残差阈值
  min_omega_(min_omega),              // 角频率最小值
  max_omega_(max_omega),              // 角频率最大值
  gen_(std::random_device{}())        // 随机数生成器（真随机种子）
{
}

/**
 * @brief 拟合数据添加接口
 * @details 向拟合器添加时序数据点 (t, v)，支持增量式数据输入，
 *          包含数据有效性校验和缓存清理逻辑，保证拟合数据的时序连续性
 * @param t 时序自变量（如时间），要求单调递增
 * @param v 时序因变量（如传感器测量值），为待拟合的原始数据
 * @note 1. 若新数据与最后一个数据的时间差超过5，清空历史数据（时序断裂，重新开始拟合）；
 *       2. 采用emplace_back直接构造数据对，避免拷贝开销，提升数据添加效率
 */
void RansacSineFitter::add_data(double t, double v)
{
  // 时序断裂检测：数据时间间隔过大则清空历史缓存，重新积累拟合数据
  if (fit_data_.size() > 0 && (t - fit_data_.back().first > 5)) fit_data_.clear();
  // 添加新数据点到拟合缓存，std::make_pair构造(t, v)键值对
  fit_data_.emplace_back(std::make_pair(t, v));
}

/**
 * @brief RANSAC正弦曲线核心拟合接口
 * @details 实现**随机采样一致性（RANSAC）**的完整拟合逻辑：随机采样→局部模型求解→内点评估→最优模型选择，
 *          拟合目标为正弦曲线通用形式：y = A·sin(ωt + φ) + C，其中A(振幅)、ω(角频率)、φ(初相位)、C(直流偏置)
 * @note 1. 数据量小于3个时直接返回（正弦曲线拟合至少需要3个点）；
 *       2. 拟合完成后若数据量超过150，弹出头部数据（滑动窗口缓存，避免内存溢出）；
 *       3. 采用角频率随机采样+最小二乘求解，将非线性拟合转化为线性拟合，降低求解复杂度
 */
void RansacSineFitter::fit()
{
  // 数据量校验：正弦曲线拟合至少需要3个点，不足则不执行拟合
  if (fit_data_.size() < 3) return;

  // 初始化角频率随机分布：在[min_omega_, max_omega_]范围内生成均匀分布的随机角频率
  std::uniform_real_distribution<double> omega_dist(min_omega_, max_omega_);
  // 生成数据索引序列：用于后续随机洗牌采样
  std::vector<size_t> indices(fit_data_.size());
  std::iota(indices.begin(), indices.end(), 0);  // 生成0,1,2,...,n-1的连续索引

  // RANSAC核心迭代循环：多次采样拟合，选择内点最多的模型作为最优模型
  for (int iter = 0; iter < max_iterations_; ++iter) {
    // 随机洗牌索引：实现数据点的随机无放回采样
    std::shuffle(indices.begin(), indices.end(), gen_);

    // 随机采样3个数据点：构成RANSAC的最小采样集（刚好满足拟合条件）
    std::vector<std::pair<double, double>> sample;
    for (int i = 0; i < 3; ++i) {
      sample.push_back(fit_data_[indices[i]]);
    }

    // 随机生成当前迭代的角频率ω：将非线性拟合的ω解耦为随机采样，转化为线性拟合问题
    double omega = omega_dist(gen_);
    Eigen::Vector3d params;  // 存储局部模型拟合参数 [A1, A2, C]
    // 求解局部模型：若求解失败（如矩阵奇异），跳过本次迭代
    if (!fit_partial_model(sample, omega, params)) continue;

    // 从线性参数[A1, A2]转换为正弦曲线振幅A和初相位φ（三角恒等变换）
    double A1 = params(0);  // 线性拟合的sin(ωt)系数
    double A2 = params(1);  // 线性拟合的cos(ωt)系数
    double C = params(2);   // 直流偏置项
    double A = std::sqrt(A1 * A1 + A2 * A2);  // 振幅：A = √(A1²+A2²)
    double phi = std::atan2(A2, A1);          // 初相位：φ = arctan2(A2, A1)

    // 评估当前模型的内点数量：统计所有数据中残差小于阈值的点的个数
    int inlier_count = evaluate_inliers(A, omega, phi, C);

    // 最优模型更新：若当前模型内点数量多于历史最优，更新最优结果
    if (inlier_count > best_result_.inliers) {
      best_result_.A = A;          // 最优振幅
      best_result_.omega = omega;  // 最优角频率
      best_result_.phi = phi;      // 最优初相位
      best_result_.C = C;          // 最优直流偏置
      best_result_.inliers = inlier_count;  // 最优模型内点数量
    }
  }

  // 滑动窗口缓存：数据量超过150时弹出头部旧数据，避免内存堆积，适配持续数据输入场景
  if (fit_data_.size() > 150) fit_data_.pop_front();
}

/**
 * @brief 局部模型最小二乘求解函数
 * @details 将正弦曲线拟合转化为**线性最小二乘问题**，通过SVD分解求解超定方程组，
 *          输入固定角频率ω，求解线性参数[A1, A2, C]，对应变换：A·sin(ωt+φ)+C = A1·sin(ωt)+A2·cos(ωt)+C
 * @param sample RANSAC随机采样的3个数据点，构成最小拟合集
 * @param omega 固定角频率（本次迭代随机生成）
 * @param params 输出线性拟合参数，Eigen::Vector3d([A1, A2, C])
 * @return bool 求解是否成功，true=成功，false=失败（如矩阵奇异、不可逆）
 * @note 采用Eigen的BDCSVD分解（分块对角SVD），适合中小型矩阵，求解稳定且效率高
 */
bool RansacSineFitter::fit_partial_model(
  const std::vector<std::pair<double, double>> & sample, double omega, Eigen::Vector3d & params)
{
  // 构建线性方程组：X·params = Y，其中X为设计矩阵，Y为观测向量
  Eigen::MatrixXd X(sample.size(), 3);  // 设计矩阵：n行3列（n=3，最小采样集）
  Eigen::VectorXd Y(sample.size());     // 观测向量：n行1列

  // 填充设计矩阵和观测向量：逐点计算sin(ωt)、cos(ωt)、1.0
  for (size_t i = 0; i < sample.size(); ++i) {
    double t = sample[i].first;
    double y = sample[i].second;
    X(i, 0) = std::sin(omega * t);  // 第一列：sin(ωt)
    X(i, 1) = std::cos(omega * t);  // 第二列：cos(ωt)
    X(i, 2) = 1.0;                  // 第三列：常数项1（对应直流偏置C）
    Y(i) = y;                       // 观测向量：原始测量值
  }

  try {
    // SVD分解求解线性最小二乘：ComputeThinU/ComputeThinV表示计算薄矩阵，提升效率
    params = X.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Y);
    return true;  // 求解成功，返回参数
  } catch (...) {
    // 捕获所有异常（如矩阵奇异、数值溢出），求解失败则返回false
    return false;
  }
}

/**
 * @brief 模型内点数量评估函数
 * @details 根据当前拟合的正弦曲线模型，遍历所有数据点计算预测值与真实值的残差，
 *          统计残差小于阈值的内点数量，为RANSAC最优模型选择提供依据
 * @param A 正弦曲线振幅
 * @param omega 正弦曲线角频率
 * @param phi 正弦曲线初相位
 * @param C 正弦曲线直流偏置
 * @return int 内点数量，数量越多表示模型拟合效果越好
 * @note 预测值计算严格遵循正弦曲线通用公式：y_pred = A·sin(ωt + φ) + C
 */
int RansacSineFitter::evaluate_inliers(double A, double omega, double phi, double C)
{
  int count = 0;  // 内点计数器
  // 遍历所有拟合数据点，逐点评估残差
  for (const auto & p : fit_data_) {
    double t = p.first;  // 自变量t
    double y = p.second; // 真实值y
    // 计算模型预测值
    double pred = A * std::sin(omega * t + phi) + C;
    // 残差绝对值小于阈值则为内点，计数器自增
    if (std::abs(y - pred) < threshold_) {
      ++count;
    }
  }
  return count;  // 返回当前模型的内点总数
}

}  // namespace tools
