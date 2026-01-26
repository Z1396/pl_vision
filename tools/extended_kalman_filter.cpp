#include "extended_kalman_filter.hpp"
#include "tools/logger.hpp"
#include <numeric>

namespace tools
{
/**
 * @brief 扩展卡尔曼滤波器（Extended Kalman Filter, EKF）的构造函数
 * 
 * 扩展卡尔曼滤波器用于处理非线性系统的状态估计，该构造函数初始化滤波器的
 * 初始状态、协方差矩阵，并设置状态更新的加法函数，同时初始化用于记录滤波过程
 * 统计信息的数据结构。
 * 
 * @param x0 初始状态向量（Eigen::VectorXd），包含系统初始时刻的状态估计值
 * @param P0 初始状态协方差矩阵（Eigen::MatrixXd），表示初始状态估计的不确定性
 * @param x_add 状态加法函数（函数对象），定义状态向量的加法运算（因可能涉及角度等周期性状态，需自定义加法规则）
 */
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
: x(x0), P(P0), I(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add(x_add)
{
  // 初始化滤波过程中的统计数据（存储在data字典中，用于后续分析或调试）
  data["residual_yaw"] = 0.0;           // 偏航角（yaw）的残差（测量值与预测值的差）
  data["residual_pitch"] = 0.0;         // 俯仰角（pitch）的残差
  data["residual_distance"] = 0.0;      // 距离的残差
  data["residual_angle"] = 0.0;         // 角度的综合残差
  data["nis"] = 0.0;                    // 归一化创新平方（Normalized Innovation Squared），用于验证测量噪声模型
  data["nees"] = 0.0;                   // 归一化估计误差平方（Normalized Estimation Error Squared），用于验证状态估计性能
  data["nis_fail"] = 0.0;               // NIS验证失败的次数（超过置信区间）
  data["nees_fail"] = 0.0;              // NEES验证失败的次数
  data["recent_nis_failures"] = 0.0;    // 最近连续NIS验证失败的次数
}

Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)
{
  return predict(F, Q, [&](const Eigen::VectorXd & x) { return F * x; });
}

/**
 * @brief 扩展卡尔曼滤波器的预测步骤
 * 
 * 预测步骤根据系统的状态转移模型，基于当前状态估计下一时刻的状态及其不确定性。
 * 适用于线性或非线性系统（通过传入不同的状态转移函数f实现）。
 * 
 * @param F 状态转移矩阵（线性化后的的Jacobian矩阵）
 *          描述状态变量之间的线性转移关系，维度为n×n（n为状态向量维度）
 * @param Q 过程噪声协方差矩阵
 *          描述系统模型不确定性（如随机加速度），维度为n×n
 * @param f 状态转移函数（非线性或线性）
 *          输入当前状态x，输出预测的下一时刻状态x_prior
 *          对于线性系统，f(x) = F * x；对于非线性系统，f为自定义非线性映射
 * 
 * @return Eigen::VectorXd 预测后的状态向量x_prior
 */
Eigen::VectorXd ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
{
  // 1. 更新状态协方差矩阵（预测不确定性）
  // 公式：P_k⁻ = F_k * P_{k-1}⁺ * F_k^T + Q_k
  // 物理意义：结合上一时刻的后验协方差和过程噪声，得到当前时刻的先验协方差
  P = F * P * F.transpose() + Q;

  // 2. 计算预测状态（先验估计）
  // 通过状态转移函数f对当前状态x进行预测，得到下一时刻的状态x_prior
  x = f(x);

  // 返回预测后的状态向量
  return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  return update(z, H, R, [&](const Eigen::VectorXd & x) { return H * x; }, z_subtract);
}

/**
 * @brief EKF观测更新环节：融合传感器测量值，修正预测状态并评估滤波稳定性
 * @param z 传感器实际测量值向量（如4维ypd：yaw, pitch, distance, armor_angle）
 * @param H 观测模型雅可比矩阵（维度：测量值维度×状态维度，如4×11）
 * @param R 观测噪声协方差矩阵（维度：测量值维度×测量值维度，描述传感器误差）
 * @param h 观测模型函数：输入状态x，输出“预测的测量值”h(x)（如状态→ypd）
 * @param z_subtract 测量值差值计算函数：输入z(实际测量)和h(x)(预测测量)，输出归一化差值（如角度差值归一到[-π, π]）
 * @return 更新后的EKF状态向量x（修正后的目标状态）
 */
Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
    // 1. 保存更新前的预测状态（x_prior），用于后续NEES检验（状态误差评估）
    Eigen::VectorXd x_prior = x;

    // 2. 计算卡尔曼增益K（核心：决定“测量值”与“预测状态”的融合权重）
    // K = P * H^T * (H*P*H^T + R)^(-1)
    // 其中：
    // - P：更新前的状态协方差矩阵（预测的不确定性）
    // - H^T：雅可比矩阵的转置
    // - H*P*H^T：预测状态不确定性在“观测空间”的投影
    // - R：观测噪声不确定性
    // - (H*P*H^T + R)：总观测不确定性（预测投影不确定性 + 传感器噪声）
    // 物理意义：K越大，说明测量值越可靠，滤波越信任测量值；K越小，越信任预测状态
    Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

    // 3. 稳定更新状态协方差矩阵P（后验协方差）
    // 公式：P = (I - K*H) * P * (I - K*H)^T + K*R*K^T
    // 传统简化公式为 P = (I - K*H)*P，但该公式忽略观测噪声对协方差的影响，可能导致数值不稳定
    // 此处使用“Joseph形式”的协方差更新，保证P的正定性（数值更稳定，适用于工程场景）
    // 参考：https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
    P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();

    // 4. 修正EKF状态向量x（核心：用测量值误差修正预测状态）
    // 步骤：
    // a. 计算测量残差（实际测量 - 预测测量）：residual = z - h(x)（通过z_subtract处理归一化）
    // b. 计算状态修正量：K * residual（卡尔曼增益×残差，即“该用多少测量误差修正状态”）
    // c. 用x_add函数更新状态（x_add需适配状态量类型，如角度归一化，此处函数未展示）
    x = x_add(x, K * z_subtract(z, h(x)));

    /// 5. 卡方检验：评估滤波有效性（NIS：测量残差检验；NEES：状态误差检验）
    // 5.1 计算测量残差（更新后的状态对应的预测测量与实际测量的差值）
    Eigen::VectorXd residual = z_subtract(z, h(x));

    // 5.2 计算NIS（Normalized Innovation Squared，归一化残差平方和）：检验测量残差是否符合噪声模型
    // S = H*P*H^T + R（总观测不确定性，与计算K时的分母一致，可复用但此处重新计算以确保正确性）
    Eigen::MatrixXd S = H * P * H.transpose() + R;
    // NIS = residual^T * S^(-1) * residual（残差在“观测空间”的归一化值）
    double nis = residual.transpose() * S.inverse() * residual;

    // 5.3 计算NEES（Normalized Estimation Error Squared，归一化估计误差平方和）：检验状态修正是否合理
    // NEES = (x - x_prior)^T * P^(-1) * (x - x_prior)（状态修正量在“状态空间”的归一化值）
    double nees = (x - x_prior).transpose() * P.inverse() * (x - x_prior);

    // 5.4 卡方检验阈值设置（自由度=4，置信水平95%）
    // 自由度=测量值维度（如4维ypd），查卡方分布表得：P(χ²(4) ≤ 0.711) = 0.05，P(χ²(4) ≤ 9.49) = 0.95
    // 此处阈值取0.711（实际工程中通常取双侧阈值，此处可能为简化仅判断上限或下限，需结合场景调整）
    constexpr double nis_threshold = 0.711;
    constexpr double nees_threshold = 0.711;

    // 5.5 统计检验失败次数（用于后续滤波异常判断）
    if (nis > nis_threshold) {  // NIS超过阈值：测量残差异常（如传感器数据跳变、观测模型错误）
        nis_count_++; 
        data["nis_fail"] = 1;   // 记录失败标记（用于日志/调试）
    }
    if (nees > nees_threshold) {  // NEES超过阈值：状态修正异常（如预测模型错误、协方差矩阵设置不合理）
        nees_count_++; 
        data["nees_fail"] = 1;   // 记录失败标记
    }
    total_count_++;  // 累计总更新次数（用于计算失败率）
    last_nis = nis;  // 保存最新NIS值（用于实时监控）

    // 5.6 滑动窗口统计近期NIS失败率（避免单次失败误判，提升检验鲁棒性）
    recent_nis_failures.push_back(nis > nis_threshold ? 1 : 0);  // 1=失败，0=成功
    if (recent_nis_failures.size() > window_size) {  // 窗口大小超限，移除最早数据
        recent_nis_failures.pop_front();
    }
    // 计算近期失败率：失败次数/窗口总次数
    int recent_failures = std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
    double recent_rate = static_cast<double>(recent_failures) / recent_nis_failures.size();

    // 6. 记录调试/监控数据（用于后续分析滤波性能，如残差大小、检验结果）
    data["residual_yaw"] = residual[0];       // yaw测量残差
    data["residual_pitch"] = residual[1];     // pitch测量残差
    data["residual_distance"] = residual[2];  // distance测量残差
    data["residual_angle"] = residual[3];     // armor_angle测量残差
    data["nis"] = nis;                        // 当前NIS值
    data["nees"] = nees;                      // 当前NEES值
    data["recent_nis_failures"] = recent_rate;// 近期NIS失败率
    // 在ExtendedKalmanFilter::update函数末尾添加（仅调试用）
    // 打印第9/10维状态值和协方差
    if (x.size() >= 11) {
        tools::logger()->debug(
            "EKF state: delta_z_01={:.6f}, delta_z_02={:.6f}, P9={:.8f}, P10={:.8f}",
            x[9], x[10], P(9,9), P(10,10)
        );
    }


    // 7. 返回更新后的状态向量（修正后的目标状态）
    return x;
}

}  // namespace tools