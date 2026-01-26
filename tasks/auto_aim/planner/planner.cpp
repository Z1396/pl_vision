#include "planner.hpp"  // 对应类声明头文件（包含Planner类、Plan结构体等定义）

#include <vector>  // 标准容器库，用于存储配置文件中的向量参数（如Q矩阵、R矩阵）

// 自定义工具类头文件
#include "tools/math_tools.hpp"    // 数学辅助工具（角度限制、插值等）
#include "tools/trajectory.hpp"    // 弹道计算工具（子弹飞行时间、俯仰角补偿）
#include "tools/yaml.hpp"          // YAML配置文件解析工具（加载MPC参数、偏移量等）

using namespace std::chrono_literals;  // 简化时间字面量使用（如1us、1ms）

namespace auto_aim
{
/**
 * @brief Planner类构造函数（初始化配置参数和MPC求解器）
 * @param config_path YAML配置文件路径（存储偏移量、MPC权重、约束等参数）
 */
Planner::Planner(const std::string & config_path)
{
  // 1. 加载YAML配置文件
  auto yaml = tools::load(config_path);

  // 2. 读取机械偏移量（配置文件中为角度值，转换为弧度：除以57.3≈180/π）
  yaw_offset_ = tools::read<double>(yaml, "yaw_offset") / 57.3;    // 偏航角安装偏移
  pitch_offset_ = tools::read<double>(yaml, "pitch_offset") / 57.3;// 俯仰角安装偏移

  // 3. 读取发射相关参数
  fire_thresh_ = tools::read<double>(yaml, "fire_thresh");         // 发射允许的最大瞄准误差
  decision_speed_ = tools::read<double>(yaml, "decision_speed");   // 子弹高低速分界阈值（m/s）
  high_speed_delay_time_ = tools::read<double>(yaml, "high_speed_delay_time");// 高速子弹发射延迟（s）
  low_speed_delay_time_ = tools::read<double>(yaml, "low_speed_delay_time");// 低速子弹发射延迟（s）

  // 4. 初始化偏航角和俯仰角MPC求解器（加载对应通道的MPC参数）
  setup_yaw_solver(config_path);
  setup_pitch_solver(config_path);
}

/**
 * @brief 规划接口（重载：输入确定目标）
 * @details 核心流程：子弹速度校验→目标弹道补偿预测→生成参考轨迹→MPC求解→输出控制指令+发射决策
 * @param target 确定的目标对象（含装甲板位置、运动状态等）
 * @param bullet_speed 实际子弹速度（m/s）
 * @return 规划结果（控制指令、发射标志、角度/角速度/加速度状态）
 */
Plan Planner::plan(Target target, double bullet_speed)
{
  // 0. 子弹速度校验（限制在合理范围，避免异常值影响弹道计算）
  if (bullet_speed < 10 || bullet_speed > 25) {
    bullet_speed = 22;  // 速度异常时使用默认值22m/s
  }

  // 1. 计算子弹飞行时间（用于目标未来位置预测，实现弹道补偿）
  Eigen::Vector3d xyz;          // 选中的装甲板3D坐标（x/y/z）
  auto min_dist = 1e10;         // 初始化最小距离为极大值
  // 遍历目标所有装甲板，选择距离最近的装甲板作为瞄准点（提高命中率）
  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();  // 计算装甲板x-y平面距离（忽略z轴高度）
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();            // 记录最近装甲板的3D坐标
    }
  }
  // 计算子弹弹道：输入子弹速度、水平距离、高度差，输出飞行时间和补偿俯仰角
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
  // 预测子弹飞行时间后目标的位置（补偿目标运动，实现"提前瞄准"）
  target.predict(bullet_traj.fly_time);

  // 2. 生成MPC所需的参考轨迹（目标未来HORIZON步的角度/角速度参考值）
  double yaw0;                   // 初始偏航角参考值（用于轨迹归一化）
  Trajectory traj;               // 参考轨迹（4×HORIZON矩阵）
  try {
    yaw0 = aim(target, bullet_speed)(0);  // 计算初始瞄准偏航角
    traj = get_trajectory(target, yaw0, bullet_speed);  // 生成完整参考轨迹
  } catch (const std::exception & e) {
    // 轨迹生成失败（如弹道无解），输出警告并返回"停止控制"指令
    tools::logger()->warn("Unsolvable target {:.2f}", bullet_speed);
    return {false};  // Plan.control = false，停止控制
  }

  // 3. 偏航角MPC求解（跟踪参考轨迹，输出最优控制量）
  Eigen::VectorXd x0(2);         // 偏航通道初始状态（角度、角速度）
  x0 << traj(0, 0), traj(1, 0);  // 初始状态取自参考轨迹第0步
  tiny_set_x0(yaw_solver_, x0);  // 设置MPC求解器初始状态

  // 设置偏航通道参考轨迹（取参考轨迹前2行：偏航角、偏航角速度）
  yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
  tiny_solve(yaw_solver_);       // 调用TinyMPC求解器计算最优控制

  // 4. 俯仰角MPC求解（与偏航通道逻辑一致，独立解算）
  x0 << traj(2, 0), traj(3, 0);  // 俯仰通道初始状态（角度、角速度）
  tiny_set_x0(pitch_solver_, x0);  // 设置初始状态

  // 设置俯仰通道参考轨迹（取参考轨迹后2行：俯仰角、俯仰角速度）
  pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
  tiny_solve(pitch_solver_);     // 求解俯仰通道最优控制

  // 5. 封装规划结果（Plan结构体）
  Plan plan;
  plan.control = true;  // 控制使能标志（轨迹求解成功，允许控制）

  // 目标角度（取参考轨迹中间步HALF_HORIZON的角度，叠加安装偏移）
  plan.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);  // 偏航目标角（限制在[-π, π]）
  plan.target_pitch = traj(2, HALF_HORIZON);                         // 俯仰目标角

  // 偏航通道当前状态（取自MPC求解结果的中间步，叠加初始偏航角y0）
  plan.yaw = tools::limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);  // 当前偏航角
  plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);                        // 当前偏航角速度
  plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);                        // 当前偏航角加速度（控制量）

  // 俯仰通道当前状态（取自MPC求解结果的中间步）
  plan.pitch = pitch_solver_->work->x(0, HALF_HORIZON);                        // 当前俯仰角
  plan.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);                    // 当前俯仰角速度
  plan.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);                    // 当前俯仰角加速度（控制量）

  // 6. 发射决策（判断未来shoot_offset_步的瞄准误差是否小于阈值）
  auto shoot_offset_ = 2;  // 发射提前步长（预测未来2步的瞄准状态）
  plan.fire =
    // 计算未来步的偏航+俯仰联合误差（欧氏距离）
    std::hypot(
      traj(0, HALF_HORIZON + shoot_offset_) - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset_),
      traj(2, HALF_HORIZON + shoot_offset_) -
        pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset_)) < fire_thresh_;
  return plan;  // 返回完整规划结果
}

/**
 * @brief 规划接口（重载：输入可选目标）
 * @details 处理"未检测到目标"或"需延迟瞄准"的场景，适配目标检测的不确定性
 * @param target 可选目标（std::optional：有值=检测到目标，无值=未检测到）
 * @param bullet_speed 子弹速度（m/s）
 * @return 规划结果（未检测到目标则返回"停止控制"）
 */
Plan Planner::plan(std::optional<Target> target, double bullet_speed)
{
  // 未检测到目标，返回"停止控制"指令（control=false，fire=false）
  if (!target.has_value()) return {false};

  // 检测到目标：根据目标角速度选择发射延迟时间（动态调整提前瞄准时机）
  double delay_time =
    // 目标角速度（ekf_x()[7]为目标角速度状态，取自EKF滤波结果）大于分界值→高速延迟，否则→低速延迟
    std::abs(target->ekf_x()[7]) > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;

  // 计算未来延迟时间后的时间点（用于目标预测）
  auto future = std::chrono::steady_clock::now() + std::chrono::microseconds(int(delay_time * 1e6));

  // 预测目标在未来延迟时间点的状态（补偿发射延迟，提高命中率）
  target->predict(future);

  // 调用重载的plan函数（传入确定目标），返回规划结果
  return plan(*target, bullet_speed);
}

/**
 * @brief 初始化偏航角MPC求解器
 * @details 配置MPC的系统模型、权重矩阵、约束条件，基于TinyMPC库实现
 * @param config_path YAML配置文件路径（读取偏航通道MPC参数）
 */
void Planner::setup_yaw_solver(const std::string & config_path)
{
  // 加载YAML配置文件，读取偏航通道MPC参数
  auto yaml = tools::load(config_path);
  auto max_yaw_acc = tools::read<double>(yaml, "max_yaw_acc");  // 偏航角最大加速度（控制约束）
  auto Q_yaw = tools::read<std::vector<double>>(yaml, "Q_yaw");  // 状态权重矩阵（角度、角速度权重）
  auto R_yaw = tools::read<std::vector<double>>(yaml, "R_yaw");  // 控制权重矩阵（加速度权重）

  // 定义MPC系统模型（离散时间状态方程：x(k+1) = A*x(k) + B*u(k) + f）
  Eigen::MatrixXd A{{1, DT}, {0, 1}};  // 状态矩阵（2×2）：角度-角速度一阶积分模型
  Eigen::MatrixXd B{{0}, {DT}};        // 输入矩阵（2×1）：加速度对状态的影响
  Eigen::VectorXd f{{0, 0}};           // 偏移向量（无系统偏差，设为0）

  // 构造权重矩阵（对角矩阵，Q为状态权重，R为控制权重）
  Eigen::Matrix<double, 2, 1> Q(Q_yaw.data());
  Eigen::Matrix<double, 1, 1> R(R_yaw.data());

  // 初始化TinyMPC求解器
  // 参数说明：求解器指针、A/B/f矩阵、Q/R权重、终端成本系数、状态维度(2)、输入维度(1)、预测时域(HORIZON)、是否使用warm-start(0)
  tiny_setup(&yaw_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

  // 设置状态和控制约束（边界值）
  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);  // 状态下界（角度/角速度无硬约束）
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);  // 状态上界
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_yaw_acc);  // 控制下界（最小加速度）
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_yaw_acc);  // 控制上界（最大加速度）
  tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max);  // 应用约束

  yaw_solver_->settings->max_iter = 10;  // MPC最大迭代次数（平衡求解速度和精度）
}

/**
 * @brief 初始化俯仰角MPC求解器
 * @details 逻辑与偏航角求解器完全一致，仅参数（最大加速度、权重）不同
 * @param config_path YAML配置文件路径（读取俯仰通道MPC参数）
 */
void Planner::setup_pitch_solver(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto max_pitch_acc = tools::read<double>(yaml, "max_pitch_acc");  // 俯仰角最大加速度
  auto Q_pitch = tools::read<std::vector<double>>(yaml, "Q_pitch");  // 俯仰通道状态权重
  auto R_pitch = tools::read<std::vector<double>>(yaml, "R_pitch");  // 俯仰通道控制权重

  // 系统模型与偏航通道一致（角度-角速度一阶积分模型）
  Eigen::MatrixXd A{{1, DT}, {0, 1}};
  Eigen::MatrixXd B{{0}, {DT}};
  Eigen::VectorXd f{{0, 0}};

  // 构造权重矩阵
  Eigen::Matrix<double, 2, 1> Q(Q_pitch.data());
  Eigen::Matrix<double, 1, 1> R(R_pitch.data());

  // 初始化TinyMPC求解器
  tiny_setup(&pitch_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

  // 设置约束（状态无硬约束，控制约束为最大俯仰加速度）
  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_pitch_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_pitch_acc);
  tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min, u_max);

  pitch_solver_->settings->max_iter = 10;  // 最大迭代次数
}

/**
 * @brief 计算瞄准角（含弹道补偿和机械偏移）
 * @details 核心功能：选择瞄准点→计算方位角→弹道俯仰角补偿→叠加安装偏移
 * @param target 目标对象（含装甲板位置）
 * @param bullet_speed 子弹速度（m/s）
 * @return 2维向量（[补偿后偏航角, 补偿后俯仰角]，单位：rad）
 * @throws std::runtime_error 弹道无解时抛出异常
 */
Eigen::Matrix<double, 2, 1> Planner::aim(const Target & target, double bullet_speed)
{
  Eigen::Vector3d xyz;          // 选中装甲板的3D坐标（x/y/z）
  double yaw;                   // 选中装甲板的偏航角（目标自身姿态）
  auto min_dist = 1e10;         // 初始化最小距离为极大值

  // 遍历所有装甲板，选择距离最近的作为瞄准点（提高瞄准精度）
  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();  // 计算x-y平面距离
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();            // 记录3D坐标
      yaw = xyza[3];                   // 记录装甲板偏航角
    }
  }
  debug_xyza = Eigen::Vector4d(xyz.x(), xyz.y(), xyz.z(), yaw);  // 存储调试数据（x/y/z/偏航角）

  // 计算方位角（目标在x-y平面的角度：atan2(y, x)）
  auto azim = std::atan2(xyz.y(), xyz.x());
  // 计算弹道补偿：输入子弹速度、水平距离、高度差，输出补偿后的俯仰角
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
  if (bullet_traj.unsolvable) {
    // 弹道无解（如距离过远、子弹速度不足），抛出异常
    throw std::runtime_error("Unsolvable bullet trajectory!");
  }

  // 返回最终瞄准角：偏航角（方位角+安装偏移，限制在[-π, π]），俯仰角（弹道补偿+安装偏移，负号为方向适配）
  return {tools::limit_rad(azim + yaw_offset_), -bullet_traj.pitch - pitch_offset_};
}

/**
 * @brief 生成MPC参考轨迹（目标未来HORIZON步的角度/角速度）
 * @details 基于目标运动预测，生成平滑的参考轨迹，用于MPC跟踪
 * @param target 目标对象（需预测未来状态）
 * @param yaw0 初始偏航角（用于轨迹归一化，使初始状态为0）
 * @param bullet_speed 子弹速度（用于弹道补偿）
 * @return 参考轨迹（4×HORIZON矩阵：偏航角、偏航角速度、俯仰角、俯仰角速度）
 */
Trajectory Planner::get_trajectory(Target & target, double yaw0, double bullet_speed)
{
  Trajectory traj;  // 输出参考轨迹

  // 1. 反向预测目标状态（为了计算第0步的角速度，需要前一步的角度）
  target.predict(-DT * (HALF_HORIZON + 1));  // 回退(HALF_HORIZON+1)*DT时间
  auto yaw_pitch_last = aim(target, bullet_speed);  // 记录回退状态的瞄准角（前一步）

  // 2. 前进到轨迹起始点（第0步对应的目标状态）
  target.predict(DT);  // 前进DT时间，此时对应轨迹第0步的目标状态
  auto yaw_pitch = aim(target, bullet_speed);  // 记录当前瞄准角（当前步）

  // 3. 生成HORIZON步参考轨迹（循环预测目标未来状态）
  for (int i = 0; i < HORIZON; i++) {
    target.predict(DT);  // 预测下一步目标状态（前进DT时间）
    auto yaw_pitch_next = aim(target, bullet_speed);  // 记录下一步瞄准角（下一步）

    // 计算角速度：中心差分（(下一步-前一步)/(2*DT)），提高平滑性
    auto yaw_vel = tools::limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2 * DT);  // 偏航角速度（限制角度差）
    auto pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * DT);  // 俯仰角速度

    // 填充当前步轨迹数据：偏航角（归一化后）、偏航角速度、俯仰角、俯仰角速度
    traj.col(i) << tools::limit_rad(yaw_pitch(0) - yaw0), yaw_vel, yaw_pitch(1), pitch_vel;

    // 更新状态：前一步=当前步，当前步=下一步（为下一次迭代做准备）
    yaw_pitch_last = yaw_pitch;
    yaw_pitch = yaw_pitch_next;
  }

  return traj;
}

}  // namespace auto_aim