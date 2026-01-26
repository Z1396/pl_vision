#include "target.hpp"

#include <numeric>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Target::Target(
  const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
  Eigen::VectorXd P0_dig)
: name(armor.name),                // 继承装甲板名称（如"outpost"、"robot"等）
  armor_type(armor.type),          // 继承装甲板类型（如小装甲板、大装甲板）
  jumped(false),                   // 标记是否目标发生跳变（初始为false）
  last_id(0),                      // 上一个装甲板ID（初始为0）
  update_count_(0),                // 目标更新次数计数器（初始为0）
  armor_num_(armor_num),           // 目标包含的装甲板数量
  t_(t),                           // 记录目标创建时间戳
  is_switch_(false),               // 标记是否发生装甲板切换（初始为false）
  is_converged_(false),            // 标记滤波器是否收敛（初始为false）
  switch_count_(0),                 // 装甲板切换计数器（初始为0）
  outpost_state_("wu")
{
  auto r = radius;                 // 目标旋转半径（从参数获取）
  priority = armor.priority;       // 继承装甲板优先级（用于跟踪目标选择）
  
  // 从装甲板获取世界坐标系下的3D坐标(x,y,z)和姿态角(yaw,pitch,roll)
  const Eigen::VectorXd & xyz = armor.xyz_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;

  // 计算目标旋转中心坐标（基于装甲板位置和旋转半径）
  // 装甲板通常在目标旋转圆周上，旋转中心 = 装甲板位置 + 半径×方向向量
  auto center_x = xyz[0] + r * std::cos(ypr[0]);  // x坐标（yaw角的余弦分量）
  auto center_y = xyz[1] + r * std::sin(ypr[0]);  // y坐标（yaw角的正弦分量）
  auto center_z = xyz[2];                         // z坐标（高度，暂不考虑旋转影响）

  // 初始化卡尔曼滤波器的状态向量x0（共11个维度）
  // 状态定义：[x, vx, y, vy, z, vz, a, w, r, 0, 0]
  // 各维度含义：
  // x,y,z: 旋转中心的3D位置
  // vx,vy,vz: x,y,z方向的速度
  // a: 角度（yaw角）
  // w: 角速度
  // r: 旋转半径
  // 最后两个0: 预留维度（可用于长度、高度等）
  if(armor.name == ArmorName::outpost)
  {
    Eigen::VectorXd x0{{center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0.15, 0.3}};
      // 初始化状态协方差矩阵P0（对角矩阵）
    // 对角元素由参数P0_dig提供，代表各状态量的初始不确定性
    Eigen::MatrixXd P0 = P0_dig.asDiagonal();  //P0_dig.asDiagonal()：这是 Eigen 库提供的一个方法，用于将向量转换为对角矩阵
                                             //整体含义：将向量 P0_dig 的元素依次作为对角矩阵 P0 的主对角线元素

    // 定义状态加法函数（用于卡尔曼滤波器的状态更新）
    // 处理角度的周期性（将角度限制在[-π, π]范围内，避免角度跳变）
    auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd 
    {  
      Eigen::VectorXd c = a + b;          // 正常向量加法
      c[6] = tools::limit_rad(c[6]);     // 第6维是角度，特殊处理：限制在[-π, π]
      return c;
    };

    // 初始化扩展卡尔曼滤波器（EKF）
    // 参数：初始状态x0、初始协方差P0、状态加法函数x_add
    ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);
  }
  else
  {
    Eigen::VectorXd x0{{center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0, 0.0}};
    // 初始化状态协方差矩阵P0（对角矩阵）
    // 对角元素由参数P0_dig提供，代表各状态量的初始不确定性
    Eigen::MatrixXd P0 = P0_dig.asDiagonal();  //P0_dig.asDiagonal()：这是 Eigen 库提供的一个方法，用于将向量转换为对角矩阵
                                              //整体含义：将向量 P0_dig 的元素依次作为对角矩阵 P0 的主对角线元素

    // 定义状态加法函数（用于卡尔曼滤波器的状态更新）
    // 处理角度的周期性（将角度限制在[-π, π]范围内，避免角度跳变）
    auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd 
    {
      Eigen::VectorXd c = a + b;          // 正常向量加法
      c[6] = tools::limit_rad(c[6]);     // 第6维是角度，特殊处理：限制在[-π, π]
      return c;
    };

    // 初始化扩展卡尔曼滤波器（EKF）
    // 参数：初始状态x0、初始协方差P0、状态加法函数x_add
    ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);
  }
}

std::string Target::outpost_state() const { return outpost_state_; }

Target::Target(double x, double vyaw, double radius, double h) : armor_num_(4)
{
  Eigen::VectorXd x0{{x, 0, 0, 0, 0, 0, 0, vyaw, radius, 0, h}};
  Eigen::VectorXd P0_dig{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd 
  {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);  //初始化滤波器（预测量、预测量协方差）
}

void Target::predict(std::chrono::steady_clock::time_point t)
{
  auto dt = tools::delta_time(t, t_);
  predict(dt);
  t_ = t;
}

/**
 * @brief 预测目标的状态（基于时间间隔dt）
 * 
 * 该函数实现目标状态的预测过程，通过构建状态转移矩阵和过程噪声矩阵，
 * 结合扩展卡尔曼滤波器（EKF）对目标的未来状态（位置、速度、角度等）进行预测，
 * 并针对特定目标（如前哨站）的运动特性进行特殊处理。
 * 
 * @param dt 预测未来dt时间点后的目标状态（单位：秒）
 */


void Target::predict(double dt)
{
    // 构建11×11的状态转移矩阵F
    // clang-format off
    Eigen::MatrixXd F
    {
        {1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0},
        {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
        {0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0},
        {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0},
        {0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0},
        {0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0},
        {0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0},
        {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0},
        {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0},
        {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0},
        {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1}
    };
    // clang-format on
    double v_z_delta = 1e-6;  // 高度差的过程噪声系数
    // 分段白噪声模型：设置过程噪声方差
    double v1, v2;
    if (name == ArmorName::outpost) 
    {
        v1 = 10;
        v2 = 0.1;
    } else 
    {
        v1 = 100;
        v2 = 400;
    }

    // 计算过程噪声矩阵的系数
    auto a = dt * dt * dt * dt / 4;
    auto b = dt * dt * dt / 2;
    auto c = dt * dt;

    // 构建11×11的过程噪声矩阵Q
    // clang-format off
    // 重构Q矩阵
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(11, 11);
    // 原有维度的噪声
    Q.block<8,8>(0,0) = Eigen::MatrixXd{
        {a * v1, b * v1,      0,      0,      0,      0,      0,      0},
        {b * v1, c * v1,      0,      0,      0,      0,      0,      0},
        {     0,      0, a * v1, b * v1,      0,      0,      0,      0},
        {     0,      0, b * v1, c * v1,      0,      0,      0,      0},
        {     0,      0,      0,      0, a * v1, b * v1,      0,      0},
        {     0,      0,      0,      0, b * v1, c * v1,      0,      0},
        {     0,      0,      0,      0,      0,      0, a * v2, b * v2},
        {     0,      0,      0,      0,      0,      0, b * v2, c * v2}
    };
    // 高度差的噪声（极小值）
    Q(9,9) = c * v_z_delta;   // delta_z_01 过程噪声
    Q(10,10) = c * v_z_delta; // delta_z_02 过程噪声
    // clang-format on

    // 定义状态预测的映射函数，确保角度在[-π, π]范围内
    auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd 
    {
        Eigen::VectorXd x_prior = F * x;
        x_prior[6] = tools::limit_rad(x_prior[6]);
        return x_prior;
    };

    // 前哨站转速特殊处理
    if (this->convergened() && this->name == ArmorName::outpost && std::abs(this->ekf_.x[7]) > 2)
        this->ekf_.x[7] = this->ekf_.x[7] > 0 ? 2.51 : -2.51;

    // 调用EKF的预测方法
    ekf_.predict(F, Q, f);

    // ========== 新增：Z轴物理约束（位置+速度限幅） ==========
    // 1. Z轴速度（vz，x[5]）限幅（与X轴速度约束对齐）
    const double max_vz = 0.001;  // 单位：m/s，可根据实际场景调整
    ekf_.x[5] = std::clamp(ekf_.x[5], -max_vz, max_vz);
}

/**
 * @brief 根据新检测到的装甲板更新目标状态（匹配装甲板ID、判断切换状态、触发参数更新）
 * 
 * 该函数是目标跟踪的核心更新接口，主要完成三件事：
 * 1. 从目标的多个预设装甲板（如机器人的多个装甲板）中，匹配与当前检测装甲板最接近的装甲板ID；
 * 2. 判断是否发生装甲板切换（如从机器人的“前装甲板”切换到“侧装甲板”），记录切换状态和次数；
 * 3. 调用update_ypda函数，基于匹配的装甲板ID更新目标的位姿（yaw/pitch/distance/angle）参数。
 * 
 * @param armor 新检测到的装甲板对象（提供当前帧的位姿、角度等信息）
 */
void Target::update(const Armor & armor)
{
    // 1. 初始化装甲板匹配相关变量：目标装甲板ID、最小角度误差
    int id;                                  // 最终匹配的装甲板ID
    auto min_angle_error = 1e10;             // 初始最小角度误差设为极大值
    const std::vector<Eigen::Vector4d> & xyza_list = armor_xyza_list();  // 目标的所有预设装甲板参数（x/y/z坐标 + angle角度，共4维）

    // 2. 构建“装甲板参数-ID”对列表（便于后续排序和匹配）
    std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
    for (int i = 0; i < armor_num_; i++) // armor_num_：目标的预设装甲板总数（如机器人有4个装甲板则为4）
    {   
        xyza_i_list.push_back({xyza_list[i], i});  // 存储“第i个装甲板的xyza参数 + 其ID=i”
    }

    // 3. 对装甲板列表按“距离”从小到大排序（优先匹配距离当前目标更近的装甲板）
    std::sort(
        xyza_i_list.begin(), xyza_i_list.end(),
        [](const std::pair<Eigen::Vector4d, int> & a, const std::pair<Eigen::Vector4d, int> & b) 
        {
            // head(3)：取xyza的前3个元素（x/y/z坐标），转换为ypd（yaw/pitch/distance）
            Eigen::Vector3d ypd1 = tools::xyz2ypd(a.first.head(3));
            Eigen::Vector3d ypd2 = tools::xyz2ypd(b.first.head(3));
            return ypd1[2] < ypd2[2];  // 按distance（ypd[2]）升序排序，距离近的在前
        });

    // 4. 从排序后的前3个最近装甲板中，匹配角度误差最小的装甲板ID
    // 取前3个：平衡“距离近”和“角度匹配”，避免因单个近距装甲板角度偏差导致误匹配
    for (int i = 0; i < 2; i++) 
    {
        const auto & xyza = xyza_i_list[i].first;  // 当前候选装甲板的xyza参数
        Eigen::Vector3d ypd = tools::xyz2ypd(xyza.head(3));  // 候选装甲板的ypd（yaw/pitch/distance）
        
        // 计算角度误差：yaw角误差（装甲板实际yaw vs 候选装甲板预设angle） + 方位角误差（装甲板实际ypd-yaw vs 候选ypd-yaw）
        // limit_rad：将角度差限制在[-π, π]，避免跨±π的角度差计算错误（如350°与10°的差应为20°而非340°）
        auto angle_error = std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza[3])) +
                           std::abs(tools::limit_rad(armor.ypd_in_world[0] - ypd[0]));

        // 更新最小角度误差和匹配ID（误差更小则替换）
        if (std::abs(angle_error) < std::abs(min_angle_error)) 
        {
            id = xyza_i_list[i].second;  // 记录当前匹配的装甲板ID
            min_angle_error = angle_error;  // 更新最小角度误差
        }
    }

    // 5. 判断是否发生“跳跃匹配”（匹配ID不为0，通常0为默认主装甲板，非0则视为跳跃）
    if (id != 0) jumped = true;  //因为在记录id时，0通常代表主装甲板，非0则表示切换到其他装甲板，视为跳跃
    else jumped = false;

    // 6. 判断是否发生装甲板切换（当前匹配ID与上一帧ID不同则为切换）
    if (id != last_id) 
    {
        is_switch_ = true;  // 标记为切换状态
    } else 
    {
        is_switch_ = false; // 标记为未切换
    }

    // 7. 累计切换次数（切换状态时计数+1，用于判断目标是否频繁切换，避免跟踪不稳定）
    if (is_switch_) switch_count_++;

    // 8. 更新状态变量：上一帧匹配ID、总更新次数
    last_id = id;          // 保存当前ID为下一帧的“上一帧ID”
    update_count_++;       // 累计目标更新次数（用于判断目标跟踪稳定性）

    // 9. 基于匹配的装甲板ID，更新目标的ypda（yaw/pitch/distance/angle）核心参数
    update_ypda(armor, id);
}

/**
 * @brief 基于检测到的装甲板和匹配的ID，更新目标的EKF状态（核心观测更新步骤）
 * 
 * 该函数是目标跟踪的观测更新核心，通过构建观测模型（雅可比矩阵、测量噪声矩阵）、
 * 定义非线性观测映射和角度差处理规则，将当前装甲板的观测信息（ypd方位角/俯仰角/距离、ypr偏航角）
 * 输入扩展卡尔曼滤波器（EKF），完成状态的修正与更新，提升目标状态估计的精度。
 * 
 * @param armor 新检测到的装甲板对象（提供观测到的ypd、ypr等信息）
 * @param id 匹配到的装甲板ID（对应目标身上的特定装甲板，用于确定观测模型参数）
 */

// 注意：需确保Target类中已有以下成员变量：
// - ekf_: EKF类的实例，包含状态x（Eigen::VectorXd）
// - armor_num_: 装甲板数量（int）
// - name: ArmorName类型（如outpost、base等）
// - 其他必要成员（如convergened()方法）

void Target::update_ypda(const Armor & armor, int id)
{
    // 1. 计算观测模型的雅可比矩阵H（线性化观测函数）
    Eigen::MatrixXd H = h_jacobian(ekf_.x, id);

    // 2. 计算测量噪声矩阵R（对角矩阵，反映观测值的不确定性）
    auto center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
    auto delta_angle = tools::limit_rad(armor.ypr_in_world[0] - center_yaw);
    
    // 构建噪声对角元素（R_dig）
    Eigen::VectorXd R_dig{
        {4e-3, 4e-3,
        (log(std::abs(delta_angle) + 1) + 1),
        (log(std::abs(armor.ypd_in_world[2]) + 1) / 200) + 9e-2}
    };

    // 由对角元素构建测量噪声矩阵R
    Eigen::MatrixXd R = R_dig.asDiagonal();

    // 3. 定义非线性观测映射函数h：EKF状态向量x → 观测向量z
    auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d 
    {
        Eigen::VectorXd xyz = h_armor_xyz(x, id);
        Eigen::VectorXd ypd = tools::xyz2ypd(xyz);
        auto angle = tools::limit_rad(x[6] + id * 2 * M_PI / armor_num_);  // 替换CV_PI为M_PI（更通用）
        return {ypd[0], ypd[1], ypd[2], angle};
    };

    // 4. 定义观测向量的减法规则z_subtract：处理角度量的差值
    auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd 
    {
        Eigen::VectorXd c = a - b;
        c[0] = tools::limit_rad(c[0]);
        c[1] = tools::limit_rad(c[1]);
        c[3] = tools::limit_rad(c[3]);
        return c;
    };

    // 5. 构建实际观测向量z
    const Eigen::VectorXd & ypd = armor.ypd_in_world;
    const Eigen::VectorXd & ypr = armor.ypr_in_world;
    Eigen::VectorXd z{{ypd[0], ypd[1], ypd[2], ypr[0]}};

    // 6. 调用EKF的观测更新接口
    ekf_.update(z, H, R, h, z_subtract);
}


/**
 * @brief 获取卡尔曼滤波器的当前状态向量（只读副本）
 * 
 * 状态向量是目标跟踪的核心数据，存储目标的位置、速度、角度等关键信息，
 * 本接口返回状态向量的拷贝，避免外部直接操作内部滤波器状态，同时提供
 * 便捷的状态数据访问方式（无需了解EKF类的内部结构）。
 * 
 * 状态向量维度：11维，各维度定义如下（与EKF初始化时的x0对应）：
 * [0] x: 目标旋转中心x坐标（世界坐标系）
 * [1] vx: x方向速度
 * [2] y: 目标旋转中心y坐标（世界坐标系）
 * [3] vy: y方向速度
 * [4] z: 目标旋转中心z坐标（世界坐标系，高度）
 * [5] vz: z方向速度
 * [6] yaw: 目标偏航角（世界坐标系，已限制在[-π, π]）
 * [7] vyaw: 偏航角速度
 * [8] r: 目标旋转半径（装甲板到旋转中心的距离）
 * [9] reserved1: 预留维度（用于长轴补偿等扩展）
 * [10] reserved2: 预留维度（用于高度补偿等扩展）
 * 
 * @return Eigen::VectorXd 状态向量的只读副本（外部修改不影响内部状态）
 * @note 函数末尾const：表明该函数不修改Target类的任何成员变量，是纯只读操作
 * @note 返回值为值传递：虽有拷贝开销，但能最大程度保护内部核心状态不被外部误改
 */
Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

/**
 * @brief 获取内部扩展卡尔曼滤波器（EKF）的只读引用
 * 
 * 提供对完整EKF实例的访问接口，支持外部读取滤波器的所有公开只读数据（如状态向量x、
 * 协方差矩阵P等），但通过const限制禁止修改内部状态，兼顾数据访问灵活性和安全性。
 * 
 * 适用场景：
 * 1. 需要读取EKF的非状态数据（如协方差矩阵P，用于判断状态估计的可信度）
 * 2. 需要调用EKF的只读成员函数（如自定义的状态校验逻辑）
 * 3. 批量获取EKF相关数据，避免多次调用单个状态接口
 * 
 * @return const tools::ExtendedKalmanFilter& EKF实例的只读引用（无拷贝，效率极高）
 * @note 双重const保护：
 *       1. 函数末尾const：保证函数内部不修改Target类成员（包括ekf_）
 *       2. 返回值const：外部通过引用只能读取EKF数据，不能调用修改接口（如setX、update等）
 * @note 引用生命周期：返回的引用与当前Target对象生命周期一致，对象销毁后不可再使用
 * @note 封装思想：隐藏EKF的实现细节，仅暴露安全访问接口，避免外部直接操作内部滤波逻辑
 */
const tools::ExtendedKalmanFilter & Target::ekf() const { return ekf_; }

/**
 * @brief 生成目标所有预设装甲板的“坐标-角度”列表（xyza_list）
 * 
 * 该函数为目标跟踪中的装甲板匹配提供核心数据：基于当前扩展卡尔曼滤波器（EKF）的状态估计，
 * 计算目标身上所有预设装甲板（如机器人的多个装甲板）在世界坐标系下的3D坐标（x/y/z）和对应的角度（angle），
 * 并以4维向量（x,y,z,angle）的形式组成列表返回，用于后续与检测到的装甲板进行匹配。
 * 
 * @return std::vector<Eigen::Vector4d> 装甲板xyza列表，每个元素为Eigen::Vector4d（x,y,z,angle）
 *         - x/y/z：装甲板在世界坐标系下的3D坐标（单位：米）
 *         - angle：装甲板对应的偏航角（yaw），限制在[-π, π]范围内（单位：弧度）
 */
std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
    // 初始化存储装甲板xyza数据的列表
    std::vector<Eigen::Vector4d> _armor_xyza_list;

    // 遍历目标的所有预设装甲板（armor_num_为目标装甲板总数，如机器人有4个装甲板则循环4次）
    for (int i = 0; i < armor_num_; i++) 
    {
        // 1. 计算当前装甲板的angle（偏航角）
        // ekf_.x[6]：EKF状态向量的第7个元素（0基索引），通常是目标整体的基准偏航角（yaw）
        // i * 2*CV_PI / armor_num_：当前装甲板相对于基准yaw的角度偏移（假设装甲板均匀分布在目标周身）
        // 例：4个装甲板均匀分布时，相邻装甲板偏移90°（2π/4 = π/2弧度），i=0→0°，i=1→90°，i=2→180°，i=3→270°
        // limit_rad：将计算出的角度限制在[-π, π]范围内，避免角度溢出（如360°→0°，-10°保持-10°）
        auto angle = tools::limit_rad(ekf_.x[6] + i * 2 * CV_PI / armor_num_);

        // 2. 计算当前装甲板在世界坐标系下的3D坐标（x/y/z）
        // h_armor_xyz：自定义的坐标映射函数，输入EKF状态向量（ekf_.x）和装甲板索引（i），
        // 输出该装甲板在世界坐标系下的x/y/z坐标（考虑目标整体位置、姿态对单个装甲板位置的影响）
        Eigen::Vector3d xyz = h_armor_xyz(ekf_.x, i);

        // 3. 将当前装甲板的x/y/z/angle组成4维向量，加入列表
        // Eigen::Vector4d{x,y,z,angle}：前3个元素为坐标，第4个元素为角度
        _armor_xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
    }

    // 返回生成的装甲板xyza列表
    return _armor_xyza_list;
}
/// @brief 
/// @return 
bool Target::diverged() const 
{
  // 检查短轴半径是否在合理范围内(5cm~50cm)
  auto r_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5;
  
  // 检查长轴半径(短轴+补偿)是否在合理范围内(5cm~50cm) 
  auto l_ok = ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[8] + ekf_.x[9] < 0.5;

  // 如果长短轴都在合理范围内,说明没有发散
  if (r_ok && l_ok) return false;

  // 如果有任一轴超出范围,打印调试信息并返回发散状态
  tools::logger()->debug("[Target] r={:.3f}, l={:.3f}", ekf_.x[8], ekf_.x[9]);
  return true;
}

bool Target::convergened()
{
  if (this->name != ArmorName::outpost && update_count_ > 3 && !this->diverged()) {
    is_converged_ = true;
  }

  //前哨站特殊判断
  if (this->name == ArmorName::outpost && update_count_ > 10 && !this->diverged()) {
    is_converged_ = true;
  }

  return is_converged_;
}

/**
 * @brief 计算目标身上特定ID装甲板在世界坐标系下的3D中心坐标
 * 
 * 该函数是装甲板坐标映射的核心实现，基于目标的EKF状态向量（包含位置、姿态、尺寸等信息），
 * 结合装甲板ID，考虑装甲板的“长短轴差异”（不同ID装甲板可能对应不同尺寸），
 * 最终计算出该装甲板中心在世界坐标系下的x/y/z坐标，为后续装甲板匹配和跟踪提供位置基准。
 * 
 * @param x EKF状态向量（Eigen::VectorXd），包含目标的全局状态（如位置、角度、尺寸参数等）
 * @param id 装甲板ID（0~armor_num_-1），标识目标身上的某一个特定装甲板
 * @return Eigen::Vector3d 该装甲板中心在世界坐标系下的3D坐标（x, y, z），单位通常为米
 */
// 计算出装甲板中心的坐标（考虑长短轴）
Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
    auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
    auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);
    auto r = (use_l_h) ? x[8] + x[9] : x[8];

    auto armor_x = x[0] - r * std::cos(angle);
    auto armor_y = x[2] - r * std::sin(angle);
    
    // 核心修改：前哨站根据装甲板ID应用高度差
    double armor_z = x[4];
    if (name == ArmorName::outpost && x[7] > 0) 
    {
        switch(id) {
            case 0:  // 基准装甲板
                armor_z = x[4];
                break;
            case 1:  // 01号装甲板（加第一个高度差）
                armor_z = x[4] - x[9];
                break;
            case 2:  // 02号装甲板（加第二个高度差）
                armor_z = x[4] - x[10];
                break;
            default:
                armor_z = x[4];
                break;
        }
    }    
    else if (name == ArmorName::outpost && x[7] < 0) 
    {
        switch(id) {
            case 0:  // 基准装甲板
                armor_z = x[4];
                break;
            case 1:  // 01号装甲板（加第一个高度差）
                armor_z = x[4] + x[9];
                break;
            case 2:  // 02号装甲板（加第二个高度差）
                armor_z = x[4] + x[10];
                break;
            default:
                armor_z = x[4];
                break;
        }
    }
     else 
    {
        // 非前哨站保持原有逻辑
        armor_z = (use_l_h) ? x[4] + x[10] : x[4];
    }

    return {armor_x, armor_y, armor_z};
}

/**
 * @brief 计算目标雅可比矩阵，用于状态估计中的观测模型线性化
 * @param x 状态向量，包含目标的位置、姿态等状态信息
 * @param id 当前装甲板的编号（用于区分不同位置的装甲板）
 * @return 雅可比矩阵H，描述观测值对状态变量的偏导数关系
 */
Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd & x, int id) const
{
  // 计算当前装甲板的角度（考虑目标自身旋转角和装甲板安装位置）
  // x[6]是目标的旋转角，通过limit_rad确保角度在[-π, π]范围内
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  
  // 判断是否使用长装甲板参数（针对4个装甲板的情况，1号和3号为长装甲板）
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  // 计算旋转半径r：长装甲板使用x[8]+x[9]，短装甲板仅使用x[8]
  // x[8]可能代表基础半径，x[9]代表长装甲板的额外长度
  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  
  // 计算x坐标对角度(angle)的偏导数：dx/da = r*sin(angle)
  auto dx_da = r * std::sin(angle);
  // 计算y坐标对角度(angle)的偏导数：dy/da = -r*cos(angle)
  auto dy_da = -r * std::cos(angle);

  // 计算x坐标对半径(r)的偏导数：dx/dr = -cos(angle)
  auto dx_dr = -std::cos(angle);
  // 计算y坐标对半径(r)的偏导数：dy/dr = -sin(angle)
  auto dy_dr = -std::sin(angle);
  
  // 计算x坐标对长装甲板额外长度(l)的偏导数：仅长装甲板有值
  auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
  // 计算y坐标对长装甲板额外长度(l)的偏导数：仅长装甲板有值
  auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;

  // 计算z坐标对高度(h)的偏导数：仅长装甲板有高度变化
  auto dz_dh = (use_l_h) ? 1.0 : 0.0;


  // 定义装甲板位置(x,y,z)和角度(angle)对状态变量的雅可比矩阵
  // 矩阵维度为4x11，对应4个观测值(xyz+angle)对11个状态变量的偏导
  // clang-format off
    // 新增：前哨站高度差的偏导数
    double dz_ddelta01 = 0.0;
    double dz_ddelta02 = 0.0;
    if (name == ArmorName::outpost) {
        if (id == 1) dz_ddelta01 = 1.0;  // 01号装甲板对delta_z_01的偏导
        if (id == 2) dz_ddelta02 = 1.0;  // 02号装甲板对delta_z_02的偏导
    }

    // 修改H_armor_xyza矩阵，添加高度差的偏导数
    // clang-format off
    Eigen::MatrixXd H_armor_xyza{
        {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},
        {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},
        {0, 0, 0, 0, 1, 0,     0, 0,     0, dz_ddelta01, dz_ddelta02},  // 新增高度差偏导
        {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0}
    };
  // clang-format on

  // 获取当前装甲板的坐标(xyz)
  Eigen::VectorXd armor_xyz = h_armor_xyz(x, id);
  
  // 计算从xyz坐标到ypd(仰角、方位角、距离)的雅可比矩阵
  // ypd是视觉系统常用的观测表示方式
  Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);
  
  // 构建ypd+angle对xyz+angle的雅可比矩阵
  // 将角度项直接传递(最后一行)，ypd项通过H_armor_ypd转换
  // clang-format off
  Eigen::MatrixXd H_armor_ypda{
    {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},  // 仰角(yaw)的偏导行
    {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},  // 方位角(pitch)的偏导行
    {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},  // 距离(distance)的偏导行
    {                0,                 0,                 0, 1}   // 角度的偏导行(直接传递)
  };
  // clang-format on
    // 新增日志：打印H矩阵中z轴对delta_z_01/02的偏导数
    if (name == ArmorName::outpost) {
        std::cout << "[debug] id=" << id 
                  << ", dz_ddelta01=" << dz_ddelta01 
                  << ", dz_ddelta02=" << dz_ddelta02 << std::endl;
        std::cout << "[debug] H matrix col9: " << H_armor_xyza(2,9) 
                  << ", col10: " << H_armor_xyza(2,10) << std::endl;
    }
  // 复合雅可比矩阵：ypd+angle对状态变量的总偏导 = H_armor_ypda * H_armor_xyza
  return H_armor_ypda * H_armor_xyza;
}

bool Target::checkinit() { return isinit; }

}  // namespace auto_aim
