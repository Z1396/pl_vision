#include "math_tools.hpp"

#include <cmath>
#include <opencv2/core.hpp>  // CV_PI

namespace tools
{
double limit_rad(double angle)
{
  while (angle > CV_PI) angle -= 2 * CV_PI;
  while (angle <= -CV_PI) angle += 2 * CV_PI;
  return angle;
}

/**
 * @brief 将四元数转换为欧拉角
 * 
 * 该函数实现了四元数到欧拉角的转换，支持自定义旋转轴顺序和内外旋模式，
 * 处理了特殊角度（如万向锁）的边界情况，并确保角度在[-π, π]范围内。
 * 
 * @param q 输入的四元数（Eigen::Quaterniond类型），表示三维旋转
 * @param axis0 第一个旋转轴（0=X, 1=Y, 2=Z）
 * @param axis1 第二个旋转轴（0=X, 1=Y, 2=Z，需与其他轴不同）
 * @param axis2 第三个旋转轴（0=X, 1=Y, 2=Z）
 * @param extrinsic 旋转模式：true=外旋（绕固定世界坐标系轴），false=内旋（绕当前物体坐标系轴）
 * @return Eigen::Vector3d 输出的欧拉角向量，三个元素分别对应绕axis0、axis1、axis2的旋转角度（弧度制）
 */
Eigen::Vector3d eulers(Eigen::Quaterniond q, int axis0, int axis1, int axis2, bool extrinsic)
{
    // 内旋模式下交换首尾旋转轴（内旋与外旋的轴顺序存在对偶关系）
    if (!extrinsic) std::swap(axis0, axis2);

    // 简化轴变量名：i=第一个轴，j=第二个轴，k=第三个轴
    auto i = axis0, j = axis1, k = axis2;
    // 判断是否为" proper Euler angles "（ proper 欧拉角：首尾旋转轴相同，如Z-Y-Z）
    auto is_proper = (i == k);
    // 若为proper欧拉角，临时调整k为第三个轴（用于中间计算）
    if (is_proper) k = 3 - i - j;
    // 计算符号因子，用于适配不同轴顺序的旋转方向
    auto sign = (i - j) * (j - k) * (k - i) / 2;

    double a, b, c, d;
    // 获取四元数的系数 [x, y, z, w]（注意Eigen中coeffs()返回顺序为(x,y,z,w)）
    Eigen::Vector4d xyzw = q.coeffs();
    
    // 根据是否为proper欧拉角，提取四元数系数并进行适配处理
    if (is_proper) 
    {
        a = xyzw[3];  // w分量
        b = xyzw[i];  // 第一个轴对应的分量
        c = xyzw[j];  // 第二个轴对应的分量
        d = xyzw[k] * sign;  // 第三个轴对应的分量（带符号调整）
    } else 
    {
        // 非proper欧拉角（如X-Y-Z）的系数提取方式
        a = xyzw[3] - xyzw[j];
        b = xyzw[i] + xyzw[k] * sign;
        c = xyzw[j] + xyzw[3];
        d = xyzw[k] * sign - xyzw[i];
    }

    Eigen::Vector3d eulers;  // 存储计算结果的欧拉角向量
    auto n2 = a * a + b * b + c * c + d * d;  // 归一化因子（四元数模长平方的相关计算）
    
    // 计算第二个旋转角（中间角），范围[0, π]
    eulers[1] = std::acos(2 * (a * a + b * b) / n2 - 1);

    // 计算半角和与半角差（用于推导首尾两个旋转角）
    auto half_sum = std::atan2(b, a);    // (角0 + 角2)/2
    auto half_diff = std::atan2(-d, c);  // (角0 - 角2)/2

    // 定义微小量，用于判断是否接近特殊角度（避免万向锁导致的计算不稳定）
    auto eps = 1e-7;
    // 判断是否远离特殊角度（非万向锁状态）
    auto safe1 = std::abs(eulers[1]) >= eps;          // 远离0弧度
    auto safe2 = std::abs(eulers[1] - CV_PI) >= eps;  // 远离π弧度
    auto safe = safe1 && safe2;

    if (safe) 
    {
        // 非特殊角度：正常计算首尾两个旋转角
        eulers[0] = half_sum + half_diff;  // 第一个旋转角 = 半角和 + 半角差
        eulers[2] = half_sum - half_diff;  // 第三个旋转角 = 半角和 - 半角差
    } else 
    {
        // 特殊角度（万向锁情况）：处理自由度丢失问题
        if (!extrinsic) 
        {
            // 内旋模式：固定第一个角为0，根据具体特殊角度计算第三个角
            eulers[0] = 0;
            if (!safe1) eulers[2] = 2 * half_sum;    // 接近0弧度时
            if (!safe2) eulers[2] = -2 * half_diff;  // 接近π弧度时
        } else 
        {
            // 外旋模式：固定第三个角为0，根据具体特殊角度计算第一个角
            eulers[2] = 0;
            if (!safe1) eulers[0] = 2 * half_sum;    // 接近0弧度时
            if (!safe2) eulers[0] = 2 * half_diff;   // 接近π弧度时
        }
    }

    // 将所有角度限制在[-π, π]范围内（标准化角度）
    for (int i = 0; i < 3; i++) eulers[i] = limit_rad(eulers[i]);

    // 非proper欧拉角的额外处理：调整角度偏移和符号
    if (!is_proper) 
    {
        eulers[2] *= sign;          // 修正第三个角的符号
        eulers[1] -= CV_PI / 2;     // 第二个角减去π/2偏移
    }

    // 内旋模式下交换首尾角度（还原轴顺序）
    if (!extrinsic) std::swap(eulers[0], eulers[2]);

    return eulers;
}

/**
 * @brief 将旋转矩阵转换为欧拉角
 * 
 * 该函数通过先将旋转矩阵转换为四元数，再调用四元数转欧拉角的重载函数，
 * 实现旋转矩阵到欧拉角的转换，支持自定义旋转轴顺序和内外旋模式。
 * 
 * @param R 输入的3×3旋转矩阵（Eigen::Matrix3d类型），需满足正交性（R*R^T=I）和行列式为1
 * @param axis0 第一个旋转轴（通常用0表示X轴，1表示Y轴，2表示Z轴）
 * @param axis1 第二个旋转轴（需与其他轴不同，遵循欧拉角旋转轴规则）
 * @param axis2 第三个旋转轴（通常与第一个轴相同，如Z-Y-Z、X-Y-X等）
 * @param extrinsic 旋转模式标志：true表示外旋（每次绕固定世界坐标系轴旋转），false表示内旋（每次绕当前物体坐标系轴旋转）
 * @return Eigen::Vector3d 输出的欧拉角向量，三个元素分别对应绕axis0、axis1、axis2的旋转角度（弧度制）
 */
Eigen::Vector3d eulers(Eigen::Matrix3d R, int axis0, int axis1, int axis2, bool extrinsic)
{
    // 将旋转矩阵R转换为四元数q（利用Eigen四元数的构造函数，自动从旋转矩阵初始化）
    Eigen::Quaterniond q(R);   // Eigen::Quaterniond是双精度浮点数四元数类型，用于表示三维旋转
    
    // 调用四元数转欧拉角的重载函数，返回转换结果
    return eulers(q, axis0, axis1, axis2, extrinsic);
}

Eigen::Matrix3d rotation_matrix(const Eigen::Vector3d & ypr)
{
  double roll = ypr[2];
  double pitch = ypr[1];
  double yaw = ypr[0];
  double cos_yaw = cos(yaw);
  double sin_yaw = sin(yaw);
  double cos_pitch = cos(pitch);
  double sin_pitch = sin(pitch);
  double cos_roll = cos(roll);
  double sin_roll = sin(roll);
  // clang-format off
    Eigen::Matrix3d R{
      {cos_yaw * cos_pitch, cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll, cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll},
      {sin_yaw * cos_pitch, sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll, sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll},
      {         -sin_pitch,                                cos_pitch * sin_roll,                                cos_pitch * cos_roll}
    };
  // clang-format on
  return R;
}

Eigen::Vector3d xyz2ypd(const Eigen::Vector3d & xyz)
{
  auto x = xyz[0], y = xyz[1], z = xyz[2];
  auto yaw = std::atan2(y, x);
  auto pitch = std::atan2(z, std::sqrt(x * x + y * y));
  auto distance = std::sqrt(x * x + y * y + z * z);
  return {yaw, pitch, distance};
}

/**
 * @brief 计算相机坐标系下3D位置(xyz)到观测值(ypd)的雅可比矩阵
 * @param xyz 相机坐标系下的目标3D位置向量 [x, y, z]^T，单位：米(m)
 *            x：相机向前方向，y：相机向右方向，z：相机向上方向
 * @return J 3×3雅可比矩阵，每行对应一个观测值(ypd)，每列对应一个位置分量(xyz)
 */
Eigen::MatrixXd xyz2ypd_jacobian(const Eigen::Vector3d & xyz)
{
    // 提取3D位置的x、y、z分量（相机坐标系），简化后续计算
    auto x = xyz[0], y = xyz[1], z = xyz[2];

    // ====================== 1. 计算偏航角(yaw)对xyz的偏导数 ======================
    // yaw：相机观测目标的水平偏角（绕z轴旋转，向右为正）
    // 推导依据：yaw = arctan2(y, x)（从x轴（向前）逆时针转到y轴（向右）的角度）
    auto dyaw_dx = -y / (x * x + y * y);  // yaw对x的偏导数：∂(arctan2(y,x))/∂x
    auto dyaw_dy = x / (x * x + y * y);   // yaw对y的偏导数：∂(arctan2(y,x))/∂y
    auto dyaw_dz = 0.0;                   // yaw对z的偏导数：z不影响水平偏角，故为0

    // ====================== 2. 计算俯仰角(pitch)对xyz的偏导数 ======================
    // pitch：相机观测目标的竖直偏角（绕y轴旋转，向上为正）
    // 推导依据：pitch = arctan2(z, √(x²+y²))（从x-y平面（水平）向上转到z轴（竖直）的角度）
    // 分母简化：(z²/(x²+y²) + 1) * (x²+y²)^1.5 = (z² + x² + y²) * √(x²+y²)
    auto dpitch_dx = -(x * z) / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 1.5));  // pitch对x的偏导数
    auto dpitch_dy = -(y * z) / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 1.5));  // pitch对y的偏导数
    auto dpitch_dz = 1 / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 0.5));        // pitch对z的偏导数

    // ====================== 3. 计算距离(distance)对xyz的偏导数 ======================
    // distance：相机到目标的直线距离（欧氏距离）
    // 推导依据：distance = √(x² + y² + z²)
    auto ddistance_dx = x / std::pow((x * x + y * y + z * z), 0.5);  // distance对x的偏导数：∂(√(x²+y²+z²))/∂x
    auto ddistance_dy = y / std::pow((x * x + y * y + z * z), 0.5);  // distance对y的偏导数：∂(√(x²+y²+z²))/∂y
    auto ddistance_dz = z / std::pow((x * x + y * y + z * z), 0.5);  // distance对z的偏导数：∂(√(x²+y²+z²))/∂z

    // ====================== 4. 构建3×3雅可比矩阵 ======================
    // 矩阵结构：
    // [ dyaw_dx   dyaw_dy   dyaw_dz ]  → 第0行：yaw对xyz的偏导数
    // [ dpitch_dx dpitch_dy dpitch_dz ]  → 第1行：pitch对xyz的偏导数
    // [ ddistance_dx ddistance_dy ddistance_dz ]  → 第2行：distance对xyz的偏导数
    // clang-format off
    Eigen::MatrixXd J{
        {dyaw_dx, dyaw_dy, dyaw_dz},
        {dpitch_dx, dpitch_dy, dpitch_dz},
        {ddistance_dx, ddistance_dy, ddistance_dz}
    };
    // clang-format on

    // 返回雅可比矩阵，用于EKF观测更新
    return J;
}

Eigen::Vector3d ypd2xyz(const Eigen::Vector3d & ypd)
{
  auto yaw = ypd[0], pitch = ypd[1], distance = ypd[2];
  auto x = distance * std::cos(pitch) * std::cos(yaw);
  auto y = distance * std::cos(pitch) * std::sin(yaw);
  auto z = distance * std::sin(pitch);
  return {x, y, z};
}

Eigen::MatrixXd ypd2xyz_jacobian(const Eigen::Vector3d & ypd)
{
  auto yaw = ypd[0], pitch = ypd[1], distance = ypd[2];
  double cos_yaw = std::cos(yaw);
  double sin_yaw = std::sin(yaw);
  double cos_pitch = std::cos(pitch);
  double sin_pitch = std::sin(pitch);

  auto dx_dyaw = distance * cos_pitch * -sin_yaw;
  auto dy_dyaw = distance * cos_pitch * cos_yaw;
  auto dz_dyaw = 0.0;

  auto dx_dpitch = distance * -sin_pitch * cos_yaw;
  auto dy_dpitch = distance * -sin_pitch * sin_yaw;
  auto dz_dpitch = distance * cos_pitch;

  auto dx_ddistance = cos_pitch * cos_yaw;
  auto dy_ddistance = cos_pitch * sin_yaw;
  auto dz_ddistance = sin_pitch;

  // clang-format off
  Eigen::MatrixXd J{
    {dx_dyaw, dx_dpitch, dx_ddistance},
    {dy_dyaw, dy_dpitch, dy_ddistance},
    {dz_dyaw, dz_dpitch, dz_ddistance}
  };
  // clang-format on

  return J;
}

double delta_time(
  const std::chrono::steady_clock::time_point & a, const std::chrono::steady_clock::time_point & b)
{
  std::chrono::duration<double> c = a - b;
  return c.count();
}

double get_abs_angle(const Eigen::Vector2d & vec1, const Eigen::Vector2d & vec2)
{
  if (vec1.norm() == 0. || vec2.norm() == 0.) {
    return 0.;
  }
  return std::acos(vec1.dot(vec2) / (vec1.norm() * vec2.norm()));
}

double limit_min_max(double input, double min, double max)
{
  if (input > max)
    return max;
  else if (input < min)
    return min;
  return input;
}
}  // namespace tools