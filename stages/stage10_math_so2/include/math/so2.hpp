#pragma once
// SO(2) 旋转李群：用类型系统保证角度永远归一化在 [-pi, pi]。
//
// 为什么不用裸 double 存角度？
//   a - b 在跨 ±pi 边界时会得到 ~2*pi 的错误增量：
//     350° - 10° 裸减法 = +340°，而流形上的真实增量是 -20°。
//   SO2 的减法结果天然落在 [-pi, pi]，这是跟踪器 yaw 解算不跳变的基础。
//
// 真实项目中位于 crates/math/src/so2.hpp，demo 直接 include 即可。

#include <array>
#include <cmath>

namespace math {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kTwoPi = 2.0 * kPi;

/// 二维旋转群 SO(2) 的元素。angle 构造后必然落在 [-pi, pi]。
struct SO2 {
  double angle{};  // 弧度，[-pi, pi]

  SO2() = default;

  /// 任意角度构造，内部用 std::remainder 归一化到 [-pi, pi]。
  explicit SO2(double theta) : angle{normalize(theta)} {}

  /// std::remainder(x, 2pi) 的结果域恰为 [-pi, pi]，且取最近整周期。
  static double normalize(double theta) { return std::remainder(theta, kTwoPi); }

  /// 流形增量：Delta = this - rhs，结果直接是归一化的角差。
  SO2 operator-(const SO2& rhs) const { return SO2(angle - rhs.angle); }

  /// 叠加增量：this + delta，跨界自动回绕。
  SO2 operator+(const SO2& delta) const { return SO2(angle + delta.angle); }

  /// 旋转复合（与 + 同语义，保留群乘法记号）。
  SO2 operator*(const SO2& rhs) const { return SO2(angle + rhs.angle); }

  /// 群逆：旋转 -angle。
  SO2 inverse() const { return SO2(-angle); }

  double cos() const { return std::cos(angle); }
  double sin() const { return std::sin(angle); }

  /// 2x2 旋转矩阵，行主序 {r00, r01, r10, r11}。
  std::array<double, 4> rotation_matrix() const {
    const double c = cos(), s = sin();
    return {c, -s, s, c};
  }
};

/// 角度制工厂。
inline SO2 from_degrees(double deg) { return SO2(deg * kPi / 180.0); }

}  // namespace math
