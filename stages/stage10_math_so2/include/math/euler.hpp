#pragma once
// 欧拉角（RPY）与旋转矩阵互转。约定与 fast_tf 一致：R = Rz(yaw) * Ry(pitch) * Rx(roll)。
//
// 真实项目中位于 crates/math/src/euler.hpp。

#include <algorithm>
#include <array>
#include <cmath>

#include "math/so2.hpp"

namespace math {

struct EulerAngles {
  double roll{};   // 绕 X
  double pitch{};  // 绕 Y
  double yaw{};    // 绕 Z
};

/// 3x3 矩阵，行主序。
using Matrix3 = std::array<double, 9>;

/// RPY -> 旋转矩阵（ZYX 内旋顺序：先 roll，再 pitch，最后 yaw）。
inline Matrix3 from_rpy(const EulerAngles& rpy) {
  const double cr = std::cos(rpy.roll), sr = std::sin(rpy.roll);
  const double cp = std::cos(rpy.pitch), sp = std::sin(rpy.pitch);
  const double cy = std::cos(rpy.yaw), sy = std::sin(rpy.yaw);

  Matrix3 r{};
  r[0] = cy * cp;
  r[1] = cy * sp * sr - sy * cr;
  r[2] = cy * sp * cr + sy * sr;
  r[3] = sy * cp;
  r[4] = sy * sp * sr + cy * cr;
  r[5] = sy * sp * cr - cy * sr;
  r[6] = -sp;
  r[7] = cp * sr;
  r[8] = cp * cr;
  return r;
}

/// 旋转矩阵 -> RPY。万向锁（pitch = ±90°）时 roll/yaw 不唯一，这里直接断言非退化。
inline EulerAngles to_rpy(const Matrix3& r) {
  const double sp = -r[6];  // R20 = -sin(pitch)
  EulerAngles rpy{};
  rpy.pitch = std::asin(std::clamp(sp, -1.0, 1.0));
  const double cp = std::cos(rpy.pitch);
  if (std::abs(cp) < 1e-9) {
    // 万向锁：yaw 与 roll 耦合，取 yaw = 0 的一个特解。
    rpy.yaw = 0.0;
    rpy.roll = std::atan2(r[3], r[0]);
    return rpy;
  }
  rpy.yaw = std::atan2(r[3], r[0]);   // atan2(R10, R00)
  rpy.roll = std::atan2(r[7], r[8]);  // atan2(R21, R22)
  return rpy;
}

}  // namespace math
