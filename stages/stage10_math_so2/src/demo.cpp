// stage10_math_so2：math 模块（SO(2) 李群 + 欧拉角）教学 demo。
//
// 核心命题：为什么角度必须用 SO2 而不是裸 double ——
//   流形上 350° 与 -10° 是同一个角，350° - 10° 的真实增量是 -20°；
//   裸 double 减法给出 +340°，EKF 观测残差直接错一个周期。

#include <cmath>
#include <cstdio>

#include "math/euler.hpp"
#include "math/so2.hpp"

using math::SO2;

namespace {

int g_failures = 0;

void check(bool ok, const char* what) {
  if (ok) {
    std::printf("  [PASS] %s\n", what);
  } else {
    ++g_failures;
    std::printf("  [FAIL] %s\n", what);
  }
}

// ---------------------------------------------------------------------------
// test 1: 归一化 —— 350° - 10° 必须是 -20°，而不是 +340°
// ---------------------------------------------------------------------------
void test_normalize_cross_boundary() {
  std::puts("[test 1] SO2 跨 ±pi 边界减法");

  const SO2 a = math::from_degrees(350.0);
  const SO2 b = math::from_degrees(10.0);
  const SO2 delta = a - b;

  const double raw = 350.0 - 10.0;  // 裸 double：+340°

  std::printf("  SO2:  %.3f rad (%.1f deg)\n", delta.angle, delta.angle * 180.0 / math::kPi);
  std::printf("  raw:  %.3f deg\n", raw);

  check(std::abs(delta.angle - math::from_degrees(-20.0).angle) < 1e-9,
        "delta == -20 deg (而非 +340 deg)");
  check(std::abs(delta.angle) <= math::kPi + 1e-12, "结果落在 [-pi, pi]");
}

// ---------------------------------------------------------------------------
// test 2: 增量叠加 —— (pi - 0.1) + 0.2 回绕成 -(pi - 0.1)
// ---------------------------------------------------------------------------
void test_delta_accumulation() {
  std::puts("[test 2] 增量叠加跨界回绕");

  const SO2 base = SO2(math::kPi - 0.1);
  const SO2 rotated = base + SO2(0.2);  // pi + 0.1 -> -(pi - 0.1)

  std::printf("  (pi - 0.1) + 0.2 = %.6f, 期望 -(pi - 0.1) = %.6f\n", rotated.angle,
              -(math::kPi - 0.1));

  check(std::abs(rotated.angle - (-(math::kPi - 0.1))) < 1e-9, "pi+0.1 回绕为 -(pi-0.1)");
  check(rotated.inverse().angle == -rotated.angle, "inverse = -angle");
}

// ---------------------------------------------------------------------------
// test 3: euler 与 SO2 混用 —— RPY 旋转矩阵的 yaw 平面与 SO2 一致
// ---------------------------------------------------------------------------
void test_euler_so2_interop() {
  std::puts("[test 3] euler 与 SO2 混用");

  // 纯 yaw 时，R(rpy) 必须精确等于 SO2(yaw) 的旋转矩阵
  const double yaw = math::from_degrees(170.0).angle;
  const math::Matrix3 rz = math::from_rpy({0.0, 0.0, yaw});
  const auto r2 = SO2(yaw).rotation_matrix();
  check(std::abs(rz[0] - r2[0]) < 1e-12 && std::abs(rz[1] - r2[1]) < 1e-12 &&
            std::abs(rz[3] - r2[2]) < 1e-12 && std::abs(rz[4] - r2[3]) < 1e-12,
        "R(0,0,yaw) == R(SO2(yaw))");

  // 一般姿态下矩阵 <-> RPY 往返
  const math::EulerAngles rpy{0.1, -0.2, yaw};
  const math::Matrix3 r = math::from_rpy(rpy);
  const math::EulerAngles back = math::to_rpy(r);
  const math::Matrix3 r_back = math::from_rpy(back);
  double max_err = 0.0;
  for (size_t i = 0; i < 9; ++i) {
    max_err = std::max(max_err, std::abs(r[i] - r_back[i]));
  }
  std::printf("  rpy 往返最大误差 = %.2e\n", max_err);
  check(max_err < 1e-12, "to_rpy(from_rpy(x)) == x");
}

// ---------------------------------------------------------------------------
// test 4: 反例对照 —— 裸 double 在跨界的观测残差直接错一个周期
// ---------------------------------------------------------------------------
void test_raw_double_counterexample() {
  std::puts("[test 4] 裸 double 反例（观测残差视角）");

  const double target_yaw = 350.0 * math::kPi / 180.0;   // 目标真实 yaw
  const double measured_yaw = 10.0 * math::kPi / 180.0;  // 量测（等价于 -350°）

  const double raw_residual = target_yaw - measured_yaw;         // 错：+340°
  const SO2 so2_residual = SO2(target_yaw) - SO2(measured_yaw);  // 对：-20°

  std::printf("  raw residual: %+8.3f deg   <- EKF 会把 0.4 rad 的残差放大成 6 rad\n",
              raw_residual * 180.0 / math::kPi);
  std::printf("  SO2 residual: %+8.3f deg   <- 真实的最短角差\n",
              so2_residual.angle * 180.0 / math::kPi);

  check(std::abs(raw_residual - so2_residual.angle - 2.0 * math::kPi) < 1e-9,
        "raw 与 SO2 相差整整一个周期");
}

}  // namespace

int main() {
  std::puts("=== stage10_math_so2 ===");
  test_normalize_cross_boundary();
  test_delta_accumulation();
  test_euler_so2_interop();
  test_raw_double_counterexample();
  std::printf("\n%s\n", g_failures == 0 ? "ALL TESTS PASSED" : "SOME TESTS FAILED");
  return g_failures == 0 ? 0 : 1;
}
