// stage9_fast_tf：fast_tf 模块（强类型坐标变换树）教学 demo。
//
// 坐标系链（复刻 boot.cpp init_coordinate_system）：
//   world -> odom -> gimbal_yaw -> gimbal_pitch -> camera
//
// 5 个 test：链式变换手算对照 / 编译期错误演示 / 时间戳插值 / 跨 ±pi 插值 / 环检测。

#include <cmath>
#include <cstdio>

#include <Eigen/Geometry>

#include "fast_tf/buffer.hpp"
#include "fast_tf/transform.hpp"
#include "fast_tf/validation.hpp"

// ---------------------------------------------------------------------------
// 坐标系类型：一次声明，全项目共享。类型即文档。
// ---------------------------------------------------------------------------
FAST_TF_ROOT_FRAME(world_t);        // 根：世界系
FAST_TF_FRAME(odom_t, world_t);      // 里程计系
FAST_TF_FRAME(gimbal_yaw_t, odom_t);      // 云台 yaw 系
FAST_TF_FRAME(gimbal_pitch_t, gimbal_yaw_t);  // 云台 pitch 系
FAST_TF_FRAME(camera_t, gimbal_pitch_t);       // 相机系

namespace {

using rm::fast_tf::EdgeTransform;
using rm::fast_tf::Vector3;

int g_failures = 0;

void check(bool ok, const char* what) {
  if (ok) {
    std::printf("  [PASS] %s\n", what);
  } else {
    ++g_failures;
    std::printf("  [FAIL] %s\n", what);
  }
}

/// 从边变换提取 yaw（绕 Z），用于插值验证。
double yaw_of(const Eigen::Isometry3d& t) {
  return std::atan2(t.linear()(1, 0), t.linear()(0, 0));
}

// ---------------------------------------------------------------------------
// test 1: 链式变换 —— camera 下的点转 odom，与手算 Eigen 矩阵连乘对照
// ---------------------------------------------------------------------------
void test_chain_transform() {
  std::puts("[test 1] 链式变换（手算对照）");

  // 复刻 boot.cpp：每条边一个 from_rpy / from_translation
  const EdgeTransform<gimbal_yaw_t> t_yaw{rm::fast_tf::from_translation(0.5, 0.0, 0.3), 1.0};
  const EdgeTransform<gimbal_pitch_t> t_pitch{rm::fast_tf::from_rpy(0.0, 0.3, 0.0), 1.0};
  Eigen::Isometry3d t_cam_iso = rm::fast_tf::from_rpy(0.2, 0.1, -0.05);
  t_cam_iso.translation() = Eigen::Vector3d{0.05, 0.0, 0.08};
  const EdgeTransform<camera_t> t_cam{t_cam_iso, 1.0};

  // 类型安全的链式复合：camera -> pitch -> yaw -> odom
  const auto camera_to_odom = t_yaw * t_pitch * t_cam;

  // 被变换的点：camera 系下的 (1, 2, 3)
  const auto p_cam = rm::fast_tf::make_vector3<camera_t>(1.0, 2.0, 3.0);
  const auto p_odom = camera_to_odom * p_cam;

  // 手算：同一个 Eigen 矩阵连乘
  const Eigen::Vector3d hand = t_yaw.tf * t_pitch.tf * t_cam.tf * Eigen::Vector3d{1, 2, 3};

  std::printf("  typed: (%.9f, %.9f, %.9f)\n", p_odom.data.x(), p_odom.data.y(),
              p_odom.data.z());
  std::printf("  hand:  (%.9f, %.9f, %.9f)\n", hand.x(), hand.y(), hand.z());

  const double err = (p_odom.data - hand).norm();
  std::printf("  误差 = %.2e\n", err);
  check(err < 1e-9, "typed 链与手算 Eigen 连乘一致（< 1e-9）");

  // ---------------------------------------------------------------------------
  // 编译期错误演示（放开任何一行都编不过 —— 这正是 fast_tf 的价值）：
  //
  //   const auto bad1 = t_pitch * p_cam;
  //     // error: no operator* ... constraints not satisfied
  //     // [gimbal_pitch_t 的边只能作用于 gimbal_pitch_t 系的点]
  //
  //   EdgeTransform<gimbal_yaw_t> bad2 = t_cam;
  //     // error: 不同边类型不可转换/赋值
  //
  //   const auto bad3 = t_yaw * t_yaw;
  //     // error: operator*(EdgeTransform<A>, EdgeTransform<B>) requires parent(B) == A
  //     // [world 的边后面不能再接 world 的边]
  // ---------------------------------------------------------------------------
  check(true, "编译期错误演示见上方注释（不参与编译）");
}

// ---------------------------------------------------------------------------
// test 2: 时间戳插值 —— 两时刻的 gimbal yaw，查中间时刻
// ---------------------------------------------------------------------------
void test_interpolation() {
  std::puts("[test 2] 时间戳插值");

  auto& yaw_buf = rm::fast_tf::frame_buffer<gimbal_yaw_t>();
  auto& pitch_buf = rm::fast_tf::frame_buffer<gimbal_pitch_t>();
  auto& cam_buf = rm::fast_tf::frame_buffer<camera_t>();
  yaw_buf = {};   // 演示隔离：清空模板单例
  pitch_buf = {};
  cam_buf = {};

  // yaw 边：t=0 时 yaw=0.0，t=1 时 yaw=0.4（rad）
  yaw_buf.insert(EdgeTransform<gimbal_yaw_t>{rm::fast_tf::from_rpy(0, 0, 0.0), 0.0});
  yaw_buf.insert(EdgeTransform<gimbal_yaw_t>{rm::fast_tf::from_rpy(0, 0, 0.4), 1.0});

  // 其余边恒定
  const EdgeTransform<gimbal_pitch_t> pitch{rm::fast_tf::from_rpy(0, 0.3, 0), 0.0};
  const EdgeTransform<camera_t> cam{rm::fast_tf::from_translation(0, 0, 0.05), 0.0};
  pitch_buf.insert(EdgeTransform<gimbal_pitch_t>{pitch.tf, 0.0});
  pitch_buf.insert(EdgeTransform<gimbal_pitch_t>{pitch.tf, 1.0});
  cam_buf.insert(EdgeTransform<camera_t>{cam.tf, 0.0});
  cam_buf.insert(EdgeTransform<camera_t>{cam.tf, 1.0});

  // 查中间时刻：线性插值 yaw = (0.0 + 0.4) / 2 = 0.2
  auto yaw_mid = yaw_buf.lookup(0.5);
  check(yaw_mid.has_value(), "lookup(0.5) 成功");
  const double got = yaw_of(yaw_mid->tf);
  std::printf("  插值 yaw = %.9f, 期望 0.2\n", got);
  check(std::abs(got - 0.2) < 1e-9, "yaw 插值 == 两端均值");

  // 整链插值查询：camera 点在 t=0.5 转到 odom
  const auto e_yaw = *yaw_buf.lookup(0.5);
  const auto e_pitch = *pitch_buf.lookup(0.5);
  const auto e_cam = *cam_buf.lookup(0.5);
  const auto chain = e_yaw * e_pitch * e_cam;
  const auto p = chain * rm::fast_tf::make_vector3<camera_t>(0.1, 0.2, 0.3);

  // 手算：直接用 t=0.5 的 yaw=0.2 构造
  const Eigen::Isometry3d hand =
      rm::fast_tf::from_rpy(0, 0, 0.2) * pitch.tf * cam.tf;
  const Eigen::Vector3d expect = hand * Eigen::Vector3d{0.1, 0.2, 0.3};

  const double err = (p.data - expect).norm();
  std::printf("  链式插值误差 = %.2e\n", err);
  check(err < 1e-9, "整链插值与手算一致");

  // 边界：早于最老样本 -> 错误；晚于最新 -> clamp
  check(!yaw_buf.lookup(-1.0).has_value(), "早于最老样本返回错误");
  const auto future = yaw_buf.lookup(99.0);
  check(future.has_value() && std::abs(yaw_of(future->tf) - 0.4) < 1e-12,
        "晚于最新样本 clamp 到最新值");
}

// ---------------------------------------------------------------------------
// test 3: 跨 ±pi 的旋转插值（slerp 走最短路径）
// ---------------------------------------------------------------------------
void test_wraparound_interpolation() {
  std::puts("[test 3] 跨 ±pi 旋转插值");

  rm::fast_tf::TfBuffer<gimbal_yaw_t> buf;
  buf.insert(EdgeTransform<gimbal_yaw_t>{rm::fast_tf::from_rpy(0, 0, M_PI - 0.1), 0.0});
  buf.insert(EdgeTransform<gimbal_yaw_t>{rm::fast_tf::from_rpy(0, 0, -M_PI + 0.1), 1.0});

  const auto mid = buf.lookup(0.5);
  check(mid.has_value(), "lookup(0.5) 成功");
  const double yaw = yaw_of(mid->tf);
  // 两端在流形上相距 0.2 rad，中点应落在 |yaw| = pi 附近
  const double wrapped = std::remainder(yaw, 2.0 * M_PI);
  std::printf("  mid yaw = %.9f（等效 ±pi = %.9f）\n", yaw, wrapped);
  check(std::abs(std::abs(wrapped) - M_PI) < 1e-9, "slerp 最短路径：中点 = ±pi");
}

// ---------------------------------------------------------------------------
// test 4: 环检测 —— 人为造 A -> B -> A，validation 必须报错
// ---------------------------------------------------------------------------
void test_cycle_detection() {
  std::puts("[test 4] 环检测");

  // 先确认当前树合法
  check(!rm::fast_tf::validate().has_value(), "正常树 validate 通过");

  // 注入假边制造环：fake_a -> fake_b -> fake_a
  rm::fast_tf::detail::register_edge("fake_a", "fake_b");
  rm::fast_tf::detail::register_edge("fake_b", "fake_a");

  const auto err = rm::fast_tf::validate();
  check(err.has_value(), "存在环时 validate 报错");
  if (err) {
    std::printf("  错误信息: %s\n", err->c_str());
    check(err->find("cycle") != std::string::npos && err->find("fake_a") != std::string::npos,
          "错误信息含 cycle 与环节点名");
  }

  // 清理：恢复合法
  rm::fast_tf::detail::unregister_edge("fake_a");
  rm::fast_tf::detail::unregister_edge("fake_b");
  check(!rm::fast_tf::validate().has_value(), "移除假边后恢复合法");
}

// ---------------------------------------------------------------------------
// test 5: parent 链不存在的帧 -> validate 报"unknown parent"
// ---------------------------------------------------------------------------
void test_unknown_parent() {
  std::puts("[test 5] 未知父帧");

  rm::fast_tf::detail::register_edge("orphan", "ghost");
  const auto err = rm::fast_tf::validate();
  check(err.has_value(), "指向不存在父帧时 validate 报错");
  if (err) {
    std::printf("  错误信息: %s\n", err->c_str());
    check(err->find("ghost") != std::string::npos, "错误信息带未知父帧名");
  }
  rm::fast_tf::detail::unregister_edge("orphan");
  check(!rm::fast_tf::validate().has_value(), "清理后恢复合法");
}

}  // namespace

int main() {
  std::puts("=== stage9_fast_tf ===");
  test_chain_transform();
  test_interpolation();
  test_wraparound_interpolation();
  test_cycle_detection();
  test_unknown_parent();
  std::printf("\n%s\n", g_failures == 0 ? "ALL TESTS PASSED" : "SOME TESTS FAILED");
  return g_failures == 0 ? 0 : 1;
}
