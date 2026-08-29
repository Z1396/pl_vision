#pragma once
// EdgeTransform：强类型"边变换"与带坐标系标签的点/向量。
//
// 约定（与 TF2 一致）：EdgeTransform<Child> 把"Child 系下的坐标"变换到
// "parent(Child) 系下"（child -> parent）。整条链 world -> ... -> camera
// 上每条边都是一次 from_rpy / from_translation 的产物（复刻 boot.cpp
// init_coordinate_system 的用法）。
//
// 类型安全全部来自 operator* 的 requires 约束：
//   EdgeTransform<A> * EdgeTransform<B>   要求 parent(B) == A（B 的父恰是 A）
//   EdgeTransform<A> * Vector3<F>        要求 F == A（点必须长在 A 系上）
// 把 camera 系的向量喂给 gimbal 系的函数 → 直接编译报错，不是运行期 bug。
//
// 真实项目中位于 crates/fast_tf/src/（约 2,713 行，本 stage 是教学核心）。

#include <Eigen/Geometry>
#include <concepts>

#include "fast_tf/frame.hpp"

namespace rm::fast_tf {

/// 边变换：Child -> parent(Child)。
template <class Child>
struct EdgeTransform {
  using child_frame = Child;
  using parent_frame = typename Child::parent_frame;

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  double stamp{};  // 该边样本的时间戳
};

/// 长在 F 系上的三维点（不带时间戳；带时间戳的查询走 TfBuffer::lookup）。
template <class F>
struct Vector3 {
  Eigen::Vector3d data = Eigen::Vector3d::Zero();
};

template <class F>
Vector3<F> make_vector3(double x, double y, double z) {
  return Vector3<F>{Eigen::Vector3d{x, y, z}};
}

/// 链式复合：a: A->parent(A)，b: B->A。结果：B->parent(A)。
/// 数学上 tf = tf_a * tf_b（先作用 b，再作用 a）。
template <class A, class B>
  requires std::same_as<typename B::parent_frame, A>
EdgeTransform<B> operator*(const EdgeTransform<A>& a, const EdgeTransform<B>& b) {
  return EdgeTransform<B>{a.tf * b.tf, a.stamp};
}

/// 边作用于点：A 系下的点 -> parent(A) 系。
template <class A, class F>
  requires std::same_as<A, F>
Vector3<typename A::parent_frame> operator*(const EdgeTransform<A>& e, const Vector3<F>& v) {
  return Vector3<typename A::parent_frame>{e.tf * v.data};
}

// ---------------------------------------------------------------------------
// 构造工具（boot.cpp init_coordinate_system 同款）
// ---------------------------------------------------------------------------

inline Eigen::Isometry3d from_translation(double x, double y, double z) {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d{x, y, z};
  return t;
}

/// RPY -> 旋转（ZYX 内旋：R = Rz(yaw) * Ry(pitch) * Rx(roll)）。
inline Eigen::Isometry3d from_rpy(double roll, double pitch, double yaw) {
  const Eigen::AngleAxisd r{roll, Eigen::Vector3d::UnitX()};
  const Eigen::AngleAxisd p{pitch, Eigen::Vector3d::UnitY()};
  const Eigen::AngleAxisd y{yaw, Eigen::Vector3d::UnitZ()};
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = (y * p * r).toRotationMatrix();
  return t;
}

}  // namespace rm::fast_tf
