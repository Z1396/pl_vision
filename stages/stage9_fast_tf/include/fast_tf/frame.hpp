#pragma once
// fast_tf：强类型坐标变换树 —— 每个坐标系是独立的 C++ 类型。
//
// frame.hpp 定义坐标系类型层级：父子链在编译期由类型表达：
//   FAST_TF_ROOT_FRAME(world_t)              // 根坐标系
//   FAST_TF_FRAME(odom_t, world_t)           // odom 的父是 world
//   FAST_TF_FRAME(gimbal_yaw_t, odom_t)      // ...
// 类型本身没有数据，只携带"我的父坐标系是谁"这一编译期信息。
//
// 宏同时把 (名字, 父名字) 注册进运行期注册表，供 validation.hpp 做环检测。
//
// 真实项目中位于 crates/fast_tf/src/frame.hpp（DECL 宏）。

#include <map>
#include <mutex>
#include <string>
#include <type_traits>

namespace rm::fast_tf {

/// 根坐标系标记（无父）。
struct world_root {};

/// 帧类型基类：唯一的编译期信息就是 parent_frame。
template <class Parent>
struct frame {
  using parent_frame = Parent;
};

/// 判断 F 是否有父坐标系。
template <class F>
struct has_parent : std::false_type {};
template <class P>
struct has_parent<frame<P>> : std::true_type {};
template <class F>
inline constexpr bool has_parent_v = has_parent<F>::value;

namespace detail {

/// 运行期注册表：child 名 -> parent 名（"" 表示根）。环检测用。
inline std::map<std::string, std::string>& registry() {
  static std::map<std::string, std::string> reg;
  return reg;
}

inline std::mutex& registry_mutex() {
  static std::mutex m;
  return m;
}

inline bool register_edge(const std::string& child, const std::string& parent) {
  std::scoped_lock lock{registry_mutex()};
  registry()[child] = parent;
  return true;
}

inline void unregister_edge(const std::string& child) {
  std::scoped_lock lock{registry_mutex()};
  registry().erase(child);
}

}  // namespace detail

/// 声明根坐标系（链的终点，如世界系）。
#define FAST_TF_ROOT_FRAME(name)                                                       \
  struct name {};                                                                       \
  [[maybe_unused]] inline const bool registered_##name =                               \
      ::rm::fast_tf::detail::register_edge(#name, "");

/// 声明子坐标系：parent 必须已声明。
#define FAST_TF_FRAME(name, parent)                                                     \
  struct name : ::rm::fast_tf::frame<parent> {};                                        \
  [[maybe_unused]] inline const bool registered_##name =                                \
      ::rm::fast_tf::detail::register_edge(#name, #parent);

}  // namespace rm::fast_tf
