#pragma once
// lazy：延迟构造。构造时只保存参数包（std::tuple），第一次 get() 才真正 new T。
//
// 价值：依赖在构造顺序未定的场景里先声明、后实例化；构造有副作用的重资源
// （相机句柄、CUDA context）可以推迟到第一次使用，且保证恰好构造一次。
//
// 两条路径（requires 约束区分）：
//   1. 预绑定：Lazy<Foo>(1, 2) —— 参数在构造期打包进 std::tuple，
//      get() 时 std::apply 解包 new T(args...)；
//   2. 运行时参数：Lazy<Foo> 默认构造，get(x, y) 传参构造
//      （要求此前没有预绑定参数）。
//
// 真实项目中位于 crates/primitive/src/lazy.hpp。

#include <functional>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <utility>

namespace rm::primitive {

template <class T>
class Lazy {
  /// 类型擦除的"参数包盒子"：唯一虚函数 construct() 里做 std::apply 解包。
  struct BoundBase {
    virtual ~BoundBase() = default;
    virtual std::unique_ptr<T> construct() = 0;
  };

  template <class... Args>
  struct Bound final : BoundBase {
    std::tuple<std::decay_t<Args>...> stored;

    explicit Bound(Args&&... args) : stored(std::forward<Args>(args)...) {}

    std::unique_ptr<T> construct() override {
      return std::apply(
          [](auto&&... a) { return std::make_unique<T>(std::forward<decltype(a)>(a)...); },
          std::move(stored));
    }
  };

 public:
  Lazy() = default;

  /// 预绑定构造参数（sizeof... > 0，与默认构造区分开）。
  template <class... Args>
    requires(sizeof...(Args) > 0)
  Lazy(Args&&... args)
      : bound_(std::make_unique<Bound<Args...>>(std::forward<Args>(args)...)) {}

  Lazy(const Lazy&) = delete;
  Lazy& operator=(const Lazy&) = delete;
  Lazy(Lazy&&) = default;
  Lazy& operator=(Lazy&&) = default;

  /// 路径 1：预绑定参数解包构造，或默认构造。重复调用返回同一实例。
  T* get() {
    if (ptr_ == nullptr) {
      if (bound_ != nullptr) {
        ptr_ = bound_->construct();
      } else if constexpr (std::is_default_constructible_v<T>) {
        ptr_ = std::make_unique<T>();
      } else {
        throw std::logic_error("Lazy<T>: no prebound args and T is not default constructible");
      }
    }
    return ptr_.get();
  }

  /// 路径 2：运行时传参构造（仅无预绑定参数时可用；重复调用返回同一实例）。
  template <class... Args>
    requires(sizeof...(Args) > 0)
  T* get(Args&&... args) {
    if (bound_ != nullptr) {
      throw std::logic_error("Lazy<T>: runtime args conflict with prebound args");
    }
    if (ptr_ == nullptr) {
      ptr_ = std::make_unique<T>(std::forward<Args>(args)...);
    }
    return ptr_.get();
  }

  const T* get() const { return ptr_.get(); }
  bool is_constructed() const { return ptr_ != nullptr; }
  explicit operator bool() const { return is_constructed(); }

 private:
  std::unique_ptr<BoundBase> bound_;  // nullptr = 无预绑定参数
  std::unique_ptr<T> ptr_;
};

}  // namespace rm::primitive
