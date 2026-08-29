#pragma once
// overloaded：让一组 lambda 成为 std::visit 的多态 visitor（C++17 pack expansion 继承）。
//
// 原理：每个 lambda 都是一个有唯一 operator() 的类；让 visitor 同时继承所有 lambda，
// 再用 using 把每个基类的 operator() 引入派生类作用域 —— std::visit 就能按
// variant 当前持有的备选精确匹配到对应 lambda。
// （对应真实项目 output_interface 的三策略分发。）

namespace rm::primitive {

template <class... Ts>
struct overloaded : Ts... {
  using Ts::operator()...;
};

// C++17 CTAD 推导指引：overloaded{f, g, h} 直接聚合初始化。
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

}  // namespace rm::primitive
