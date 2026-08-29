#pragma once
// toml 反射式解析：定义 struct 后自动从 TOML 表填充，不写解析代码。
//
// 四种字段语义（由字段类型自动判定）：
//   普通 T            —— 缺省时保留 struct 内默认值；
//   required<T>       —— 必填，缺失即失败（fail-fast，对应项目"配置缺字段直接退出"）；
//   std::optional<T>  —— 缺省 = nullopt，与显式赋值可区分；
//   flatten<T>        —— serde 扁平化：T 的字段来自同一张表。
// 容器：std::vector<T>（TOML array）；嵌套 struct 走子表递归。
//
// 错误信息带字段路径（"missing required field 'id'"），expected<T, string> 返回。
// 真实项目中位于 crates/toml/src/（约 3,960 行，含 Eigen 扩展，本 stage 是其教学核心）。

#include <expected>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <vector>

#include "tomlplusplus/toml.hpp"

namespace rm::toml_reflect {

// ---------------------------------------------------------------------------
// 字段语义包装
// ---------------------------------------------------------------------------

template <class T>
struct required {
  T value{};
};

template <class T>
struct flatten {
  T value{};
};

// ---------------------------------------------------------------------------
// 字段描述符与 Traits
// ---------------------------------------------------------------------------

template <class S, class M>
struct Field {
  std::string_view name;
  M S::*ptr;
};
template <class S, class M>
Field(std::string_view, M S::*) -> Field<S, M>;

/// 每个 struct 用 TOML_REFLECT 宏特化；未特化的 struct 无法参与 from_table。
template <class T>
struct Traits;  // 故意不给主模板定义

template <class T>
concept Reflectable = requires { Traits<T>::fields; };

// ---------------------------------------------------------------------------
// 类型萃取
// ---------------------------------------------------------------------------

template <class T>
struct is_required : std::false_type {};
template <class T>
struct is_required<required<T>> : std::true_type {};
template <class T>
inline constexpr bool is_required_v = is_required<T>::value;

template <class T>
struct is_flatten : std::false_type {};
template <class T>
struct is_flatten<flatten<T>> : std::true_type {};
template <class T>
inline constexpr bool is_flatten_v = is_flatten<T>::value;

template <class T>
struct is_optional : std::false_type {};
template <class T>
struct is_optional<std::optional<T>> : std::true_type {};
template <class T>
inline constexpr bool is_optional_v = is_optional<T>::value;

template <class T>
struct is_vector : std::false_type {};
template <class T, class A>
struct is_vector<std::vector<T, A>> : std::true_type {};
template <class T>
inline constexpr bool is_vector_v = is_vector<T>::value;

template <class T>
struct inner_of;
template <class T>
struct inner_of<required<T>> {
  using type = T;
};
template <class T>
struct inner_of<flatten<T>> {
  using type = T;
};
template <class T>
using inner_t = typename inner_of<T>::type;

// ---------------------------------------------------------------------------
// 解析核心
// ---------------------------------------------------------------------------

template <Reflectable T>
std::expected<T, std::string> from_table(const toml::table& tbl);

/// 数组元素版本（前置声明：数组元素是裸节点，不是"表里的键值对"）。
template <class M>
std::optional<std::string> parse_into(const toml::node& node, M& member);

/// 把 tbl 中名为 name 的字段解析进 member。返回 nullopt = 成功。
template <class M>
std::optional<std::string> parse_into(const toml::table& tbl, std::string_view name, M& member) {
  // ---- required<T>：缺失即失败 ----
  if constexpr (is_required_v<M>) {
    if (tbl.get(name) == nullptr) {
      return std::string{"missing required field '"} + std::string{name} + "'";
    }
    return parse_into(tbl, name, member.value);
  }
  // ---- flatten<T>：字段来自同一张表（serde 扁平化） ----
  else if constexpr (is_flatten_v<M>) {
    auto res = from_table<inner_t<M>>(tbl);
    if (!res) {
      return std::string{"flatten '"} + std::string{name} + "': " + res.error();
    }
    member.value = std::move(*res);
    return std::nullopt;
  }
  // ---- optional<T>：缺省 = nullopt ----
  else if constexpr (is_optional_v<M>) {
    const auto* node = tbl.get(name);
    if (node == nullptr) {
      member = std::nullopt;
      return std::nullopt;
    }
    return parse_into(tbl, name, member.emplace());
  }
  // ---- vector<T>：TOML array ----
  else if constexpr (is_vector_v<M>) {
    const auto* node = tbl.get(name);
    if (node == nullptr) {
      return std::nullopt;  // 缺省保留默认（通常为空）
    }
    const auto* arr = node->as_array();
    if (arr == nullptr) {
      return std::string{"field '"} + std::string{name} + "' should be an array";
    }
    member.clear();
    std::size_t i = 0;
    for (auto&& elem_node : *arr) {
      auto& elem = member.emplace_back();
      auto err = parse_into(elem_node, elem);  // 数组元素走"裸节点"重载
      if (err) {
        return std::string{"field '"} + std::string{name} + "'[" + std::to_string(i) +
               "]: " + *err;
      }
      ++i;
    }
    return std::nullopt;
  }
  // ---- 嵌套 struct：子表递归 ----
  else if constexpr (Reflectable<M>) {
    const auto* node = tbl.get(name);
    if (node == nullptr) {
      return std::nullopt;  // 子表整体缺省：保留 struct 默认值
    }
    const auto* sub = node->as_table();
    if (sub == nullptr) {
      return std::string{"field '"} + std::string{name} + "' should be a table";
    }
    auto res = from_table<M>(*sub);
    if (!res) {
      return std::string{"table '"} + std::string{name} + "': " + res.error();
    }
    member = std::move(*res);
    return std::nullopt;
  }
  // ---- 标量 ----
  else {
    const auto* node = tbl.get(name);
    if (node == nullptr) {
      return std::nullopt;  // 缺省保留默认值
    }
    auto v = node->value<M>();
    if (!v) {
      return std::string{"field '"} + std::string{name} + "': type mismatch";
    }
    member = *v;
    return std::nullopt;
  }
}

/// 数组元素版本：元素不是"表中的键值对"，而是裸节点。
template <class M>
std::optional<std::string> parse_into(const toml::node& node, M& member) {
  if constexpr (is_required_v<M> || is_flatten_v<M>) {
    return std::string{"array element cannot be required/flatten"};
  } else if constexpr (is_optional_v<M>) {
    member = std::nullopt;
    return std::nullopt;
  } else if constexpr (is_vector_v<M>) {
    return std::string{"nested vector not supported"};
  } else if constexpr (Reflectable<M>) {
    return std::string{"array of struct not supported"};
  } else {
    auto v = node.value<M>();
    if (!v) {
      return std::string{"type mismatch"};
    }
    member = *v;
    return std::nullopt;
  }
}

template <Reflectable T>
std::expected<T, std::string> from_table(const toml::table& tbl) {
  T obj{};  // 先落默认值，再逐字段覆盖
  std::optional<std::string> first_err;
  std::apply(
      [&](auto const&... fs) {
        auto parse_one = [&](auto const& f) {
          if (first_err == std::nullopt) {
            first_err = parse_into(tbl, f.name, obj.*(f.ptr));
          }
        };
        (parse_one(fs), ...);
      },
      Traits<T>::fields);
  if (first_err) {
    return std::unexpected(*first_err);
  }
  return obj;
}

/// 便捷入口：TOML 文本 -> T。解析异常也转成 expected 错误。
template <Reflectable T>
std::expected<T, std::string> parse(std::string_view text) {
  try {
    toml::parse_result result = toml::parse(text);
    return from_table<T>(result);
  } catch (const toml::parse_error& e) {
    return std::unexpected(std::string{"toml syntax error: "} + e.what());
  }
}

}  // namespace rm::toml_reflect

// ---------------------------------------------------------------------------
// TOML_REFLECT 宏：为 struct 注册字段列表（名字 + 成员指针）。
// 用法：TOML_REFLECT(Intrinsics, fx, fy, cx, cy)
// ---------------------------------------------------------------------------

#define TOML_FIELD_ONE(s, m) ::rm::toml_reflect::Field{#m, &s::m}

#define TOML_FE_1(m, s, x) m(s, x)
#define TOML_FE_2(m, s, x, ...) m(s, x), TOML_FE_1(m, s, __VA_ARGS__)
#define TOML_FE_3(m, s, x, ...) m(s, x), TOML_FE_2(m, s, __VA_ARGS__)
#define TOML_FE_4(m, s, x, ...) m(s, x), TOML_FE_3(m, s, __VA_ARGS__)
#define TOML_FE_5(m, s, x, ...) m(s, x), TOML_FE_4(m, s, __VA_ARGS__)
#define TOML_FE_6(m, s, x, ...) m(s, x), TOML_FE_5(m, s, __VA_ARGS__)
#define TOML_FE_7(m, s, x, ...) m(s, x), TOML_FE_6(m, s, __VA_ARGS__)
#define TOML_FE_8(m, s, x, ...) m(s, x), TOML_FE_7(m, s, __VA_ARGS__)
#define TOML_FE_9(m, s, x, ...) m(s, x), TOML_FE_8(m, s, __VA_ARGS__)
#define TOML_FE_10(m, s, x, ...) m(s, x), TOML_FE_9(m, s, __VA_ARGS__)
#define TOML_FE_11(m, s, x, ...) m(s, x), TOML_FE_10(m, s, __VA_ARGS__)
#define TOML_FE_12(m, s, x, ...) m(s, x), TOML_FE_11(m, s, __VA_ARGS__)

#define TOML_FE_GET(_1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, NAME, ...) NAME
#define TOML_FOR_EACH(m, s, ...)\
  TOML_FE_GET(__VA_ARGS__, TOML_FE_12, TOML_FE_11, TOML_FE_10, TOML_FE_9, TOML_FE_8,\
              TOML_FE_7, TOML_FE_6, TOML_FE_5, TOML_FE_4, TOML_FE_3, TOML_FE_2,\
              TOML_FE_1)(m, s, __VA_ARGS__)

/// 注意：至少要列 1 个字段（空 struct 请手工特化 Traits）。
#define TOML_REFLECT(Struct, ...)\
  template <>\
  struct ::rm::toml_reflect::Traits<Struct> {\
    static constexpr auto fields = std::tuple{TOML_FOR_EACH(TOML_FIELD_ONE, Struct,\
                                                            __VA_ARGS__)};\
  }
