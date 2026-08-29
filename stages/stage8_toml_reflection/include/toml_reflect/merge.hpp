#pragma once
// merge_configs：配置分层合并（base <- overlay）。
//
// 规则（serde merge 同款）：
//   - overlay 中不存在的 key：保留 base；
//   - 两侧同 key 且都是 table：递归深合并；
//   - 其它情况（含类型不同）：overlay 覆盖。
//
// 典型用途：机器人默认配置 <- 赛场 override 文件。
// 真实项目中位于 crates/toml/src/ 下。

#include <string>
#include <type_traits>

#include "toml_reflect/reflect.hpp"

namespace rm::toml_reflect {

namespace detail {

/// 把 src 节点按具体类型拷进 dst 表（toml++ 节点是多态的，需 visit 分发后拷贝）。
inline void copy_node(toml::table& dst, std::string_view key, const toml::node& src) {
  src.visit([&](auto&& concrete) {
    using V = std::decay_t<decltype(concrete)>;
    if constexpr (std::is_same_v<V, toml::table>) {
      toml::table sub;
      for (auto&& [k, v] : concrete) {
        copy_node(sub, k, v);
      }
      dst.insert_or_assign(key, std::move(sub));
    } else if constexpr (std::is_same_v<V, toml::array>) {
      toml::array arr;
      for (auto&& v : concrete) {
        v.visit([&](auto&& c2) {
          using W = std::decay_t<decltype(c2)>;
          if constexpr (std::is_same_v<W, toml::table>) {
            toml::table elem;
            for (auto&& [k2, v2] : c2) {
              copy_node(elem, k2, v2);
            }
            arr.push_back(std::move(elem));
          } else {
            arr.push_back(c2);  // 标量 / 嵌套数组：直接拷贝
          }
        });
      }
      dst.insert_or_assign(key, std::move(arr));
    } else {
      // 标量（integer/float/boolean/string/date...）
      dst.insert_or_assign(key, concrete);
    }
  });
}

}  // namespace detail

inline toml::table merge_configs(const toml::table& base, const toml::table& overlay) {
  toml::table out;

  // 1) base 全量拷入
  for (auto&& [k, v] : base) {
    detail::copy_node(out, k, v);
  }

  // 2) overlay 覆盖：双侧都是 table 才递归合并，否则直接覆盖
  for (auto&& [k, v] : overlay) {
    if (out.contains(k) && out.at(k).is_table() && v.is_table()) {
      toml::table merged = merge_configs(*out.at(k).as_table(), *v.as_table());
      out.insert_or_assign(k, std::move(merged));
    } else {
      detail::copy_node(out, k, v);
    }
  }

  return out;
}

/// 便捷入口：两个 TOML 文本合并成一个表。
inline toml::table merge_texts(std::string_view base_text, std::string_view overlay_text) {
  toml::parse_result base = toml::parse(base_text);
  toml::parse_result overlay = toml::parse(overlay_text);
  return merge_configs(base, overlay);
}

}  // namespace rm::toml_reflect
