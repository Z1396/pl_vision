#pragma once
// validation：坐标变换合法性校验。
//
// 静态部分（编译期）：frame.hpp 的类型层级 + transform.hpp 的 requires 约束
//   已经保证"单条查询"不可能拼错链 —— 这是 fast_tf 比 string-frame TF 强的地方。
// 动态部分（运行期）：注册表可能出现环（典型事故：标定时把 camera 的 parent
//   填成 gimbal_pitch，又把 gimbal_pitch 的 parent 填成 camera）。本文件做环检测。
//
// 真实项目中位于 crates/fast_tf/src/validation.cpp。

#include <algorithm>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "fast_tf/frame.hpp"

namespace rm::fast_tf {

/// 校验注册表：
///   1) 每条 child->parent 边的 parent 必须存在（或为根 ""）；
///   2) 沿 parent 链上行必须最终到达根，途中不得重复（无环）。
/// 返回 nullopt = 合法；否则返回首个发现的错误描述。
inline std::optional<std::string> validate() {
  std::scoped_lock lock{detail::registry_mutex()};
  const auto& reg = detail::registry();

  for (const auto& [child, parent] : reg) {
    // 沿 parent 链上行，visited 检测环
    std::vector<std::string> chain{child};
    std::string cur = parent;
    while (!cur.empty()) {
      if (std::find(chain.begin(), chain.end(), cur) != chain.end()) {
        chain.push_back(cur);
        // 找到环：从首次出现处截取
        const auto first = std::find(chain.begin(), chain.end(), cur);
        std::string path;
        for (auto it = first; it != chain.end(); ++it) {
          path += *it + " -> ";
        }
        path += cur;
        return std::string{"cycle detected: "} + path;
      }
      chain.push_back(cur);
      const auto it = reg.find(cur);
      if (it == reg.end()) {
        return std::string{"unknown parent frame '"} + cur + "' (child '" + child + "')";
      }
      cur = it->second;
    }
  }
  return std::nullopt;
}

}  // namespace rm::fast_tf
