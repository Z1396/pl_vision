#pragma once
// log 模块：spdlog 全局初始化钩子（真实项目约 149 行）。
//
// 为什么不直接在业务代码里裸用 spdlog：
//   1. 全项目只有一处决定 pattern / 级别 / 落盘策略，改格式不用改 100 个文件；
//   2. SPDLOG_ACTIVE_LEVEL 决定宏的编译期裁剪，阈值必须统一，否则各 TU 不一致；
//   3. flush 策略（error 立即落盘）集中在 init 里。
//
// 真实项目中位于 crates/log/src/spdlog_hook.hpp。

#include <spdlog/spdlog.h>

#include <cstdint>
#include <string_view>

namespace rm::log {

enum class Level : std::uint8_t {
  trace = 0,
  debug = 1,
  info = 2,
  warn = 3,
  error = 4,
  critical = 5,
  off = 6,
};

inline spdlog::level::level_enum to_spdlog(Level lv) {
  return static_cast<spdlog::level::level_enum>(static_cast<int>(lv));
}

/// 默认 pattern：[时间] [级别] 消息，与 RM 项目日志规范一致。
inline constexpr std::string_view kDefaultPattern = "[%Y-%m-%d %H:%M:%S.%e] [%l] %v";

/// 进程内第一次调用时完成 spdlog 全局配置；后续调用可热更新级别。
inline void init(Level level = Level::info, std::string_view pattern = kDefaultPattern) {
  spdlog::set_pattern(std::string{pattern});
  spdlog::set_level(to_spdlog(level));
  // error 及以上立即 flush，崩溃前的日志不丢。
  spdlog::flush_on(spdlog::level::err);
}

/// 热更新运行时级别（不动 pattern）。
inline void set_level(Level level) { spdlog::set_level(to_spdlog(level)); }

}  // namespace rm::log
