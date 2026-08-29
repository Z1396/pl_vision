// 编译期裁剪演示 TU：include spdlog 之前把 SPDLOG_ACTIVE_LEVEL 定为 warn。
// 结果：本文件里所有低于 warn 的宏（SPDLOG_TRACE/DEBUG/INFO）展开为空，
// 运行时 set_level(trace) 也无法恢复——这就是"全项目阈值必须统一"的原因。

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_WARN

#include <spdlog/spdlog.h>

#include <string>

#include "log/spdlog_hook.hpp"

std::string trimmed_tu_output() {
  // 编译期就被裁掉：展开为 static_cast<void>(0)，不产生任何输出。
  SPDLOG_INFO("info from trimmed TU");

  const std::string before = "ok";
  SPDLOG_WARN("warn from trimmed TU");
  return before;
}
