// stage11_log_spdlog：log 模块（spdlog 全局钩子）教学 demo。
//
// 三个观察点：
//   1. init() 之后，SPDLOG_* 宏输出统一 pattern：[时间] [级别] 消息；
//   2. 运行时级别裁剪：set_level(info) 后 SPDLOG_DEBUG 不再输出；
//   3. 编译期裁剪：SPDLOG_ACTIVE_LEVEL 低于宏级别时宏展开为空，代码根本不进二进制。
//
// 验证方式：把 stdout 重定向到临时文件（dup/dup2），读回字符串做断言——
// 这正是项目里“配置缺字段直接退出”规范要求的可测试日志。

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

// 必须在 include spdlog 之前定义：本 TU 的日志最低保留到 debug。
// 若定义为 SPDLOG_LEVEL_WARN，则下面所有 SPDLOG_INFO/DEBUG 宏都编译为空。
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#include "log/spdlog_hook.hpp"

namespace {

int g_failures = 0;

void check(bool ok, const char* what) {
  if (ok) {
    std::printf("  [PASS] %s\n", what);
  } else {
    ++g_failures;
    std::printf("  [FAIL] %s\n", what);
  }
}

/// 把 stdout 重定向到 path，执行 fn，恢复 stdout，返回捕获内容。
template <class Fn>
std::string capture_stdout(const char* path, Fn&& fn) {
  std::fflush(stdout);
  const int saved = dup(STDOUT_FILENO);
  const int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
  dup2(fd, STDOUT_FILENO);
  close(fd);

  fn();

  std::fflush(stdout);
  spdlog::default_logger()->flush();
  dup2(saved, STDOUT_FILENO);
  close(saved);

  std::ifstream in{path};
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

// ---------------------------------------------------------------------------
// test 1: init 后各级别按统一 pattern 输出
// ---------------------------------------------------------------------------
void test_pattern_format() {
  std::puts("[test 1] 统一 pattern 格式");

  const std::string out = capture_stdout("/tmp/stage11_t1.log", [] {
    rm::log::init(rm::log::Level::trace);
    SPDLOG_TRACE("trace msg {}", 1);
    SPDLOG_DEBUG("debug msg {}", 2);
    SPDLOG_INFO("hello {}", 42);
    SPDLOG_WARN("warn msg");
    SPDLOG_ERROR("error msg");
  });

  std::printf("  捕获 %zu 行日志，示例:\n    %s",
              std::count(out.begin(), out.end(), '\n'), out.substr(0, 44).c_str());

  // 格式断言：[YYYY-MM-DD HH:MM:SS.mmm] [info] hello 42
  const auto pos = out.find("[info] hello 42");
  check(pos != std::string::npos, "包含 \"[info] hello 42\"");
  check(out.find("trace msg 1") != std::string::npos, "trace 级别可输出");
  check(out.find("debug msg 2") != std::string::npos, "debug 级别可输出");
  check(out.find("[error] error msg") != std::string::npos, "error 级别可输出");

  // 时间戳格式："[2026-08-29 13:10:27.467] " 恰好 26 字符，[info] 紧随其后
  bool ts_ok = false;
  if (pos != std::string::npos && pos >= 26 && out[pos - 26] == '[') {
    const std::string ts = out.substr(pos - 25, 25);
    ts_ok = ts[4] == '-' && ts[7] == '-' && ts[10] == ' ' && ts[13] == ':' && ts[16] == ':' &&
            ts[19] == '.' && ts[23] == ']';
  }
  check(ts_ok, "时间戳格式 [YYYY-MM-DD HH:MM:SS.mmm]");
}

// ---------------------------------------------------------------------------
// test 2: 运行时级别裁剪 —— set_level(info) 后 debug/trace 不输出
// ---------------------------------------------------------------------------
void test_runtime_level_filter() {
  std::puts("[test 2] 运行时级别裁剪");

  const std::string out = capture_stdout("/tmp/stage11_t2.log", [] {
    rm::log::init(rm::log::Level::info);  // info 及以上
    SPDLOG_DEBUG("should NOT appear");
    SPDLOG_INFO("should appear");
  });

  check(out.find("should NOT appear") == std::string::npos, "debug 被运行时级别过滤");
  check(out.find("should appear") != std::string::npos, "info 正常输出");
}

// ---------------------------------------------------------------------------
// test 3: 热更新 —— init 后再次调低级别，日志恢复输出
// ---------------------------------------------------------------------------
void test_level_hot_update() {
  std::puts("[test 3] 级别热更新");

  const std::string out = capture_stdout("/tmp/stage11_t3.log", [] {
    rm::log::init(rm::log::Level::error);
    SPDLOG_WARN("suppressed");
    rm::log::set_level(rm::log::Level::warn);  // 热更新，不动 pattern
    SPDLOG_WARN("recovered");
  });

  check(out.find("suppressed") == std::string::npos, "error 级别下 warn 被抑制");
  check(out.find("recovered") != std::string::npos, "热更新后 warn 恢复输出");
  check(out.find("[warning] recovered") != std::string::npos, "pattern 未被热更新破坏");
}

// ---------------------------------------------------------------------------
// test 4: 编译期裁剪演示 —— 同一二进制里另一个 TU 编到 SPDLOG_ACTIVE_LEVEL=warn
// （见 compile_time_trim.cpp：其 SPDLOG_INFO 展开为空，永不输出）
// ---------------------------------------------------------------------------

}  // namespace

// 另一个翻译单元：SPDLOG_ACTIVE_LEVEL=warn，info 宏被编译器裁掉。
// 用非 inline 函数 + 弱符号计数，验证“宏没有产生任何输出”。
namespace {
std::string g_from_other_tu;
}

// 定义在 compile_time_trim.cpp
std::string trimmed_tu_output();

void test_compile_time_trim() {
  std::puts("[test 4] 编译期裁剪（另一个 TU 的 ACTIVE_LEVEL=warn）");

  const std::string out = capture_stdout("/tmp/stage11_t4.log", [] {
    rm::log::init(rm::log::Level::trace);  // 运行时全开也救不回来
    g_from_other_tu = trimmed_tu_output();
  });

  check(out.find("info from trimmed TU") == std::string::npos,
        "SPDLOG_ACTIVE_LEVEL=warn 的 TU：info 宏展开为空");
  check(out.find("[warning] warn from trimmed TU") != std::string::npos,
        "同 TU 的 warn 宏正常输出");
  check(g_from_other_tu == "ok", "函数返回值不受裁剪影响");
}

int main() {
  std::puts("=== stage11_log_spdlog ===");
  test_pattern_format();
  test_runtime_level_filter();
  test_level_hot_update();
  test_compile_time_trim();
  std::printf("\n%s\n", g_failures == 0 ? "ALL TESTS PASSED" : "SOME TESTS FAILED");
  return g_failures == 0 ? 0 : 1;
}
