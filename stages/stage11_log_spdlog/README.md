# stage11_log_spdlog —— log 模块（spdlog 全局钩子）

## 功能分析

`log` 模块本体是一个 `spdlog_hook.hpp`（真实项目约 149 行）：

- `rm::log::init(level, pattern)`：进程启动时一处调用，统一 pattern / 级别 / flush 策略；
- `rm::log::set_level(level)`：运行时热更新级别；
- error 及以上立即 flush——崩溃前的日志不丢。

**为什么不直接在业务代码里裸用 spdlog**：
1. 全项目只有一处决定格式，改 pattern 不用改一百个文件；
2. `SPDLOG_ACTIVE_LEVEL` 是**按翻译单元**生效的编译期裁剪，各 TU 阈值不一致会出现"同一条日志有的文件打得出来、有的打不出来"——必须全局统一；
3. 落盘/刷盘策略集中管理。

## 运行方法

```bash
sudo apt install libspdlog-dev   # 依赖
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/demo
```

## 预期输出

```
=== stage11_log_spdlog ===
[test 1] 统一 pattern 格式
  捕获 5 行日志，示例:
    [2026-08-29 13:10:56.530] [trace] trace msg
  [PASS] 包含 "[info] hello 42"
  ...
[test 4] 编译期裁剪（另一个 TU 的 ACTIVE_LEVEL=warn）
  [PASS] SPDLOG_ACTIVE_LEVEL=warn 的 TU：info 宏展开为空
  ...

ALL TESTS PASSED
```

关键实验：`src/compile_time_trim.cpp` 在 include spdlog 前定义了
`SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_WARN`，于是其中的 `SPDLOG_INFO` 展开为空——
即使运行时 `set_level(trace)` 也救不回来。这就是阈值必须统一的原因。

## 与真实项目的对应

- 本 stage 的 `include/log/` 与 `crates/log/src` 同构，真实项目里 include 路径指向后者即可。
- 验证手段（dup/dup2 捕获 stdout 断言格式）可直接用于 CI。
