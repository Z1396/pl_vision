# stage7_primitive_tools —— primitive 模块其余 6 组件

## 功能分析

| 组件 | 核心功能 | 关键技术点 |
|---|---|---|
| `lazy.hpp` | 延迟构造：存参数包，调用时才 `new T` | 参数包存 `std::tuple`、`std::apply` 解包、`requires` 区分有/无预绑定参数两条重载路径 |
| `overloaded.hpp` | `std::visit` 多 lambda visitor | C++17 pack expansion 继承 + `using Ts::operator()...` |
| `spin.hpp` | 自旋等待提示 `SPIN_HINT()` | x86 `_mm_pause` / ARM `yield`，降低自旋功耗与流水线冲刷 |
| `performance_probe.hpp` | 时延统计 `LatencyHistogram` | steady_clock 无锁计时；固定 8192 滑动窗口 + 原子 min/max；算 mean/p50/p95/p99/stddev |
| `system_info.hpp` | 系统信息查询 | `sysconf(_SC_NPROCESSORS_ONLN)`，失败退回 `hardware_concurrency` |
| `channel.hpp` | 三缓冲之上的通道封装 | RAII 端点、`claimed` 防重复绑定、`make_spsc_channel`/`split` |

`channel` 的底层是 `triple_buffer.hpp`（stage4 已精讲的 SPSC 三缓冲，latest-value 语义：
writer 比 reader 快时覆盖中间帧——宁可丢旧帧，不排队积压）。

## 运行方法

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/demo
```

## 预期输出（节选）

```
=== stage7_primitive_tools ===
[test 1] lazy 延迟构造
  [PASS] 预绑定路径：get() 之前不构造
  [PASS] 构造计数 == 0
  [PASS] 多次 get() 恰好构造 1 次
  ...
[test 4] LatencyHistogram 已知分布验证
  count=105  min=100  p50=100  p95=100  p99=10000  max=10000  stddev=2108
  ...
[test 6] make_spsc_channel / split / 写读一帧
  [PASS] 二次 claim_writer 被拒绝
  [PASS] 拿到的是最新一帧（id=3，中间帧被覆盖）
  [PASS] writer 析构后 reader 观察到 closed

ALL TESTS PASSED
```

三个值得注意的实现细节（都在代码注释里）：

1. **三缓冲初始索引必须不同槽**：`latest_` 与 `read_idx_` 初始相同会导致第一次
   `write` 计算 `3 - l - r = -1` 越界（本 demo 踩过的坑）；
2. **SPIN_HINT 对比实验防优化**：空转循环里若累加量可闭式求和，编译器会把整个循环
   折叠掉，计时恒为 0——用 volatile 读打断；
3. **直方图 p99 用最近邻索引** `sorted[floor(q*(n-1))]`：灌 100 个 100ns + 5 个 10μs，
   p50=100、p99=10000，正对应"检测时延主体 + GC/调度尾部"的真实形态。

## 与真实项目的对应

- 本 stage 的 `include/primitive/` 与 `crates/primitive/src` 同构；真实项目把 CMake
  include 路径指向后者即可。
- `overloaded` 的三策略分发直接对应 `output_interface.cpp` 的 `std::visit` 用法。
- `channel` 是把 stage4（底层无锁结构）与调度器（上层组件）接通的关键一课。
