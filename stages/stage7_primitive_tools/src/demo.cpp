// stage7_primitive_tools：primitive 模块其余 6 组件教学 demo。
//
//   lazy / overloaded / spin / performance_probe / system_info / channel
//
// 仿 stage4 的 test_xxx 风格：每个组件一个 test 函数，断言失败 -> 非零退出码。

#include <atomic>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>
#include <variant>
#include <vector>

#include "primitive/channel.hpp"
#include "primitive/lazy.hpp"
#include "primitive/overloaded.hpp"
#include "primitive/performance_probe.hpp"
#include "primitive/spin.hpp"
#include "primitive/system_info.hpp"

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

// ===========================================================================
// test 1: lazy —— 两条构造路径 + "调用前绝不构造"
// ===========================================================================
struct Heavy {
  static inline int ctors = 0;
  int a, b;
  Heavy(int x, int y) : a(x), b(y) { ++ctors; }
};

struct NoDefault {
  int v;
  explicit NoDefault(int x) : v(x) {}
};

void test_lazy() {
  std::puts("[test 1] lazy 延迟构造");

  // 路径 1：预绑定参数（构造期只存 tuple，不构造 Heavy）
  rm::primitive::Lazy<Heavy> prebound{7, 8};
  check(!prebound.is_constructed(), "预绑定路径：get() 之前不构造");
  check(Heavy::ctors == 0, "构造计数 == 0");
  Heavy* p1 = prebound.get();
  Heavy* p2 = prebound.get();
  check(Heavy::ctors == 1, "多次 get() 恰好构造 1 次");
  check(p1 == p2 && p1->a == 7 && p1->b == 8, "返回同一实例，参数包正确解包");

  // 路径 2：运行时参数（T 不可默认构造）
  rm::primitive::Lazy<NoDefault> runtime;
  NoDefault* q1 = runtime.get(42);
  NoDefault* q2 = runtime.get(99);  // 已构造：忽略参数，返回同一实例
  check(q1 == q2 && q1->v == 42, "运行时参数路径：首次 get(42) 生效，后续返回同一实例");
}

// ===========================================================================
// test 2: overloaded + variant —— 三策略分发（对应 OutputInterface）
// ===========================================================================
struct SerialOut {
  std::string data;
};
struct WebsocketOut {
  int code;
};
struct FoxgloveOut {
  double value;
};
using OutputInterface = std::variant<SerialOut, WebsocketOut, FoxgloveOut>;

void test_overloaded() {
  std::puts("[test 2] overloaded 多 lambda visitor");

  const auto describe = rm::primitive::overloaded{
      [](const SerialOut& s) { return "serial:" + s.data; },
      [](const WebsocketOut& w) { return "ws:" + std::to_string(w.code); },
      [](const FoxgloveOut& f) { return "foxglove:" + std::to_string(f.value); },
  };

  const OutputInterface a = SerialOut{"gimbal"};
  const OutputInterface b = WebsocketOut{7};
  const OutputInterface c = FoxgloveOut{3.5};

  check(std::visit(describe, a) == "serial:gimbal", "variant 备选 1 -> SerialOut lambda");
  check(std::visit(describe, b) == "ws:7", "variant 备选 2 -> WebsocketOut lambda");
  check(std::visit(describe, c) == "foxglove:3.500000", "variant 备选 3 -> FoxgloveOut lambda");
}

// ===========================================================================
// test 3: SPIN_HINT —— 自旋等待提示的定性对比
// ===========================================================================
volatile std::uint64_t g_sink = 0;

std::int64_t spin_loop(std::int64_t iters, bool with_hint) {
  const auto begin = std::chrono::steady_clock::now();
  std::uint64_t acc = 0;
  for (std::int64_t i = 0; i < iters; ++i) {
    acc += g_sink;  // volatile 读：阻止编译器把循环折叠成闭式求和
    if (with_hint) {
      SPIN_HINT();
    }
  }
  g_sink = acc;
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::steady_clock::now() - begin)
      .count();
}

void test_spin_hint() {
  std::puts("[test 3] SPIN_HINT 定性对比（100 万次空转）");

  constexpr std::int64_t kIters = 1'000'000;
  const std::int64_t t_plain = spin_loop(kIters, false);
  const std::int64_t t_pause = spin_loop(kIters, true);

  std::printf("  不加提示: %6ld us   加 SPIN_HINT: %6ld us\n", static_cast<long>(t_plain),
              static_cast<long>(t_pause));
  std::printf("  （SPIN_HINT 的收益在功耗/流水线冲刷/超线程邻居，时延上略有代价属正常）\n");

  check(t_plain > 0 && t_pause > 0, "两种循环均完整执行");
}

// ===========================================================================
// test 4: LatencyHistogram —— 已知分布灌入，验证分位数
// ===========================================================================
void test_performance_probe() {
  std::puts("[test 4] LatencyHistogram 已知分布验证");

  rm::primitive::LatencyHistogram hist;
  // 分布：100 个 100ns + 5 个 10us 尾部
  for (int i = 0; i < 100; ++i) {
    hist.record(100);
  }
  for (int i = 0; i < 5; ++i) {
    hist.record(10'000);
  }

  const auto s = hist.stats();
  std::printf("  count=%llu  min=%llu  p50=%.0f  p95=%.0f  p99=%.0f  max=%llu  stddev=%.0f\n",
              static_cast<unsigned long long>(s.count), static_cast<unsigned long long>(s.min),
              s.p50, s.p95, s.p99, static_cast<unsigned long long>(s.max), s.stddev);

  check(s.count == 105, "count == 105");
  check(s.min == 100 && s.max == 10'000, "min == 100, max == 10000");
  check(s.p50 == 100.0, "p50 == 100（主体分布）");
  check(s.p99 == 10'000.0, "p99 == 10000（尾部）");
  check(s.stddev > 0.0, "stddev > 0");

  // ScopedLatencyProbe：真实测量一段微小工作量的时延
  rm::primitive::LatencyHistogram probe_hist;
  for (int i = 0; i < 1000; ++i) {
    rm::primitive::ScopedLatencyProbe probe{probe_hist};
    volatile double x = 0.0;
    for (int j = 0; j < 100; ++j) {
      x = x * 1.000001 + 0.5;
    }
  }
  const auto ps = probe_hist.stats();
  std::printf("  ScopedLatencyProbe x1000:  p50=%.0fns  p99=%.0fns  max=%lluns\n", ps.p50, ps.p99,
              static_cast<unsigned long long>(ps.max));
  check(ps.count == 1000 && ps.max > 0, "probe 采样 count 正确且非零");
}

// ===========================================================================
// test 5: system_info
// ===========================================================================
void test_system_info() {
  std::puts("[test 5] system_info");

  const auto cores = rm::primitive::system_info::cpu_cores();
  std::printf("  cpu_cores = %u\n", cores);
  check(cores > 0, "cpu_cores > 0");
}

// ===========================================================================
// test 6: channel —— make -> split -> 写读一帧（接通 stage4 底层与调度器上层）
// ===========================================================================
struct Frame {
  int id;
  double stamp;
};

void test_channel() {
  std::puts("[test 6] make_spsc_channel / split / 写读一帧");

  auto channel = rm::primitive::make_spsc_channel<Frame>();

  // 防重复绑定：先单独 claim writer，split 必须失败
  auto early_writer = channel.claim_writer();
  check(early_writer.has_value(), "首次 claim_writer 成功");
  check(!channel.claim_writer().has_value(), "二次 claim_writer 被拒绝");
  check(!channel.split().has_value(), "writer 已被占用时 split 失败");

  // 释放早占的 writer（析构即关闭通道），重新来过 —— 新通道
  auto channel2 = rm::primitive::make_spsc_channel<Frame>();
  auto endpoints = channel2.split();
  check(endpoints.has_value(), "make -> split 成功");
  auto& [writer, reader] = *endpoints;

  // 写三帧（比读快），读侧只应拿到最新一帧 —— latest-value 语义
  check(writer.send(Frame{1, 0.1}), "send frame 1");
  check(writer.send(Frame{2, 0.2}), "send frame 2");
  check(writer.send(Frame{3, 0.3}), "send frame 3");

  Frame f{};
  check(reader.receive(f), "receive 拿到一帧");
  check(f.id == 3, "拿到的是最新一帧（id=3，中间帧被覆盖）");
  check(!reader.receive(f), "无新数据时 receive 返回 false");

  // RAII：writer 析构 = 通道关闭
  check(!reader.is_closed(), "writer 存活时通道未关闭");

  auto channel4 = rm::primitive::make_spsc_channel<Frame>();
  auto reader4 = channel4.claim_reader();
  {
    auto writer4 = channel4.claim_writer();
    writer4->send(Frame{11, 1.1});
  }  // writer4 析构 -> 通道关闭
  check(reader4.has_value() && reader4->is_closed(), "writer 析构后 reader 观察到 closed");
  Frame f4{};
  check(reader4->receive(f4) && f4.id == 11, "关闭后仍可读走最后写入的帧");
}

}  // namespace

int main() {
  std::puts("=== stage7_primitive_tools ===");
  test_lazy();
  test_overloaded();
  test_spin_hint();
  test_performance_probe();
  test_system_info();
  test_channel();
  std::printf("\n%s\n", g_failures == 0 ? "ALL TESTS PASSED" : "SOME TESTS FAILED");
  return g_failures == 0 ? 0 : 1;
}
