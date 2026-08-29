#pragma once
// performance_probe：时延统计探针。
//
// 设计要点：
//   - 采样用 steady_clock（单调，不受 NTP 跳变影响），无锁（单线程灌入）；
//   - 直方图 = 固定 8192 滑动窗口 ring buffer，内存恒定、自动遗忘久远尾部；
//   - min/max 用 atomic 维护（多线程灌入也安全），统计时对窗口快照排序；
//   - 统计口径：mean / p50 / p95 / p99 / stddev（总体方差）/ min / max / count。
//
// 真实项目中位于 crates/primitive/src/performance_probe.hpp。

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <vector>

namespace rm::primitive {

class LatencyHistogram {
 public:
  static constexpr std::size_t kWindowSize = 8192;

  /// 灌入一个时延采样（单位：ns）。
  void record(std::uint64_t ns) {
    window_[count_ % kWindowSize] = ns;
    ++count_;

    // 原子维护 min/max：即便多生产者灌入，最终值也正确。
    auto relax = std::memory_order_relaxed;
    std::uint64_t cur_min = min_.load(relax);
    while (ns < cur_min && !min_.compare_exchange_weak(cur_min, ns, relax)) {
    }
    std::uint64_t cur_max = max_.load(relax);
    while (ns > cur_max && !max_.compare_exchange_weak(cur_max, ns, relax)) {
    }
  }

  struct Stats {
    std::uint64_t count{};
    std::uint64_t min{};
    std::uint64_t max{};
    double mean{};
    double p50{};
    double p95{};
    double p99{};
    double stddev{};
  };

  /// 对当前窗口（最近 min(count, 8192) 个样本）做统计。
  [[nodiscard]] Stats stats() const {
    Stats s{};
    const std::size_t n = std::min<std::size_t>(count_, kWindowSize);
    if (n == 0) {
      return s;
    }

    // 按时间顺序取回窗口快照（ring buffer 中最早的样本在前）。
    std::vector<std::uint64_t> samples(n);
    const std::uint64_t start = count_ - n;
    for (std::size_t i = 0; i < n; ++i) {
      samples[i] = window_[(start + i) % kWindowSize];
    }

    s.count = count_;  // 总采样数（含已滑出窗口的）
    s.min = min_.load(std::memory_order_relaxed);
    s.max = max_.load(std::memory_order_relaxed);

    double sum = 0.0;
    for (auto v : samples) {
      sum += static_cast<double>(v);
    }
    s.mean = sum / static_cast<double>(n);

    double var = 0.0;
    for (auto v : samples) {
      const double d = static_cast<double>(v) - s.mean;
      var += d * d;
    }
    s.stddev = std::sqrt(var / static_cast<double>(n));

    std::sort(samples.begin(), samples.end());
    const auto pct = [&samples, n](double q) {
      const auto idx = static_cast<std::size_t>(q * static_cast<double>(n - 1));
      return static_cast<double>(samples[idx]);
    };
    s.p50 = pct(0.50);
    s.p95 = pct(0.95);
    s.p99 = pct(0.99);
    return s;
  }

 private:
  std::array<std::uint64_t, kWindowSize> window_{};
  std::uint64_t count_{0};                        // 滑动窗口写指针（单写者）
  std::atomic<std::uint64_t> min_{UINT64_MAX};    // 全历史最小
  std::atomic<std::uint64_t> max_{0};             // 全历史最大
};

/// 计时 guard：析构时把区间时延灌入直方图。
class ScopedLatencyProbe {
 public:
  ScopedLatencyProbe(LatencyHistogram& hist)
      : hist_(hist), begin_(std::chrono::steady_clock::now()) {}
  ~ScopedLatencyProbe() {
    const auto end = std::chrono::steady_clock::now();
    hist_.record(static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin_).count()));
  }
  ScopedLatencyProbe(const ScopedLatencyProbe&) = delete;
  ScopedLatencyProbe& operator=(const ScopedLatencyProbe&) = delete;

 private:
  LatencyHistogram& hist_;
  std::chrono::steady_clock::time_point begin_;
};

}  // namespace rm::primitive
