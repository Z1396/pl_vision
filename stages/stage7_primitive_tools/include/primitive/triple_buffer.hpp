#pragma once
// triple_buffer：SPSC（单生产者-单消费者）三缓冲，latest-value 语义。
//
// 这是 stage4 的核心，channel（见 channel.hpp）建立在它之上。三缓冲的意义：
//   writer 永远写"既不是 latest 也不是 read"的第三个槽；
//   reader 永远读 latest；
//   全程只有 latest_ / read_idx_ / version_ 三个原子量上的顺序写，无锁、无等待。
//
// 语义注意：这是"最新值"通道 —— writer 比 reader 快时会覆盖中间帧（丢帧不阻塞），
// 恰是相机帧 → 检测器想要的模式：宁可丢旧帧，也不要排队积压。
//
// 真实项目中位于 crates/primitive/src/ 下（stage4 已精讲，此处为实现）。

#include <atomic>
#include <cstdint>
#include <utility>

namespace rm::primitive {

template <class T>
class TripleBuffer {
 public:
  /// 生产者：写入一个新值（总是成功；覆盖旧帧）。
  void write(T value) {
    const int latest = latest_.load(std::memory_order_acquire);
    const int read = read_idx_.load(std::memory_order_acquire);
    const int write_idx = 3 - latest - read;  // 三个槽中既非 latest 也非 read 的那个
    slots_[static_cast<std::size_t>(write_idx)] = std::move(value);
    latest_.store(write_idx, std::memory_order_release);      // 先发布槽位
    version_.fetch_add(1, std::memory_order_acq_rel);         // 再宣布"有新数据"
  }

  /// 消费者：无新数据返回 false；有则取出最新值（旧帧被跳过）。
  bool read(T& out) {
    const std::uint64_t v = version_.load(std::memory_order_acquire);
    if (v == consumed_version_) {
      return false;  // 没有比上次读取更新的数据
    }
    const int latest = latest_.load(std::memory_order_acquire);
    read_idx_.store(latest, std::memory_order_release);  // 先占住读槽，writer 随即避开它
    out = slots_[static_cast<std::size_t>(latest)];
    consumed_version_ = v;
    return true;
  }

  /// 消费者线程私有的"已读版本号"。
  std::uint64_t consumed_version_ = 0;

 private:
  T slots_[3]{};  // 要求 T 可默认构造（演示帧类型均满足）
  // 初始时 latest 与 read 必须是不同槽位，保证第一次 write 落在第三个槽。
  std::atomic<int> latest_{0};
  std::atomic<int> read_idx_{1};
  std::atomic<std::uint64_t> version_{0};
};

}  // namespace rm::primitive
