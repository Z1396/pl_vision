#pragma once
// channel：三缓冲之上的通道封装（两层结构的上层）。
//
// 下层 triple_buffer 解决"怎么无锁传最新值"；本层解决"端点生命周期与绑定纪律"：
//   - RAII 端点：Writer/Reader 持 shared_ptr<State>，缓冲随最后一个端点存活/释放；
//     Writer 析构 = 关闭通道（set closed），reader 可感知；
//   - claimed 防重复绑定：一个通道只能 claim 一次 writer / 一次 reader，
//     split() 一次性拆出 (writer, reader)，二次 split 返回 nullopt ——
//     编译期防不住"把同一个缓冲绑给两个线程"，就在运行期第一次就拒绝。
//
// 真实项目中位于 crates/primitive/src/channel.hpp。

#include <atomic>
#include <memory>
#include <optional>
#include <utility>

#include "primitive/triple_buffer.hpp"

namespace rm::primitive {

template <class T>
class SpscChannel {
  struct State {
    TripleBuffer<T> buffer;
    std::atomic<bool> writer_claimed{false};
    std::atomic<bool> reader_claimed{false};
    std::atomic<bool> closed{false};
  };

 public:
  class Writer {
   public:
    explicit Writer(std::shared_ptr<State> state) : state_(std::move(state)) {}

    /// RAII：writer 析构 = 通道关闭。
    ~Writer() {
      if (state_ != nullptr) {
        state_->closed.store(true, std::memory_order_release);
      }
    }

    Writer(Writer&&) = default;
    Writer& operator=(Writer&&) = default;
    Writer(const Writer&) = delete;
    Writer& operator=(const Writer&) = delete;

    /// 投递一帧（覆盖旧帧）。通道已关闭时拒绝写入。
    bool send(T value) {
      if (state_->closed.load(std::memory_order_acquire)) {
        return false;
      }
      state_->buffer.write(std::move(value));
      return true;
    }

   private:
    std::shared_ptr<State> state_;
  };

  class Reader {
   public:
    explicit Reader(std::shared_ptr<State> state) : state_(std::move(state)) {}

    Reader(Reader&&) = default;
    Reader& operator=(Reader&&) = default;
    Reader(const Reader&) = delete;
    Reader& operator=(const Reader&) = delete;

    /// 非阻塞读最新帧；无新数据返回 false。
    bool receive(T& out) { return state_->buffer.read(out); }

    /// writer 是否已关闭（仍可把已写入的最后一帧读走）。
    [[nodiscard]] bool is_closed() const {
      return state_->closed.load(std::memory_order_acquire);
    }

   private:
    std::shared_ptr<State> state_;
  };

  /// 创建一个 SPSC 通道。
  [[nodiscard]] static SpscChannel make() { return SpscChannel{}; }

  /// 认领 writer（仅一次；已被认领返回 nullopt）。
  std::optional<Writer> claim_writer() {
    if (state_->writer_claimed.exchange(true, std::memory_order_acq_rel)) {
      return std::nullopt;
    }
    return Writer{state_};
  }

  /// 认领 reader（仅一次；已被认领返回 nullopt）。
  std::optional<Reader> claim_reader() {
    if (state_->reader_claimed.exchange(true, std::memory_order_acq_rel)) {
      return std::nullopt;
    }
    return Reader{state_};
  }

  /// 一次性拆出 (writer, reader)；任一端点已被认领则失败。
  std::optional<std::pair<Writer, Reader>> split() {
    auto writer = claim_writer();
    auto reader = claim_reader();
    if (writer == std::nullopt || reader == std::nullopt) {
      return std::nullopt;  // 已认领的端点随 optional 析构而释放（writer 会置 closed）
    }
    return std::make_pair(std::move(*writer), std::move(*reader));
  }

 private:
  SpscChannel() = default;
  std::shared_ptr<State> state_ = std::make_shared<State>();
};

/// 便捷工厂：auto [writer, reader] = make_spsc_channel<Frame>().split();
template <class T>
[[nodiscard]] SpscChannel<T> make_spsc_channel() {
  return SpscChannel<T>::make();
}

}  // namespace rm::primitive
