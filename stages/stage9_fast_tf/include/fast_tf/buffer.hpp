#pragma once
// TfBuffer：每条边的带时间戳环形历史 + 查询插值。
//
// 每个帧类型一个 buffer（模板单例 frame_buffer<F>()，与 TF2 的 per-frame cache
// 等价）。lookup(t)：
//   - 找到恰好匹配的时间戳：直接返回；
//   - t 落在两个样本之间：平移线性插值 + 旋转 slerp（最短路径）；
//   - t 晚于最新样本：返回最新样本（clamp，TF2 的"不外推到未来"惯例）；
//   - t 早于最老样本：返回错误（历史太老，不可信）。
//
// 真实项目中位于 crates/fast_tf/src/buffer.cpp（ring buffer 实现）。

#include <algorithm>
#include <expected>
#include <string>
#include <vector>

#include "fast_tf/transform.hpp"

namespace rm::fast_tf {

template <class F>
class TfBuffer {
 public:
  static constexpr std::size_t kCapacity = 1024;

  /// 插入一个样本（按时间戳保序，超出容量丢弃最老的）。
  void insert(EdgeTransform<F> sample) {
    const auto pos =
        std::lower_bound(storage_.begin(), storage_.end(), sample.stamp,
                         [](const EdgeTransform<F>& s, double t) { return s.stamp < t; });
    storage_.insert(pos, std::move(sample));
    if (storage_.size() > kCapacity) {
      storage_.erase(storage_.begin());
    }
  }

  /// 查询时刻 stamp 的边变换。
  std::expected<EdgeTransform<F>, std::string> lookup(double stamp) const {
    if (storage_.empty()) {
      return std::unexpected("no sample for this frame");
    }
    if (stamp < storage_.front().stamp) {
      return std::unexpected("requested stamp is older than the oldest sample");
    }
    if (stamp >= storage_.back().stamp) {
      return storage_.back();  // clamp 到最新（不向未来外推）
    }
    // 二分找 bracketing 的两个样本
    const auto upper =
        std::upper_bound(storage_.begin(), storage_.end(), stamp,
                         [](double t, const EdgeTransform<F>& s) { return t < s.stamp; });
    const auto& a = *(upper - 1);
    const auto& b = *upper;
    return interpolate(a, b, stamp);
  }

  [[nodiscard]] std::size_t size() const { return storage_.size(); }

 private:
  static EdgeTransform<F> interpolate(const EdgeTransform<F>& a, const EdgeTransform<F>& b,
                                      double stamp) {
    const double span = b.stamp - a.stamp;
    const double alpha = (stamp - a.stamp) / span;

    EdgeTransform<F> out;
    out.stamp = stamp;
    out.tf.translation() =
        (1.0 - alpha) * a.tf.translation() + alpha * b.tf.translation();

    // 旋转用四元数 slerp（最短路径，跨 ±pi 不跳变）
    const Eigen::Quaterniond qa{a.tf.linear()};
    const Eigen::Quaterniond qb{b.tf.linear()};
    out.tf.linear() = qa.slerp(alpha, qb).toRotationMatrix();
    return out;
  }

  std::vector<EdgeTransform<F>> storage_;  // 按 stamp 升序（容量截断的 ring 语义）
};

/// 每个帧类型的全局唯一 buffer（编译期按类型分发，运行期单例）。
template <class F>
TfBuffer<F>& frame_buffer() {
  static TfBuffer<F> buffer;
  return buffer;
}

}  // namespace rm::fast_tf
