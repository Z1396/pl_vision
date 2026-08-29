#pragma once
// system_info：系统信息查询（真实项目 header + cpp 分离）。
//
// 真实项目中位于 crates/primitive/src/system_info.{hpp,cpp}。

#include <cstdint>

namespace rm::primitive::system_info {

/// 在线可用 CPU 核心数（sysconf(_SC_NPROCESSORS_ONLN)，失败时退回
/// std::thread::hardware_concurrency()）。
std::uint32_t cpu_cores();

}  // namespace rm::primitive::system_info
