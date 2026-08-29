// system_info 的实现：优先走 POSIX sysconf，拿不到再退回标准库。

#include "primitive/system_info.hpp"

#include <thread>
#include <unistd.h>

namespace rm::primitive::system_info {

std::uint32_t cpu_cores() {
  const long n = sysconf(_SC_NPROCESSORS_ONLN);
  if (n > 0) {
    return static_cast<std::uint32_t>(n);
  }
  const unsigned hw = std::thread::hardware_concurrency();
  return hw > 0 ? static_cast<std::uint32_t>(hw) : 1;
}

}  // namespace rm::primitive::system_info
