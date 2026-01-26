#include "exiter.hpp"

#include <csignal>
#include <stdexcept>

namespace tools
{
bool exit_ = false;
bool exiter_inited_ = false; //namespace下的全局变量（不属于类成员），不受const成员函数的约束，修改它们是合法的。

Exiter::Exiter()
{
  if (exiter_inited_) 
  {
    throw std::runtime_error("Multiple Exiter instances!");
  }
  std::signal(SIGINT, [](int signum) { exit_ = true; });
  exiter_inited_ = true;
}

bool Exiter::exit() const { return exit_; }

}  // namespace tools