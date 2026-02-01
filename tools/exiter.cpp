#include "exiter.hpp"

#include <csignal>

/*使用std::runtime_error的代码，必须显式包含<stdexcept>，
这是其定义所在的标准头文件（无需额外包含<exception>，<stdexcept>会间接引入）。*/
#include <stdexcept>

namespace tools
{
bool exit_ = false;
bool exiter_inited_ = false; //namespace下的全局变量（不属于类成员），不受const成员函数的约束，修改它们是合法的。

Exiter::Exiter()
{
  if (exiter_inited_) 
  {
    /*std::runtime_error 是 C++ 标准库中最常用的运行时异常类，属于<stdexcept>头文件，继承自std::exception基类，
    专门用于表示程序运行期间发生的、可预测但无法在编译期检测的错误（区别于编译错误、逻辑错误）。*/
    throw std::runtime_error("Multiple Exiter instances!");
  }

  /*std::signal：C++ 标准库中的信号处理注册函数（头文件<csignal>），功能是将「信号类型」与「信号处理函数 / 可调用对象」绑定，原型简化为：
  cpp
  运行

  void (*signal(int sig, void (*func)(int)))(int);

  第一个参数是信号类型，第二个参数是信号触发时执行的处理逻辑（可接受函数指针、符合签名的 lambda 等）。
  SIGINT：中断信号（Interrupt Signal），是操作系统定义的标准信号（值通常为 2），最常见的触发方式是在终端中按下Ctrl+C，用于向前台运行的程序发送 “强制中断” 指令。
  lambda 表达式[](int signum) { exit_ = true; }：

  是信号处理函数的简化写法，替代了传统的独立函数，语法更简洁；
  形参signum：接收触发的信号值（此处为SIGINT的数值），符合std::signal要求的处理函数签名（必须接收int类型参数，无返回值）；
  核心逻辑exit_ = true;：将全局 / 类成员标志位exit_设为true，替代了直接终止程序的默认行为，实现 “优雅退出”—— 程序不会立即终止，而是通过检测exit_的状态，在合适的时机完成资源释放、数据保存等操作后再退出。*/
  std::signal(SIGINT, [](int signum) { exit_ = true; });
  exiter_inited_ = true;
}

bool Exiter::exit() const { return exit_; }

}  // namespace tools