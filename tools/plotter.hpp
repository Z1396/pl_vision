#ifndef TOOLS__PLOTTER_HPP
#define TOOLS__PLOTTER_HPP

/*1. 核心作用
该头文件是Linux/Unix 系统下 TCP/IP 网络编程的核心系统头文件，
专门用于定义IPv4 网络通信的底层数据结构、宏常量和函数原型，是实现 socket 网络编程的基础，无法替代。*/
#include <netinet/in.h>  // sockaddr_in

/*1. 核心作用
这是 C++11 及以上标准库中互斥量（互斥锁）的全局 / 局部变量声明，定义在<mutex>头文件中，核心作用是实现多线程间的 “互斥访问”，
保护共享资源不被多个线程同时修改 / 读取，避免出现数据竞争（Data Race）和未定义行为，\

2. 关键特性
属于C++ 标准库（而非系统 API），跨平台性好（Linux/Windows/macOS 均支持）；
核心操作：lock()（加锁）、unlock()（解锁）、try_lock()（尝试加锁，非阻塞）；
最佳实践：配合RAII 风格的std::lock_guard/std::unique_lock使用，自动加锁 / 解锁，避免手动解锁遗漏导致的死锁。
    */
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>

namespace tools
{
class Plotter
{
public:
  Plotter(std::string host = "127.0.0.1", uint16_t port = 9870);

  ~Plotter();

  void plot(const nlohmann::json & json);

private:
  int socket_;                      // UDP套接字句柄（文件描述符，int类型）
  struct sockaddr_in destination_;  // 目标服务端的IPv4地址结构（来自<netinet/in.h>）struct可以省略

  /*1. std::mutex mutex_：通用的互斥访问锁
  核心定位：C++11 引入的互斥锁原语，用于实现代码块 / 复杂资源的互斥访问（同一时间只有一个线程能执行加锁后的代码）。
  设计目标：保护多步操作或复杂共享资源（如std::queue、cv::Mat、自定义类对象），防止多线程同时修改导致数据竞争和数据错乱。
  典型用途：你的线程安全队列中用 mutex_ 保护底层的 std::queue<T> queue_，保证push/pop/clear等操作的原子性。
  2. std::mutex：操作系统级别的互斥锁
  互斥锁的实现依赖操作系统的线程调度，核心是 “加锁 - 执行 - 解锁” 的流程，存在锁开销和线程阻塞：
    线程 A 执行 std::unique_lock<std::mutex> lock(mutex_); 时，若锁未被占用，则获取锁并继续执行；若锁已被线程 B 占用，则阻塞当前线程（放弃 CPU 使用权，进入等待队列）。
    线程 B 执行完加锁代码块并解锁后，操作系统会从等待队列中唤醒一个线程（如线程 A），使其获取锁并执行代码。
    解锁操作由std::unique_lock的析构函数自动完成，避免手动解锁遗漏导致死锁。*/
  std::mutex mutex_;
};

}  // namespace tools

#endif  // TOOLS__PLOTTER_HPP