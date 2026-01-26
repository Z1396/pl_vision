#ifndef TOOLS__THREAD_SAFE_QUEUE_HPP
#define TOOLS__THREAD_SAFE_QUEUE_HPP

#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>

namespace tools
{
template <typename T, bool PopWhenFull = false>
class ThreadSafeQueue
{
public:
  ThreadSafeQueue(
    size_t max_size, std::function<void(void)> full_handler = [] {})
  : max_size_(max_size), full_handler_(full_handler)
  {
  }

/**
 * @brief 线程安全队列的元素入队操作
 * @param value 要入队的元素（常量引用，避免拷贝）
 * 功能：在互斥锁保护下将元素加入队列，处理队列满的情况，并通知等待的消费者线程
 */
void push(const T & value)
{
    // 加锁：std::unique_lock支持手动解锁，比lock_guard更灵活，适合条件变量场景
    std::unique_lock<std::mutex> lock(mutex_);

    // 检查队列是否已满（当前元素数 >= 最大容量）
    if (queue_.size() >= max_size_) 
    {
        // 根据模板参数PopWhenFull决定满队列策略
        if (PopWhenFull) 
        {
            // 若为true：弹出队首最旧元素，为新元素腾出空间（先进先出丢弃）
            queue_.pop();
        } else 
        {
            // 若为false：执行自定义满处理函数（如默认空操作或打印警告）
            full_handler_();
            return; // 不加入新元素，直接返回
        }
    }

    // 将新元素加入队列
    queue_.push(value);
    // 通知所有等待的消费者线程：队列已非空，可以尝试获取元素
    not_empty_condition_.notify_all();
}

/**
 * @brief 线程安全队列的元素出队操作
 * @param value 用于接收出队元素的引用
 * 功能：在互斥锁保护下等待队列非空，然后弹出队首元素，存储到value中
 */
void pop(T & value)
{
    // 加锁：与push操作共享同一把锁，保证线程安全
    std::unique_lock<std::mutex> lock(mutex_);

    // 等待条件变量：释放锁并阻塞，直到队列非空（唤醒后会重新加锁）
    // 第二个参数是 lambda 谓词，防止虚假唤醒（spurious wakeup）
    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

    // 双重检查：理论上此时队列不应为空，但极端情况（如其他线程同时取走元素）下做保护
    if (queue_.empty()) 
    {
        std::cerr << "Error: Attempt to pop from an empty queue." << std::endl;
        return;
    }

    // 将队首元素赋值给输出参数
    value = queue_.front();
    // 弹出队首元素
    queue_.pop();
}

  T pop()
  {
    std::unique_lock<std::mutex> lock(mutex_);

    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

    T value = std::move(queue_.front());
    queue_.pop();
    return std::move(value);
  }

  T front()
  {
    std::unique_lock<std::mutex> lock(mutex_);

    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

    return queue_.front();
  }

  void back(T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    if (queue_.empty()) {
      std::cerr << "Error: Attempt to access the back of an empty queue." << std::endl;
      return;
    }

    value = queue_.back();
  }

  bool empty()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  void clear()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!queue_.empty()) {
      queue_.pop();
    }
    not_empty_condition_.notify_all();  // 如果其他线程正在等待队列不为空，这样可以唤醒它们
  }

private:
  std::queue<T> queue_;   //是 C++ 标准库（<queue> 头文件）中的队列容器适配器，实现了先进先出（FIFO, First-In-First-Out） 的数据结构
  size_t max_size_;
  mutable std::mutex mutex_;
  std::condition_variable not_empty_condition_;
  std::function<void(void)> full_handler_;   // 声明了一个用于存储 “无参无返回值” 可调用对象的成员变量，核心作用是实现队列满时的自定义回调机制
};

}  // namespace tools

#endif  // TOOLS__THREAD_SAFE_QUEUE_HPP