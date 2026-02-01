// 头文件保护宏：防止该头文件被多次包含，避免类/函数重复定义编译错误
// 命名规则：项目模块__文件名_HPP（大写，分隔符用__）
#ifndef TOOLS__THREAD_SAFE_QUEUE_HPP
#define TOOLS__THREAD_SAFE_QUEUE_HPP

// 引入依赖头文件：按需引入，保证编译通过且最小化依赖
#include <condition_variable>  // 条件变量：实现线程间的等待/通知机制，配合互斥锁使用
#include <functional>          // 函数包装器：支持存储自定义回调函数（队列满时的处理逻辑）
#include <iostream>            // 标准输入输出：用于打印错误信息（如空队列弹出）
#include <mutex>               // 互斥锁：保证多线程下对队列的原子操作，防止数据竞争
#include <queue>               // 标准队列：作为底层数据容器，实现FIFO先进先出基础功能

// 工具类命名空间：隔离项目内工具类，避免命名冲突
namespace tools
{
/**
 * @brief 通用型线程安全队列类
 * @tparam T 队列存储的元素类型（模板参数，支持任意可拷贝/可移动类型，如int、cv::Mat、自定义类等）
 * @tparam PopWhenFull 队列满时的处理策略（布尔型模板参数，默认false）
 *         - false：队列满时执行自定义回调，拒绝新元素入队（默认策略）
 *         - true：队列满时自动弹出队首最旧元素，为新元素腾出空间（覆盖策略）
 * @note 基于std::queue实现，结合互斥锁+条件变量保证多线程生产/消费的线程安全
 * @note 支持生产者-消费者模型，适用于多线程数据传递（如自瞄项目中相机帧传递、检测结果传递）
 */
/*一、根本原因：C++ 模板的「编译期实例化」特性（核心）
C++ 的类模板（template class） 并非真正的 “类”，而是 **「类的生成蓝图 / 模具」—— 编译器不会在编译头文件时为模板生成任何实际的机器代码 **，
只有当代码中显式实例化模板（如tools::ThreadSafeQueue<FrameData> queue_;）时，编译器才会根据指定的模板参数（如FrameData、PopWhenFull=true），
当场生成对应类型的类和成员函数代码，这个过程称为模板实例化，且必须在编译期完成。

模板实例化的关键要求是：编译器在实例化时，必须能看到模板成员函数的完整定义—— 如果将模板成员函数的定义放到.cpp 源文件中，
头文件仅保留声明，编译器在实例化模板时（通常在其他.cpp 文件中使用模板），无法找到函数的完整定义，会直接报 **“未定义的引用（undefined reference）”** 链接错误。*/
template <typename T, bool PopWhenFull = false>
class ThreadSafeQueue
{
public:
  /**
   * @brief 构造函数：初始化线程安全队列的最大容量和满队列回调函数
   * @param max_size 队列最大容量（size_t类型，非负，超过该值视为队列满）
   * @param full_handler 队列满时的自定义处理回调（默认空函数，无参无返回值）
   * @note 初始化列表初始化成员变量，符合C++最佳实践（避免二次赋值，提高效率）
   */
  ThreadSafeQueue(
    size_t max_size, 
    // 完整代码：构造函数的默认参数写法
    std::function<void(void)> full_handler = [] {})
    /*拆解1：std::function<void(void)> → 可调用对象的类型，代表「无参数、无返回值」的可调用对象
     拆解2：full_handler → 回调函数变量名，语义为「队列满时的处理程序」
     拆解3：= [] {} → 为变量指定默认值，空lambda表达式（无捕获、无执行体）*/ 
    : max_size_(max_size), full_handler_(full_handler)
    {
    }

  /**
   * @brief 线程安全入队操作（常量引用版）：向队列中添加元素，支持满队列策略处理
   * @param value 要入队的元素（const T& 常量左值引用，避免拷贝，提高效率）
   * @note 生产者线程调用，加锁保护队列操作，保证多线程入队的原子性
   * @note 入队成功后通知所有等待的消费者线程（队列已非空，可尝试弹出）
   */
  void push(const T & value)
  {
    // 加锁：使用std::unique_lock而非std::lock_guard，支持手动解锁/配合条件变量等待
    // 锁作用域：从构造到析构，自动释放，避免手动解锁遗漏导致死锁
    std::unique_lock<std::mutex> lock(mutex_);

    // 检查队列是否已满：当前元素数量 >= 预设的最大容量
    if (queue_.size() >= max_size_) 
    {
        // 根据模板参数PopWhenFull执行不同的满队列策略（编译期确定，无运行时开销）
        if (PopWhenFull) 
        {
            // 策略1：PopWhenFull=true，弹出队首最旧元素，为新元素腾出空间
            // 先进先出丢弃，保证队列容量不超限，适用于实时性要求高的场景（如帧数据传递）
            queue_.pop();
        } else 
        {
            // 策略2：PopWhenFull=false（默认），执行自定义满处理回调
            // 回调可实现打印警告、记录日志、统计丢包数等逻辑，默认空函数无操作
            full_handler_();
            return;  // 拒绝新元素入队，直接返回，队列保持满状态
        }
    }

    // 将新元素加入底层队列（std::queue的push方法，尾插法，符合FIFO）
    queue_.push(value);
    // 通知所有等待在not_empty_condition_上的消费者线程
    // 唤醒后消费者会重新检查队列非空条件，防止虚假唤醒，保证线程安全
    not_empty_condition_.notify_all();
  }

  /**
   * @brief 线程安全出队操作（输出参数版）：从队列弹出队首元素，通过引用返回
   * @param value 接收出队元素的引用（T& 左值引用，将队首元素赋值给该参数）
   * @note 消费者线程调用，若队列为空则阻塞等待，直到队列非空
   * @note 全程加锁保护，保证多线程出队的原子性，避免数据竞争
   */
  void pop(T & value)
  {
    // 加锁：与push操作共享同一把互斥锁，保证对队列的操作互斥
    std::unique_lock<std::mutex> lock(mutex_);

    // 等待条件变量：释放锁并阻塞当前线程，直到满足以下两个条件
    // 1. 其他线程调用notify_one/notify_all唤醒当前线程
    // 2. lambda谓词返回true（队列非空）
    // 防止虚假唤醒：即使被唤醒，也会重新检查谓词，确保队列真的非空
    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

    // 双重检查队列非空：理论上谓词保证队列非空，极端场景（如多线程同时消费）做兜底保护
    if (queue_.empty()) 
    {
        // 打印错误信息到标准错误流，便于问题排查
        std::cerr << "Error: Attempt to pop from an empty queue." << std::endl;
        return;
    }

    // 将队首元素赋值给输出参数（通过引用传递，避免拷贝，适合大对象）
    value = queue_.front();
    // 弹出队首元素，完成出队操作
    queue_.pop();
  }

  /**
   * @brief 线程安全出队操作（返回值版）：从队列弹出队首元素，直接返回该元素
   * @return T 弹出的队首元素（使用std::move移动语义，避免拷贝，提高效率）
   * @note 消费者线程调用，队列为空时阻塞等待
   * @note 适用于轻量对象，返回值通过移动而非拷贝，减少性能开销
   */
  T pop()
  {
    // 加锁保护队列操作
    std::unique_lock<std::mutex> lock(mutex_);

    // 等待队列非空，防止空队列弹出
    /*前提背景
执行这行代码前，消费者线程已经通过std::unique_lock<std::mutex> lock(mutex_);成功获取了互斥锁，独占mutex_，其他线程（生产者 / 其他消费者）无法操作队列。
情况 1：执行wait()时，队列已经非空（lambda 返回true）

    执行动作：直接跳过所有等待逻辑，不阻塞、不释放锁，继续执行后续的出队代码（value = queue_.front();）；
    适用场景：生产者已提前入队数据，消费者调用pop()时队列已有数据，无需等待，直接消费；
    核心价值：无多余开销，保证消费的及时性。

情况 2：执行wait()时，队列为空（lambda 返回false）
这是最常见的场景（如Recorder类初始状态队列为空，后台保存线程调用pop()），此时wait()会执行两个原子操作（不可拆分，保证线程安全）：

    主动释放持有的mutex_互斥锁
        关键目的：让生产者线程能正常获取锁，调用push()入队数据（若不释放锁，生产者会被永久阻塞，永远无法入队，消费者也会永久等待，形成死锁）；
        释放时机：锁的释放与线程的挂起是原子的，避免中间出现竞态条件。
    立即阻塞挂起当前消费者线程
        线程状态：当前线程进入等待状态（挂起），不再占用 CPU 资源，也不会执行任何代码；
        等待触发：线程会一直挂起，直到满足以下任一条件才会被唤醒：
        ✅ 生产者线程入队后调用not_empty_condition_.notify_all()/notify_one()（正常唤醒，核心场景）；
        ❗ 系统 / 编译器的虚假唤醒（极少数情况，操作系统的正常现象，无需刻意避免，后续会防护）。

情况 3：线程被唤醒后（无论正常唤醒还是虚假唤醒）
线程被唤醒后，不会立即执行后续代码，而是严格按顺序执行以下 3 个步骤，这是保证 “安全消费” 的核心，也是防护虚假唤醒的关键：

    重新尝试获取mutex_互斥锁
        若锁当前被其他线程（如另一个消费者 / 生产者）持有，则当前线程会阻塞等待获取锁，直到成功拿到锁的独占权；
        目的：保证后续对队列的操作（判空、出队）仍是原子操作，避免数据竞争。
    重新执行 lambda 谓词，检查队列状态
        再次调用[this] { return !queue_.empty(); }，判断队列是否真的非空；
        这是防护虚假唤醒的核心：若为虚假唤醒（队列仍为空），谓词返回false，线程会重复情况 2 的逻辑（释放锁 + 再次挂起）；若为正常唤醒（生产者已入队，队列非空），谓词返回true，进入下一步。
    退出wait()方法，继续执行后续代码
        此时线程重新持有互斥锁，且队列保证非空，可安全执行front()和pop()出队操作。
    
    四、核心底层原理：为什么必须结合「互斥锁 + lambda 谓词」？
这行代码的设计是 C++ 多线程同步的经典范式，其底层原理围绕 **“解决多线程的两大核心问题”** 展开，缺一不可：
1. 互斥锁（std::unique_lock）：解决数据竞争问题

    保证对共享资源（队列queue_）的所有操作都是原子的，避免多个线程同时修改 / 读取队列导致的数据错乱；
    配合wait()实现 “锁的临时释放”，让生产者和消费者能交替访问队列，形成正常的生产 - 消费链路。

2. lambda 谓词（条件判断）：解决虚假唤醒问题

    什么是虚假唤醒：操作系统为了实现条件变量的高效性，可能会在 ** 没有任何线程调用notify_*()** 的情况下，唤醒等待的线程，这种情况就是虚假唤醒（是标准允许的，并非 bug）；
    为什么谓词能防护：虚假唤醒后，队列仍为空，lambda 谓词返回false，线程会再次释放锁并挂起，相当于 “忽略” 了这次无效的唤醒，只有队列真的非空（谓词true），才会继续执行，从根本上避免了 “空队列出队” 的致命错误；
    核心原则：使用条件变量的wait()方法时，必须配合谓词，永远不要使用无谓词的wait(lock)，否则会因虚假唤醒导致程序崩溃。    */
    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });
    // 1. not_empty_condition_ → 类的std::condition_variable（条件变量）成员，专门用于“队列非空”的同步通知
    // 2. wait(...) → 条件变量的核心方法，实现“阻塞等待+条件校验”
    // 3. lock → 传入的std::unique_lock<std::mutex>对象（必须是unique_lock，lock_guard不支持）
    // 4. [this] { return !queue_.empty(); } → lambda表达式（谓词/条件判断函数），核心是“队列非空”的校验逻辑


    // 移动构造临时对象：将队首元素的资源所有权转移，避免拷贝（如std::string、cv::Mat等）
    T value = std::move(queue_.front());
    // 弹出队首元素
    queue_.pop();
    // 移动返回：进一步优化，避免返回值的拷贝构造
    return std::move(value);
  }

  /**
   * @brief 线程安全获取队首元素：获取但不弹出队首元素
   * @return T 队首元素（值返回，若需避免拷贝可自行修改为引用版）
   * @note 消费者线程调用，队列为空时阻塞等待
   * @note 与pop的区别：仅查看队首，不修改队列状态
   */
  T front()
  {
    std::unique_lock<std::mutex> lock(mutex_);

    // 等待队列非空，保证获取的元素有效
    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

    // 返回队首元素
    return queue_.front();
  }

  /**
   * @brief 线程安全获取队尾元素：通过引用返回队尾元素，不修改队列状态
   * @param value 接收队尾元素的引用
   * @note 队列为空时仅打印错误信息，不阻塞等待（与front/pop的区别）
   * @note 适用于需要查看最新入队元素的场景
   */
  void back(T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    // 检查队列是否为空，为空则打印错误并返回
    if (queue_.empty()) {
      std::cerr << "Error: Attempt to access the back of an empty queue." << std::endl;
      return;
    }

    // 将队尾元素赋值给输出参数
    value = queue_.back();
  }

  /**
   * @brief 线程安全判断队列是否为空
   * @return bool 队列为空返回true，否则返回false
   * @note 生产/消费线程均可调用，加锁保证判断结果的原子性
   * @note 避免多线程下“判断为空”和“入队/出队”的竞态条件
   */
  bool empty()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  /**
   * @brief 线程安全清空队列：移除队列中所有元素
   * @note 加锁保护，保证清空操作的原子性
   * @note 清空后通知所有等待的消费者线程，避免线程永久阻塞
   */
  void clear()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    // 循环弹出所有元素，直到队列为空
    while (!queue_.empty()) {
      queue_.pop();
    }
    // 通知所有等待的消费者线程：队列已清空，避免线程因等待非空而永久阻塞
    not_empty_condition_.notify_all();
    // 1. not_empty_condition_ → 类的std::condition_variable（条件变量）成员，与消费者wait的是同一个对象，专门用于“队列非空”的同步通知，是生产者和消费者的“同步约定标识”
    // 2. notify_all() → std::condition_variable的核心通知方法，作用是“唤醒所有等待者”

  }

private:
  std::queue<T> queue_;                // 底层存储容器：std::queue实现FIFO基础功能，仅内部使用
  size_t max_size_;                    // 队列最大容量：限制队列元素数量，防止内存溢出
  mutable std::mutex mutex_;           // 互斥锁：保护所有队列操作，mutable表示const成员函数也可加锁
  std::condition_variable not_empty_condition_;  // 非空条件变量：实现消费者线程的阻塞等待，队列非空时唤醒
  std::function<void(void)> full_handler_;        // 满队列回调函数：队列满且PopWhenFull=false时执行，支持自定义逻辑（如日志、丢包统计）
};

}  // namespace tools

#endif  // TOOLS__THREAD_SAFE_QUEUE_HPP
