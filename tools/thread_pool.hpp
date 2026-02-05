// 头文件保护宏：采用NAMESPACE__CLASS_HPP大写命名规则，防止重复包含导致的多重定义错误
#ifndef TOOLS__THREAD_POOL_HPP
#define TOOLS__THREAD_POOL_HPP

// 引入C++条件变量：用于多线程同步，实现线程等待/唤醒机制
#include <condition_variable>
// 引入C++函数包装器：用于封装任意可调用对象，作为线程池任务载体
#include <functional>
// 引入C++互斥锁：保证多线程下共享资源的原子读写，防止数据竞争
#include <mutex>
// 引入C++队列容器：作为有序队列主队列、线程池任务队列的底层容器
#include <queue>
// 引入C++线程库：实现工作线程创建、管理与.join()等待
#include <vector>
// 引入C++无序哈希表：作为有序队列的乱序帧缓冲容器，支持O(1)时间复杂度的帧查找/插入
#include <unordered_map>
// 引入C++列表容器：用于存储装甲板检测结果，适配频繁增删操作
#include <list>

// 引入OpenCV矩阵库：用于存储图像帧数据
#include <opencv2/opencv.hpp>
// 引入Eigen四元数库：用于存储帧对应的姿态信息（旋转量）
#include <Eigen/Geometry>
// 引入C++时间库：用于记录帧的采集时间戳，实现时序同步
#include <chrono>

// 引入自动瞄准模块YOLO检测器：用于批量创建检测器实例，适配多线程并行推理
#include "tasks/auto_aim/yolo.hpp"
// 引入项目日志工具：用于打印调试、警告、信息日志，便于问题排查
#include "tools/logger.hpp"

// 工具模块命名空间：隔离通用工具类/结构/函数，避免与业务模块（omniperception/auto_aim）命名冲突，
// 本模块提供多线程任务调度、帧数据有序管理、算法实例批量创建的通用能力，为上层业务模块提供基础支撑
namespace tools
{
/**
 * @brief 帧数据核心结构体
 * @details 封装机器视觉/自动瞄准场景下的**单帧完整数据**，包含图像、时序、姿态、检测结果等信息，
 *          是跨模块（采集→检测→决策→控制）的数据传输统一载体，所有字段均为业务核心所需数据。
 * @note 结构体为纯数据载体，无成员函数，字段命名语义明确，适配多线程下的批量处理与有序传输
 */
struct Frame
{
  int id;  ///< 帧唯一标识ID，要求严格单调递增，作为有序队列的排序依据
  cv::Mat img;  ///< 帧对应的图像数据（OpenCV矩阵），为检测/推理的原始输入
  std::chrono::steady_clock::time_point t;  ///< 帧的采集时间戳，高精度时序记录，用于多模块同步
  Eigen::Quaterniond q;  ///< 帧采集时的设备姿态四元数，表征相机/云台的旋转状态，用于后续坐标变换
  std::list<auto_aim::Armor> armors;  ///< 该帧的装甲板检测结果列表，存储检测到的所有装甲板信息
};

/**
 * @brief YOLO11检测器批量创建内联函数
 * @details 快速创建指定数量的YOLO11检测器实例，所有实例使用相同配置，适配多线程并行推理场景，
 *          每个线程绑定独立检测器实例，避免多线程共享资源导致的推理冲突/性能下降。
 * @param config_path YOLO模型配置文件路径，包含模型权重、检测阈值、输入尺寸等参数
 * @param numebr 待创建的检测器实例数量（参数名笔误：numebr → number），建议与线程池线程数一致
 * @param debug 是否启用调试模式，true=打印调试日志/可视化检测结果，false=生产模式（无额外输出，效率更高）
 * @return std::vector<auto_aim::YOLO> 检测器实例向量，每个元素为独立的YOLO11实例，可直接分配给工作线程
 * @note inline内联优化，高频调用无函数调用开销；YOLO11与YOLOv8接口一致，函数为兼容不同模型版本设计
 */
inline std::vector<auto_aim::YOLO> create_yolo11s(
  const std::string & config_path, int numebr, bool debug)
{
  std::vector<auto_aim::YOLO> yolo11s;
  for (int i = 0; i < numebr; i++) {
    yolo11s.push_back(auto_aim::YOLO(config_path, debug));
  }
  return yolo11s;
}

/**
 * @brief YOLOv8检测器批量创建内联函数
 * @details 与create_yolo11s功能一致，快速创建指定数量的YOLOv8检测器实例，
 *          适配多线程并行推理，保证每个线程拥有独立的检测器资源，无共享竞争。
 * @param config_path YOLO模型配置文件路径，与create_yolo11s通用
 * @param numebr 待创建的检测器实例数量（参数名笔误：numebr → number）
 * @param debug 是否启用调试模式
 * @return std::vector<auto_aim::YOLO> 检测器实例向量，独立实例，支持多线程并行调用
 * @note 与create_yolo11s为同构函数，仅函数名区分模型版本，底层均调用YOLO类构造函数
 */
inline std::vector<auto_aim::YOLO> create_yolov8s(
  const std::string & config_path, int numebr, bool debug)
{
  std::vector<auto_aim::YOLO> yolov8s;
  for (int i = 0; i < numebr; i++) {
    yolov8s.push_back(auto_aim::YOLO(config_path, debug));
  }
  return yolov8s;
}

/**
 * @brief 基于ID的有序帧队列类
 * @details 实现**乱序入队、有序出队**的帧数据管理，解决多线程并行处理后帧数据乱序的问题，
 *          以帧ID为排序依据（要求ID严格单调递增），保证出队的帧始终按ID从小到大顺序输出；
 *          内置主队列+缓冲哈希表，结合条件变量实现阻塞/非阻塞出队，线程安全，适配多生产单消费场景。
 * @note 1. 核心特性：乱序入队、有序出队、阻塞/非阻塞出队、过期帧过滤；
 *       2. 适用场景：多线程并行图像检测后，帧数据需要按采集顺序送入后续决策/控制模块；
 *       3. 线程模型：支持多生产端（并行推理线程）同时入队，单消费端（决策线程）出队（推荐）。
 */
class OrderedQueue
{
public:
  /**
   * @brief 构造函数：初始化有序队列核心状态
   * @details 初始化当前待出队的帧ID（起始为1），主队列、缓冲哈希表默认空初始化，
   *          互斥锁与条件变量由编译器默认构造，无需手动初始化。
   * @note 帧ID建议从1开始分配，连续且单调递增，避免0值与初始状态混淆
   */
  OrderedQueue() : current_id_(1) {}

  /**
   * @brief 析构函数：优雅释放队列资源
   * @details 加锁清空主队列、缓冲哈希表，重置当前待出队ID，打印析构日志，
   *          保证多线程下资源释放的原子性，避免析构时仍有线程操作队列导致的未定义行为。
   */
  ~OrderedQueue()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      main_queue_ = std::queue<tools::Frame>();  // 清空主队列
      buffer_.clear();                            // 清空乱序帧缓冲
      current_id_ = 0;                            // 重置当前待出队ID
    }
    tools::logger()->info("OrderedQueue destroyed, queue and buffer cleared.");
  }

  /**
   * @brief 帧数据入队接口（线程安全，支持多生产端同时调用）
   * @details 实现乱序入队的核心逻辑：根据帧ID判断是否为当前待出队帧，
   *          匹配则直接入主队列，不匹配则存入缓冲哈希表；主队列有数据时唤醒阻塞的消费端，
   *          同时自动检测缓冲中是否有后续连续ID的帧，若有则批量移入主队列，保证出队连续性。
   * @param item 待入队的帧数据（const引用，避免深拷贝）
   * @note 1. 过滤过期帧：若帧ID小于current_id_，判定为过期帧，打印警告并拒绝入队；
   *       2. 连续帧自动拼接：主队列添加当前帧后，自动遍历缓冲，将后续连续ID的帧批量移入主队列；
   *       3. 轻量锁：使用std::lock_guard，作用域结束自动解锁，适合短时间临界区操作。
   */
  void enqueue(const tools::Frame & item)
  {
    std::lock_guard<std::mutex> lock(mutex_);  // 加锁保证原子操作

    // 过滤过期帧：ID小于当前待出队ID，说明该帧已错过出队时机，拒绝入队
    if (item.id < current_id_) {
      tools::logger()->warn("small id: frame id {} is less than current id {}", item.id, current_id_);
      return;
    }

    // 帧ID匹配当前待出队ID：直接入主队列，准备出队
    if (item.id == current_id_) {
      main_queue_.push(item);
      current_id_++;  // 更新下一个待出队ID

      // 遍历缓冲，将后续连续ID的帧批量移入主队列，实现连续出队
      auto it = buffer_.find(current_id_);
      while (it != buffer_.end()) {
        main_queue_.push(it->second);  // 连续帧移入主队列
        buffer_.erase(it);             // 从缓冲中删除，释放内存
        current_id_++;                 // 更新待出队ID
        it = buffer_.find(current_id_);// 继续查找下一个连续帧
      }

      // 主队列有数据时，唤醒一个阻塞的消费端（非阻塞出队无需等待，不影响）
      if (main_queue_.size() >= 1) {
        cond_var_.notify_one();
      }
    } else {
      // 帧ID不匹配：存入缓冲哈希表，等待后续连续ID帧入队后拼接
      buffer_[item.id] = item;
    }
  }

  /**
   * @brief 阻塞式出队接口（单消费端推荐）
   * @details 从队列头部按ID顺序取出帧数据，若主队列为空则阻塞当前线程，
   *          直到主队列有数据被唤醒，保证始终能取出有效帧，无空数据返回。
   * @return tools::Frame 按ID排序的帧数据，出队后从主队列中删除
   * @note 1. 使用std::unique_lock配合条件变量，支持阻塞与唤醒；
   *       2. 等待条件为lambda表达式，防止虚假唤醒，保证唤醒后主队列非空；
   *       3. 仅单消费端调用，多消费端会导致帧顺序混乱。
   */
  tools::Frame dequeue()
  {
    std::unique_lock<std::mutex> lock(mutex_);  // 可解锁的互斥锁，适配条件变量

    // 阻塞等待：主队列为空时持续阻塞，直到被notify_one()唤醒且主队列非空
    cond_var_.wait(lock, [this]() { return !main_queue_.empty(); });

    // 取出主队列头部帧数据，出队并返回
    tools::Frame item = main_queue_.front();
    main_queue_.pop();
    return item;
  }

  /**
   * @brief 非阻塞式出队接口
   * @details 尝试从主队列取出帧数据，主队列为空时直接返回false，不阻塞当前线程，
   *          适合对实时性要求高、允许暂时无数据的消费场景。
   * @param item 输出参数：若出队成功，存储取出的帧数据；失败则保持原状态
   * @return bool 出队结果：true=成功（item有效），false=失败（主队列为空，item无效）
   * @note 使用std::lock_guard，轻量加锁，适合快速判断与出队
   */
  bool try_dequeue(tools::Frame & item)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (main_queue_.empty()) {
      return false;  // 主队列为空，非阻塞返回失败
    }
    // 出队成功，赋值并删除主队列头部数据
    item = main_queue_.front();
    main_queue_.pop();
    return true;
  }

  /**
   * @brief 获取队列总数据量
   * @details 计算主队列（待出队有序帧）+ 缓冲哈希表（乱序等待帧）的总帧数，
   *          反映队列当前的缓存压力，便于上层模块做流量控制。
   * @return size_t 队列总帧数，主队列大小 + 缓冲哈希表大小
   */
  size_t get_size() { return main_queue_.size() + buffer_.size(); }

private:
  std::queue<tools::Frame> main_queue_;  ///< 有序主队列，存储可直接出队的连续ID帧，按ID从小到大排序
  std::unordered_map<int, tools::Frame> buffer_;  ///< 乱序帧缓冲，存储ID不连续的帧，key=帧ID，value=帧数据，O(1)查找/插入
  int current_id_;  ///< 当前待出队的帧ID，仅当帧ID等于该值时可直接入主队列
  std::mutex mutex_;  ///< 队列全局互斥锁，保护所有共享资源（主队列、缓冲、current_id_）的原子操作
  std::condition_variable cond_var_;  ///< 条件变量，配合mutex_实现消费端的阻塞等待与唤醒
};

/**
 * @brief 通用线程池类
 * @details 实现**固定线程数的任务池化调度**，初始化时创建指定数量的工作线程，
 *          所有线程处于空闲等待状态，有任务时唤醒线程执行，任务完成后回到等待状态；
 *          支持任意可调用对象（函数、lambda、绑定器）作为任务，线程安全的任务入队，
 *          析构时优雅停止所有线程，释放资源，无内存泄漏与僵尸线程。
 * @note 1. 核心特性：固定工作线程、任务池化、任意可调用对象、优雅退出、线程安全；
 *       2. 适用场景：通用的多线程任务调度，如多相机并行推理、批量数据处理、耗时操作异步执行；
 *       3. 线程模型：多生产端（任意线程）入队任务，多消费端（工作线程）执行任务，任务执行顺序无序。
 */
class ThreadPool
{
public:
  /**
   * @brief 构造函数：初始化线程池并创建工作线程
   * @details 根据指定线程数创建工作线程，所有线程进入无限循环的空闲等待状态，
   *          等待任务队列有任务时被唤醒执行，初始化时任务队列为空，停止标志为false。
   * @param num_threads 工作线程数量，建议根据CPU核心数设置（如CPU核心数*2），避免线程过多导致的调度开销
   * @note 工作线程通过emplace_back直接创建，lambda捕获当前对象引用，实现任务队列的访问
   */
  ThreadPool(size_t num_threads) : stop(false)
  {
    for (size_t i = 0; i < num_threads; ++i) {
      // 创建工作线程，执行无限循环的任务消费逻辑
      workers.emplace_back([this] {
        while (true) {
          std::function<void()> task;  // 任务包装器，存储待执行的任务
          {
            std::unique_lock<std::mutex> lock(queue_mutex);  // 可解锁互斥锁，适配条件变量
            // 阻塞等待：任务队列为空且未停止时，持续阻塞，直到有任务或收到停止指令
            condition.wait(lock, [this] { return stop || !tasks.empty(); });
            // 停止指令+任务队列为空：退出循环，线程结束
            if (stop && tasks.empty()) {
              return;
            }
            // 取出任务队列头部任务，移动语义转移资源，避免深拷贝
            task = std::move(tasks.front());
            tasks.pop();
          }  // 临界区结束，解锁，任务执行过程中不占用锁，提升并发效率
          task();  // 执行任务，任意可调用对象直接调用
        }
      });
    }
  }

  /**
   * @brief 析构函数：优雅停止线程池并释放所有资源
   * @details 加锁设置停止标志，清空任务队列，唤醒所有等待的工作线程，
   *          等待所有工作线程正常结束后回收资源，保证无僵尸线程、无内存泄漏，
   *          是多线程类的标准优雅退出实现。
   */
  ~ThreadPool()
  {
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      stop = true;  // 设置停止标志，通知所有工作线程退出
      tasks = std::queue<std::function<void()>>();  // 清空任务队列，丢弃未执行的任务
    }
    condition.notify_all();  // 唤醒所有等待的工作线程，确保线程能检测到停止标志
    // 等待所有工作线程正常结束，回收线程资源
    for (std::thread & worker : workers) {
      if (worker.joinable()) {  // 判空检查，避免对已结束的线程执行join导致异常
        worker.join();
      }
    }
  }

  /**
   * @brief 模板化任务入队接口（线程安全，支持任意可调用对象）
   * @details 将任意可调用对象（函数、lambda、绑定器、函数对象）封装为std::function<void()>，
   *          加入任务队列，入队后唤醒一个空闲的工作线程执行任务，线程安全，支持多生产端同时调用。
   * @tparam F 可调用对象的类型，模板自动推导，无需手动指定
   * @param f 待执行的任务，支持左值、右值引用，通过std::forward实现完美转发，避免拷贝开销
   * @throw std::runtime_error 若线程池已停止（stop=true），抛出运行时异常，禁止向已停止的线程池入队任务
   * @note 1. 完美转发：保留原始参数的左值/右值属性，提升性能；
   *       2. 轻量唤醒：入队后仅唤醒一个工作线程，避免惊群效应；
   *       3. 模板化设计：支持任意无返回值的可调用对象，通用性强。
   */
  template <class F>
  void enqueue(F && f)
  {
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      // 检查线程池状态，已停止则抛出异常
      if (stop) {
        throw std::runtime_error("enqueue on stopped ThreadPool");
      }
      // 完美转发，将可调用对象封装为任务，加入任务队列
      tasks.emplace(std::forward<F>(f));
    }  // 临界区结束，解锁
    condition.notify_one();  // 唤醒一个空闲的工作线程执行任务
  }

private:
  std::vector<std::thread> workers;         ///< 工作线程容器，存储所有创建的工作线程句柄，用于析构时join等待
  std::queue<std::function<void()>> tasks;  ///< 任务队列，存储待执行的任务，元素为std::function<void()>，封装任意可调用对象
  std::mutex queue_mutex;                   ///< 任务队列互斥锁，保护任务队列的原子入队/出队，防止数据竞争
  std::condition_variable condition;        ///< 条件变量，配合queue_mutex实现工作线程的阻塞等待与唤醒
  bool stop;                                ///< 线程池停止标志，true=停止，所有工作线程退出循环并结束
};
}  // namespace tools

#endif  // TOOLS__THREAD_POOL_HPP：头文件保护宏结束，与开头#ifndef配对
