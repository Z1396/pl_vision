// 引入当前模块头文件，包含Recorder类的声明
#include "recorder.hpp"

// 引入fmt库的时间格式化模块，用于生成带时间戳的文件名（格式化系统时间/稳态时间）
#include <fmt/chrono.h>

// 引入C++17文件系统库，用于创建记录目录（跨平台，替代传统mkdir）
/*总结（核心要点）
  核心定位不同：<fstream> 是文件内容读写库，<filesystem> 是文件 / 目录系统管理库，二者功能互补，无替代关系；
  操作对象不同：<fstream> 操作文件内部的内容，<filesystem> 操作文件 / 目录本身（路径、结构、属性）；
  标准要求不同：<fstream> 支持 C++98+，无需额外配置；<filesystem> 必须 C++17+，编译时需指定标准；
  核心搭配逻辑：<filesystem> 做前置管理（创目录、拼路径、判存在），<fstream> 做后续读写（打开文件、写 / 读内容、关文件）；
  功能边界清晰：<fstream> 不能创目录，<filesystem> 不能写内容，切勿混淆；
  工程必用组合：在你的Recorder类、自瞄项目及所有现代 C++ 工程中，二者配合使用是完成文件操作的标准方案，兼顾易用性、跨平台性和工程规范性。*/
#include <filesystem>
// 引入字符串库，用于路径和文本拼接
#include <string>

// 引入自定义数学工具库，包含时间差计算等工具函数
#include "math_tools.hpp"
// 引入项目日志工具，用于输出调试/信息日志
#include "tools/logger.hpp"

// 工具类命名空间，隔离项目内工具类，避免命名冲突
namespace tools
{
/**
 * @brief 视频与姿态数据同步记录工具类
 * @note 实现**图像帧**与**四元数姿态**的同步保存，分为视频文件(.avi)和文本文件(.txt)
 * @note 采用**生产者-消费者模型**：record接口（生产者）入队数据，后台线程（消费者）异步写入文件
 * @note 基于线程安全队列实现数据传递，保证多线程调用的线程安全，避免文件IO阻塞业务线程
 * @note 支持指定帧率录制，自动做帧间隔过滤，保证录制帧率稳定
 */
/*这种写法效果看似一致，但底层实现和性能有本质差异，初始化列表的3 个核心优势是工程中必须使用的原因：
优势 1：避免 “默认构造 + 赋值” 的二次操作，提升性能
对于非基本类型的成员变量（如queue_（ThreadSafeQueue）、std::string、cv::VideoWriter等），如果不用初始化列表：

    第一步：编译器会先调用该变量的无参默认构造函数，创建一个空对象；
    第二步：再在构造函数体内执行赋值操作，将新值赋给这个空对象。

而用初始化列表：

    直接调用该变量的带参构造函数，一步完成对象创建和初始化，跳过默认构造步骤，减少不必要的性能开销（尤其对于复杂对象，性能提升明显）。

比如你的queue_(1)：直接调用 ThreadSafeQueue 的带参构造函数（传入容量 1），一步创建队列；而体内赋值会先默认构造一个空队列，再赋值为新的队列对象，浪费资源。

优势 2：必须用于初始化 “无法赋值” 的成员变量
C++ 中有部分成员变量支持初始化，但不支持赋值，只能通过初始化列表初始化，否则编译报错，典型包括：

    const 常量成员变量（如const int MAX_FRAME = 100;）；
    引用成员变量（如int& ref_;）；
    无无参默认构造函数的自定义类成员变量（如你的 ThreadSafeQueue 若未写无参构造，就必须用初始化列表传参）。

简单说：初始化列表是这类变量唯一的初始化方式，没有替代方案。

优势 3：保证成员变量的初始化顺序，避免逻辑错误
C++ 规定：成员变量的初始化顺序，由其在类中声明的顺序决定，而非初始化列表中的书写顺序。但使用初始化列表能显式体现初始化逻辑，让代码可读性更强，同时强制开发者关注声明顺序，避免因初始化顺序错误导致的逻辑问题（如用未初始化的变量初始化另一个变量）。
比如 Recorder 类中，queue_ 先声明，stop_thread_ 后声明，无论初始化列表中书写顺序如何，都会先初始化queue_，再初始化stop_thread_，初始化列表让这个过程更清晰。*/
Recorder::Recorder(double fps) 
  : init_(false),          // 初始化标记：未完成视频/文本写入器初始化
    fps_(fps),             // 目标录制帧率（过滤过短间隔的帧，保证帧率稳定）
    queue_(1),             // 线程安全队列，容量1（仅保留最新帧，避免数据积压）
    stop_thread_(false)    // 后台保存线程的原子退出标志（无锁线程安全）
{
  // 记录录制开始的稳态时间（单调时钟，不受系统时间修改影响，用于计算相对时间）
  start_time_ = std::chrono::steady_clock::now();
  // 记录上一帧的时间戳，用于帧间隔过滤
  last_time_ = start_time_;

  // 定义录制文件的存储目录和文件名
  const auto folder_path = "records";  // 固定记录目录名
  // 生成带当前系统时间的文件名（年-月-日_时-分-秒），避免文件重名
  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  // 拼接文本文件完整路径（records/时间戳.txt），存储姿态数据
  text_path_ = fmt::format("{}/{}.txt", folder_path, file_name);
  // 拼接视频文件完整路径（records/时间戳.avi），存储图像帧数据
  video_path_ = fmt::format("{}/{}.avi", folder_path, file_name);

  // 跨平台创建记录目录：若目录已存在，无操作；不存在则创建，无需判断目录状态
  std::filesystem::create_directory(folder_path);
}

/**
 * @brief 析构函数：优雅释放资源，停止后台保存线程，关闭文件写入器
 * @note 遵循RAII原则，自动完成资源清理，避免文件句柄泄漏、线程僵尸
 * @note 处理后台线程阻塞问题，保证线程正常退出
 */
Recorder::~Recorder()
{
  /*五、关键核心：为什么用 std::atomic<bool> 而不是普通 bool？
  这是该代码的线程安全核心，也是多线程编程的关键避坑点：普通 bool 变量的赋值 / 读取不是原子操作，多线程下会出现数据竞争和未定义行为，而 std::atomic<bool> 能保证：

  写操作原子性：stop_thread_ = true; 是一次性完成的操作，CPU 不会将其拆分为 “读 - 改 - 写” 步骤，其他线程不会读到 “中间态”；
  读操作原子性：后台线程的 !stop_thread_ 判断，能读到最新的、完整的值（要么是 false，要么是 true）；
  无锁高效：无需搭配 std::mutex 加锁解锁，操作开销接近普通 bool，不会影响后台线程的执行效率（适合高频循环判断）。

  反例：用普通 bool 会出什么问题？
  如果将 stop_thread_ 定义为普通 bool：

  bool stop_thread_ = false;  // 错误：普通bool，非线程安全

  多线程下可能出现编译器优化或CPU 指令重排，导致后台线程永远读不到 true，一直卡在循环中无法退出（死循环），这是多线程编程中典型的 “可见性问题”。*/
  stop_thread_ = true;

  // 2. 向队列推入空帧：解决后台线程在queue_.pop()处的永久阻塞问题
  //    若队列无数据，pop会一直等待，推入空帧可唤醒线程并触发退出判断
  queue_.push({cv::Mat::zeros(0, 0, 0), {0, 0, 0, 0}, std::chrono::steady_clock::now()});
  // 3. 等待后台保存线程执行完毕：若线程可连接（已启动），则阻塞等待退出
  if (saving_thread_.joinable()) 
  {
    saving_thread_.join();
  }

  // 4. 若未完成初始化（未调用过init），直接返回，避免空操作
  if (!init_) 
  {
    return;
  }
  // 5. 关闭文本文件写入器，刷新缓冲区，保证所有数据写入磁盘
  text_writer_.close();
  // 6. 释放视频写入器资源，关闭视频文件，完成编码收尾（如AVI文件的索引写入）
  video_writer_.release();
}

/**
 * @brief 后台保存线程的核心处理函数（消费者）：从队列取数据，同步写入视频和文本文件
 * @note 由std::thread启动，独立线程执行，避免文件IO阻塞生产者线程（如相机采集、图像处理线程）
 * @note 空帧为退出信号，接收到空帧则跳过并继续判断退出标志
 */
/*核心执行特性：线程创建后，save_to_file与init()/record()是并行执行的，主业务线程无需等待后台线程执行完毕，实现真正的 “异步解耦”*/
void Recorder::save_to_file()
{
  // 循环读取队列数据，直到收到退出标志
  while (!stop_thread_) {
    FrameData frame;          // 定义帧数据对象，接收队列中的数据
    queue_.pop(frame);        // 从线程安全队列取出数据：队列为空时阻塞等待，有数据则返回

    // 判读是否为空白帧（退出唤醒帧），若是则跳过本次循环
    if (frame.img.empty()) {
      tools::logger()->debug("Recorder received empty img. Skip this frame.");
      continue;
    }

    // 1. 将图像帧写入视频文件：由OpenCV视频写入器完成编码和写入
    video_writer_.write(frame.img);

    // 2. 将姿态四元数+相对时间写入文本文件，实现视频与姿态的时间同步
    //    Eigen四元数coeffs()默认返回xyzw顺序，项目需要wxyz顺序，因此做索引调整
    Eigen::Vector4d xyzw = frame.q.coeffs();  // 获取四元数原始系数 [x, y, z, w]
    // 计算当前帧相对于录制开始时间的时间差（秒），用于时间戳同步
    auto since_begin = tools::delta_time(frame.timestamp, start_time_);
    // 格式化写入文本：相对时间 + w + x + y + z（按wxyz顺序，便于后续解析）
    text_writer_ << fmt::format(
      "{} {} {} {} {}\n", since_begin, xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
  }
}

/**
 * @brief 录制接口（生产者）：外部调用添加图像帧和姿态数据，线程安全
 * @param img 待录制的图像帧（cv::Mat），为空则直接返回，不处理
 * @param q 待录制的姿态四元数（Eigen::Quaterniond），与图像帧时间同步
 * @param timestamp 帧的时间戳（稳态时钟），用于帧率过滤和相对时间计算
 * @note 支持多线程调用（内部通过线程安全队列保证线程安全）
 * @note 自动做帧率过滤，保证实际录制帧率不超过设定的fps_
 * @note 懒加载初始化：首次调用时完成文件写入器和后台线程的初始化
 */
void Recorder::record(
  const cv::Mat & img, 
  const Eigen::Quaterniond & q,
  const std::chrono::steady_clock::time_point & timestamp)
{
  // 空帧直接返回，不做任何处理
  if (img.empty()) return;
  // 懒加载：首次调用record时，完成视频/文本写入器初始化和后台线程启动
  if (!init_) init(img);

  // 帧率过滤：计算当前帧与上一帧的时间差（秒），若小于目标帧间隔则跳过
  // 目标帧间隔 = 1 / 目标帧率，避免录制帧率过高（如相机30帧，录制10帧则每3帧取1帧）
  auto since_last = tools::delta_time(timestamp, last_time_);
  if (since_last < 1.0 / fps_) return;

  // 更新上一帧时间戳为当前帧时间戳
  last_time_ = timestamp;
  // 将图像、四元数、时间戳封装为FrameData，推入线程安全队列
  // 入队后立即返回，不阻塞，由后台线程异步处理写入
  queue_.push({img, q, timestamp});
}

/**
 * @brief 初始化函数：完成视频写入器、文本写入器初始化，启动后台保存线程
 * @param img 参考图像帧：用于获取视频的宽、高、通道数，确定视频编码分辨率
 * @note 仅被record接口首次调用，懒加载执行，避免无录制时的资源浪费
 * @note 初始化完成后设置init_为true，标记初始化完成
 */
void Recorder::init(const cv::Mat & img)
{
  // 1. 打开文本文件写入器：以默认模式（覆盖写）打开，若文件不存在则创建
  text_writer_.open(text_path_);
  // 2. 定义视频编码格式：MJPG（Motion JPEG），兼容性强、编码速度快，适合实时录制
  auto fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  // 3. 初始化OpenCV视频写入器
  //    参数：视频路径、编码格式、目标帧率、图像尺寸（与输入帧一致）
  video_writer_ = cv::VideoWriter(video_path_, fourcc, fps_, img.size());
  // 4. 启动后台保存线程：绑定Recorder::save_to_file成员函数作为线程入口
  /*注意：取地址符&可以省略（编译器会自动推导），即std::thread(Recorder::save_to_file, this)写法也合法，工程中两种写法均常用。*/
  saving_thread_ = std::thread(&Recorder::save_to_file, this);
  // 左值：saving_thread_ → Recorder类中声明的std::thread类型成员变量（空线程对象，未启动）
  // 赋值符：= → 将新建的已启动线程对象，赋值给类的线程成员变量
  // 右值：std::thread(...) → 构造一个临时的std::thread对象，同时**立即启动后台线程**
  // 构造参数1：&Recorder::save_to_file → Recorder类的save_to_file成员函数的**地址**
  // 构造参数2：this → Recorder类的**当前对象指针**（必传，类成员函数的隐含参数）

  // 5. 设置初始化完成标记，避免重复初始化
  init_ = true;
}

}  // namespace tools
