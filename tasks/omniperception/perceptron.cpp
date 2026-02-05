// 引入感知器类头文件，包含类声明、成员函数原型、私有成员及依赖的自定义类型（如DetectionResult）
#include "perceptron.hpp"

// 引入C++时间库，用于时间戳记录、线程休眠、延时初始化
#include <chrono>
// 引入C++智能指针库，用于YOLO检测器的动态管理与多线程安全共享
#include <memory>
// 引入C++线程库，实现多相机并行推理线程的创建与管理
#include <thread>

// 引入自动瞄准模块YOLO检测器头文件，用于装甲板目标检测
#include "tasks/auto_aim/yolo.hpp"
// 引入项目工具模块：程序退出工具（优雅终止）、日志工具（打印各等级日志）
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

// 全景感知模块命名空间，隔离模块内代码，避免与其他模块命名冲突
namespace omniperception
{

/**
 * @brief 全景感知器构造函数
 * @details 实现感知器核心初始化逻辑：初始化线程安全检测队列、决策器、退出标志，
 *          创建4个独立的YOLO检测器实例，启动4个并行推理线程对接4路USB相机，
 *          是全景感知系统的入口初始化函数，完成多硬件/多算法/多线程的协同初始化
 * @param usbcam1~usbcam4 4路USB相机指针，分别对接不同视野的相机硬件，支持空指针（内部做判空）
 * @param config_path YAML配置文件路径，传递给YOLO检测器和决策器，实现统一配置加载
 * @note 初始化列表完成核心成员初始化：检测队列（容量10）、决策器、退出标志（默认false），符合C++高效初始化规范
 */
Perceptron::Perceptron(
  io::USBCamera * usbcam1, io::USBCamera * usbcam2, io::USBCamera * usbcam3,
  io::USBCamera * usbcam4, const std::string & config_path)
: detection_queue_(10), decider_(config_path), stop_flag_(false)
{
  // 初始化4个独立的YOLO检测器智能指针实例，避免多线程共享同一检测器导致的资源竞争
  // 第二个参数false表示不启用调试模式，适配生产环境的高效推理
  yolo_parallel1_ = std::make_shared<auto_aim::YOLO>(config_path, false);
  yolo_parallel2_ = std::make_shared<auto_aim::YOLO>(config_path, false);
  yolo_parallel3_ = std::make_shared<auto_aim::YOLO>(config_path, false);
  yolo_parallel4_ = std::make_shared<auto_aim::YOLO>(config_path, false);

  // 延时2秒：等待YOLO模型权重加载、相机硬件初始化完成，避免启动阶段的资源未就绪问题
  std::this_thread::sleep_for(std::chrono::seconds(2));
  
  // 创建4个并行推理线程，每个线程绑定一路相机和一个独立YOLO检测器，实现多相机并行采集+推理
  // 采用lambda表达式传递参数，捕获当前对象引用（&），线程函数为类成员parallel_infer
  threads_.emplace_back([&] { parallel_infer(usbcam1, yolo_parallel1_); });
  threads_.emplace_back([&] { parallel_infer(usbcam2, yolo_parallel2_); });
  threads_.emplace_back([&] { parallel_infer(usbcam3, yolo_parallel3_); });
  threads_.emplace_back([&] { parallel_infer(usbcam4, yolo_parallel4_); });

  // 打印初始化完成日志，标识全景感知系统启动成功
  tools::logger()->info("Perceptron initialized.");
}

/**
 * @brief 全景感知器析构函数
 * @details 实现感知器资源的**优雅释放**：设置退出标志、唤醒所有等待线程、等待线程正常结束，
 *          避免线程僵尸、资源泄漏、程序崩溃，是C++多线程类的标准析构实现规范
 * @note 采用「互斥锁保护退出标志+条件变量唤醒+线程join等待」的三步释放策略，保证多线程安全退出
 */
Perceptron::~Perceptron()
{
  {
    // 加锁修改退出标志：保证多线程下stop_flag_的修改原子性，避免数据竞争
    std::unique_lock<std::mutex> lock(mutex_);
    stop_flag_ = true;  // 设置全局退出标志，通知所有并行推理线程终止循环
  }
  condition_.notify_all();  // 唤醒所有因等待条件变量而阻塞的线程，确保线程能检测到退出标志

  // 遍历所有推理线程，等待线程正常执行完毕后回收资源
  for (auto & t : threads_) {
    if (t.joinable()) {  // 判空检查：避免对已结束/未启动的线程执行join导致异常
      t.join();
    }
  }
  // 打印析构完成日志，标识全景感知系统资源释放成功
  tools::logger()->info("Perceptron destructed.");
}

/**
 * @brief 检测结果队列获取接口
 * @details 从**线程安全的检测队列**中批量取出所有检测结果，转换为普通向量返回，
 *          实现感知器（生产端）与上层模块（消费端，如决策/控制模块）的结果交互，
 *          本接口为消费端唯一的结果获取入口
 * @return std::vector<DetectionResult> 批量检测结果，包含所有相机的装甲板检测、角度解算结果
 * @note 1. 队列pop操作**非阻塞**：队列为空时直接返回空向量，不阻塞调用线程；
 *       2. 采用std::move转移资源：避免检测结果的深拷贝，提升数据传递效率；
 *       3. 批量取出：减少队列操作次数，提升生产消费的整体效率
 */
std::vector<DetectionResult> Perceptron::get_detection_queue()
{
  // 存储最终返回的批量检测结果
  std::vector<DetectionResult> result;
  // 临时变量，用于接收队列pop出的单个检测结果
  DetectionResult temp;

  // 循环取出队列中所有检测结果，队列为空时终止循环
  while (!detection_queue_.empty()) {
    detection_queue_.pop(temp);          // 非阻塞pop，取出单个结果
    result.push_back(std::move(temp));   // 移动语义转移资源，避免深拷贝
  }

  return result;
}

/**
 * @brief 多相机并行推理核心成员函数
 * @details 单相机的**采集-检测-解算-入队**全流程逻辑，为每个相机独立的线程函数，
 *          实现4路相机的并行化处理，大幅提升全景感知的整体检测帧率和实时性，
 *          是本类的核心业务实现函数
 * @param cam 单路USB相机指针，为当前线程绑定的采集硬件，支持空指针（内部做判空处理）
 * @param yolov8_parallel 与当前线程绑定的YOLO检测器智能指针，独立实例避免多线程资源竞争
 * @note 1. 每个线程对应**一个相机+一个独立YOLO检测器**，无资源共享，无需额外线程同步；
 *       2. 包含完善的异常处理和容错机制，单个相机/检测器异常不影响其他线程运行；
 *       3. 所有检测结果通过**线程安全队列**入队，保证多线程生产数据的安全性
 */
void Perceptron::parallel_infer(
  io::USBCamera * cam, std::shared_ptr<auto_aim::YOLO> & yolov8_parallel)
{
  // 相机指针判空：避免空指针解引用导致程序崩溃，打印错误日志并直接返回
  if (!cam) {
    tools::logger()->error("Camera pointer is null!");
    return;
  }

  // 全局异常捕获：捕获线程执行过程中的所有异常，避免单个线程异常导致整个程序崩溃
  try {
    // 无限循环执行采集-检测逻辑，直到检测到退出标志stop_flag_为true
    while (true) {
      // 存储相机采集的图像帧和对应的时间戳
      cv::Mat usb_img;
      std::chrono::steady_clock::time_point ts;

      {
        // 加锁检查退出标志：保证stop_flag_的读取原子性，避免数据竞争
        std::unique_lock<std::mutex> lock(mutex_);
        if (stop_flag_) break;  // 检测到退出标志，立即终止循环，线程准备退出
      }

      // 从绑定相机采集一帧图像，获取图像数据和采集时间戳
      cam->read(usb_img, ts);
      // 采集容错：图像为空（相机断开/采集超时）时，休眠30ms后继续下一次采集，不终止线程
      if (usb_img.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        continue;
      }

      // 调用绑定的YOLO检测器，对采集图像进行装甲板目标检测，返回检测到的装甲板列表
      auto armors = yolov8_parallel->detect(usb_img);
      // 仅当检测到有效装甲板时，才进行后续角度解算和结果入队，减少无效计算
      if (!armors.empty()) {
        // 调用决策器的角度解算函数，根据装甲板结果和相机名称解算云台角度增量（度）
        auto delta_angle = decider_.delta_angle(armors, cam->device_name);

        // 构造检测结果对象，存储本次检测的所有有效信息
        DetectionResult dr;
        dr.armors = std::move(armors);    // 移动语义转移装甲板列表，避免深拷贝
        dr.timestamp = ts;                // 记录图像采集时间戳，用于时序同步
        dr.delta_yaw = delta_angle[0] / 57.3;  // 偏航角增量：度转换为弧度（云台控制协议为弧度制）
        dr.delta_pitch = delta_angle[1] / 57.3;// 俯仰角增量：度转换为弧度，57.3≈180/π

        // 将检测结果推入**线程安全队列**，供上层模块通过get_detection_queue接口获取
        detection_queue_.push(dr);
      }
    }
  } catch (const std::exception & e) {
    // 捕获并打印线程执行过程中的所有标准异常，便于问题排查，单个线程异常不影响其他线程
    tools::logger()->error("Exception in parallel_infer: {}", e.what());
  }
}

}  // namespace omniperception
