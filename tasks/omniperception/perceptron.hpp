// 头文件保护宏：防止因重复包含导致的类/类型多重定义错误
// 命名规则：NAMESPACE__CLASS_HPP 大写格式，符合C++工程化规范，避免宏名冲突
#ifndef OMNIPERCEPTION__PERCEPTRON_HPP
#define OMNIPERCEPTION__PERCEPTRON_HPP

// 引入C++时间库：用于图像采集时间戳记录、线程时序控制
#include <chrono>
// 引入C++列表容器：适配装甲板检测结果的频繁增删操作（间接依赖）
#include <list>
// 引入C++智能指针库：用于YOLO检测器的多线程安全管理与资源自动释放
#include <memory>

// 引入本模块核心依赖：决策器类（角度解算、装甲板过滤）
#include "decider.hpp"
// 引入检测结果数据结构：封装单相机单次检测的所有有效信息
#include "detection.hpp"
// 引入IO层USB相机类：对接硬件相机，实现图像采集
#include "io/usbcamera/usbcamera.hpp"
// 引入自动瞄准模块装甲板定义：包含Armor/Color/ArmorName等核心数据结构
#include "tasks/auto_aim/armor.hpp"
// 引入项目工具：线程池（预留扩展）、线程安全队列（多线程结果缓冲）
#include "tools/thread_pool.hpp"
#include "tools/thread_safe_queue.hpp"

// 全景感知模块命名空间：隔离模块内代码，避免全局命名空间污染，
// 本模块为系统核心感知层，负责多相机并行采集、目标检测、结果封装
namespace omniperception
{

/**
 * @brief 全景感知核心类
 * @details 实现**四路USB相机的并行化图像采集、YOLO装甲板目标检测、角度解算**，
 *          是全景感知系统的入口类，承担「硬件采集-算法检测-结果分发」的核心枢纽作用；
 *          采用「一相机一线程一检测器」的完全解耦并行架构，大幅提升系统整体检测帧率；
 *          通过线程安全队列实现检测结果的多线程安全存储，为上层决策/控制模块提供统一结果入口。
 * @note 1. 核心能力：多相机并行感知、线程安全结果管理、资源优雅释放；
 *       2. 模块联动：对接IO层相机硬件、auto_aim层YOLO检测器、本模块Decider决策器；
 *       3. 线程模型：手动管理多线程（非线程池），实现相机与检测器的强绑定，无资源竞争。
 */
class Perceptron
{
public:
  /**
   * @brief 构造函数：初始化全景感知系统核心资源
   * @details 完成线程安全队列、决策器、退出标志的初始化，创建四路独立YOLO检测器实例，
   *          启动四路并行推理线程，分别绑定四路USB相机，实现多硬件/多算法/多线程的协同初始化。
   * @param usbcma1~usbcam4 四路USB相机指针（第一个参数存在笔误：usbcma1→usbcam1），
   *        支持空指针，内部做判空处理，空指针对应线程直接退出，不影响其他相机；
   * @param config_path YAML配置文件路径，统一传递给YOLO检测器和Decider决策器，
   *        实现配置集中管理，包含相机参数、检测阈值、解算参数等。
   * @note 初始化后会延时等待模型加载和相机就绪，再启动并行线程。
   */
  Perceptron(
    io::USBCamera * usbcma1, io::USBCamera * usbcam2, io::USBCamera * usbcam3,
    io::USBCamera * usbcam4, const std::string & config_path);

  /**
   * @brief 析构函数：优雅释放所有资源
   * @details 实现多线程环境下的资源安全释放：设置退出标志、唤醒所有阻塞线程、
   *          等待所有并行推理线程正常结束，避免线程僵尸、资源泄漏、空指针解引用。
   * @note 采用「互斥锁+退出标志+条件变量」的标准多线程退出策略，保证原子性和安全性。
   */
  ~Perceptron();

  /**
   * @brief 检测结果获取接口（上层模块唯一入口）
   * @details 从线程安全队列中**批量非阻塞取出所有检测结果**，转换为普通向量返回，
   *          实现感知层（生产端）与上层决策/控制层（消费端）的结果解耦，
   *          本接口为线程安全，可被多消费端调用（实际建议单消费端）。
   * @return std::vector<DetectionResult> 批量检测结果，包含所有有效相机的
   *         装甲板列表、采集时间戳、云台角度增量（弧度制），队列为空时返回空向量。
   * @note 采用移动语义转移数据，避免深拷贝，提升数据传递效率。
   */
  std::vector<DetectionResult> get_detection_queue();

  /**
   * @brief 单相机并行推理核心接口（线程函数）
   * @details 为每个相机独立的线程执行函数，实现「图像采集→YOLO检测→角度解算→结果入队」
   *          全流程逻辑，每个线程绑定唯一相机和唯一YOLO检测器，无资源共享，无需额外同步。
   * @param cam 绑定的USB相机指针，不能为空（内部做判空），负责当前线程的图像采集；
   * @param yolo_parallel 绑定的YOLO检测器智能指针，为独立实例，避免多线程资源竞争，
   *        负责当前线程的装甲板目标检测。
   * @note 包含完善的异常处理和容错机制，图像采集失败/检测异常时不终止线程，仅休眠重试。
   */
  void parallel_infer(io::USBCamera * cam, std::shared_ptr<auto_aim::YOLO> & yolo_parallel);

private:
  /**
   * @brief 并行推理线程容器
   * @details 存储四路并行推理线程的句柄，用于析构时统一管理、等待线程结束，
   *          线程数量与相机数量一一对应，容器大小固定为4。
   */
  std::vector<std::thread> threads_;

  /**
   * @brief 线程安全检测结果队列
   * @details 多线程生产-消费的核心缓冲，存储所有相机的有效检测结果（DetectionResult），
   *          保证四路推理线程同时入队、上层模块出队时的线程安全，避免数据竞争；
   *          初始化时指定固定容量，防止结果堆积导致内存溢出。
   */
  tools::ThreadSafeQueue<DetectionResult> detection_queue_;

  /**
   * @brief 四路独立YOLO检测器智能指针
   * @details 每路相机对应一个独立的YOLO检测器实例，采用std::shared_ptr实现资源自动管理，
   *          完全解耦多线程的算法资源，避免多线程共享检测器导致的推理冲突、性能下降，
   *          所有检测器通过同一配置文件初始化，检测参数保持一致。
   */
  std::shared_ptr<auto_aim::YOLO> yolo_parallel1_;
  std::shared_ptr<auto_aim::YOLO> yolo_parallel2_;
  std::shared_ptr<auto_aim::YOLO> yolo_parallel3_;
  std::shared_ptr<auto_aim::YOLO> yolo_parallel4_;

  /**
   * @brief 决策器实例
   * @details 内部封装角度解算、装甲板过滤核心逻辑，为所有并行线程共享，
   *          其暴露的delta_angle等接口为只读操作，无需额外线程同步。
   */
  Decider decider_;

  /**
   * @brief 全局退出标志
   * @details 控制所有并行推理线程的退出，析构时置为true，通知所有线程终止循环，
   *          为多线程共享变量，需通过互斥锁保证读写原子性。
   */
  bool stop_flag_;

  /**
   * @brief 全局互斥锁
   * @details 保护共享变量（如stop_flag_）的原子读写，防止多线程数据竞争；
   *          mutable修饰表示：即使在const成员函数中，也可加锁/解锁，不破坏常量正确性。
   */
  mutable std::mutex mutex_;

  /**
   * @brief 条件变量
   * @details 配合互斥锁使用，用于唤醒因等待条件而阻塞的线程，
   *          保证析构时能及时唤醒所有线程，避免线程永久阻塞导致无法退出。
   */
  std::condition_variable condition_;
};

}  // namespace omniperception

#endif  // OMNIPERCEPTION__PERCEPTRON_HPP：头文件保护宏结束，与开头#ifndef配对
