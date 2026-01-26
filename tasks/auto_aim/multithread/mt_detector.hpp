#ifndef AUTO_AIM__MT_DETECTOR_HPP
#define AUTO_AIM__MT_DETECTOR_HPP

#include <chrono>       // 用于时间戳管理（图像采集时间）
#include <opencv2/opencv.hpp>  // OpenCV库，用于图像存储和处理
#include <openvino/openvino.hpp>  // OpenVINO库，用于加载和运行深度学习模型（如YOLO）
#include <tuple>        // 用于返回多个值（图像、装甲板列表、时间戳）

// 自瞄YOLO检测器头文件（YOLOv5实现）
#include "tasks/auto_aim/yolos/yolov5.hpp"
// 工具类头文件：日志、线程安全队列
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

namespace auto_aim
{
namespace multithread
{

/**
 * 多线程装甲板检测器类（MultiThreadDetector）
 * 功能：基于OpenVINO和YOLO模型，在多线程中异步处理图像，实现高效的装甲板检测
 * 核心设计：图像推送线程与模型推理线程分离，避免主线程阻塞
 */
class MultiThreadDetector
{
public:
  /**
   * 构造函数
   * @param config_path 配置文件路径，用于加载模型路径、设备类型、检测参数等
   * @param debug 是否启用调试模式（如输出调试日志、保留原始图像用于可视化）
   */
  MultiThreadDetector(const std::string & config_path, bool debug = true);

  /**
   * 推送图像到检测队列
   * 供图像采集线程（如主线程）调用，将原始图像和时间戳加入待检测队列
   * @param img 待检测的原始图像（CV_8UC3格式，BGR通道）
   * @param t 图像采集的时间戳（用于后续时间同步，如与IMU数据对齐）
   */
  void push(cv::Mat img, std::chrono::steady_clock::time_point t);

  /**
   * 从检测结果队列弹出检测结果（非调试版）
   * 供追踪线程调用，获取检测到的装甲板列表和对应时间戳
   * @return 元组：<装甲板列表, 图像时间戳>
   * @note 暂时不支持YOLOv8，仅适配YOLOv5
   */
  std::tuple<std::list<Armor>, std::chrono::steady_clock::time_point> pop();  

  /**
   * 从检测结果队列弹出检测结果（调试版）
   * 供调试或可视化线程调用，额外返回原始图像（用于绘制检测框、显示结果）
   * @return 元组：<原始图像, 装甲板列表, 图像时间戳>
   */
  std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point> debug_pop();

private:
  ov::Core core_;  // OpenVINO核心对象，用于管理模型和设备
  ov::CompiledModel compiled_model_;  // 编译后的模型对象（绑定到指定硬件设备）
  std::string device_;  // 模型运行的硬件设备（如"CPU"、"GPU"、"MYRIAD"）
  YOLO yolo_;  // YOLO检测器对象（此处为YOLOv5实现，负责模型推理和结果解析）
  //Detector detector;

  /**
   * 线程安全队列：存储待推理的图像、时间戳和推理请求
   * 队列元素类型：<原始图像, 时间戳, OpenVINO推理请求>
   * 队列容量16，满时触发回调函数输出调试日志
   */
  tools::ThreadSafeQueue<
    std::tuple<cv::Mat, std::chrono::steady_clock::time_point, ov::InferRequest>>
    queue_{16, [] { tools::logger()->debug("[MultiThreadDetector] queue is full!"); }};
};

}  // namespace multithread
}  // namespace auto_aim

#endif  // AUTO_AIM__MT_DETECTOR_HPP