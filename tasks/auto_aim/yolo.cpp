// 引入YOLO类的头文件声明，保证实现与声明的一致性
#include "yolo.hpp"

// YAML配置文件解析库：用于加载并解析外部YAML配置文件，读取YOLO版本、模型参数等配置项
#include <yaml-cpp/yaml.h>

// 引入各版本YOLO的具体实现类头文件：均为YOLOBase抽象基类的派生类，实现各自的检测逻辑
#include "yolos/yolo11.hpp"
#include "yolos/yolov5.hpp"
#include "yolos/yolov8.hpp"

// 自动瞄准模块命名空间：与头文件保持一致，隔离模块内代码
namespace auto_aim
{

/**
 * @brief YOLO类构造函数：核心实现**基于配置的多态对象创建**
 * @param config_path YAML配置文件路径，包含yolo_name（YOLO版本）、模型路径、检测阈值等核心参数
 * @param debug 调试模式开关，透传给具体YOLO实现类，控制日志打印、中间结果保存等调试行为
 * @details 核心逻辑：解析配置文件获取YOLO版本 → 根据版本动态创建对应派生类实例 → 赋值给基类智能指针实现多态，
 *          是**工厂模式**的典型落地，将对象创建逻辑封装，上层无需关心具体实现类的实例化细节
 * @throw std::runtime_error 当配置文件中指定的yolo_name为未知版本时，抛出运行时异常，明确错误原因
 */
YOLO::YOLO(const std::string & config_path, bool debug)
{
  // 加载并解析YAML配置文件，返回根节点对象，后续通过键值对读取配置项
  auto yaml = YAML::LoadFile(config_path);
  // 从配置文件根节点读取"yolo_name"字段，转换为字符串类型，该字段指定要使用的YOLO版本（如yolov5/yolov8/yolo11）
  auto yolo_name = yaml["yolo_name"].as<std::string>();

  // 根据YOLO版本，动态创建对应派生类的实例，通过std::make_unique管理生命周期
  if (yolo_name == "yolov8") 
  {
    // 创建YOLOv8实例，传入配置路径和调试开关，返回std::unique_ptr<YOLOV8>
    yolo_ = std::make_unique<YOLOV8>(config_path, debug);
  }
  else if (yolo_name == "yolo11") 
  {
    // 创建YOLO11实例，逻辑同YOLOv8
    yolo_ = std::make_unique<YOLO11>(config_path, debug);
  }
  else if (yolo_name == "yolov5") 
  {
    // 创建YOLOv5实例，std::make_unique自动分配内存并构造对象，无需手动new
    yolo_ = std::make_unique<YOLOV5>(config_path, debug);    
    /* 关键语法与设计说明：
       1. 类型兼容：YOLOV5/YOLOV8/YOLO11均是YOLOBase的公有派生类，满足"is-a"关系；
       2. 向上转型：std::unique_ptr<派生类> 可**隐式转换**为 std::unique_ptr<基类>，这是C++多态的核心特性之一，
          转换后基类智能指针可指向派生类对象，后续调用虚函数时会触发动态绑定；
       3. 智能指针优势：std::make_unique相比手动new+unique_ptr，更安全（避免内存泄漏）、更高效（减少一次内存分配），
          且能自动管理对象生命周期，YOLO对象销毁时，yolo_会自动释放指向的派生类实例；
       4. debug参数透传：将调试开关传递给具体实现类，由派生类实现调试逻辑，如：
          - 输出模型加载日志（权重路径、层结构、推理设备）；
          - 保存图像预处理/推理中间结果（如缩放后的图像、模型输出特征图）；
          - 打印检测关键信息（推理耗时、检测框数量、置信度分布）；
          - 绘制检测结果可视化图（在图像上绘制装甲板框、类别、置信度）。
    */
  }
  // 处理未知YOLO版本的异常情况
  else 
  {
    // 抛出运行时异常，拼接错误信息（未知的YOLO版本），让上层捕获并处理，避免程序隐式崩溃
    throw std::runtime_error("Unknown yolo name: " + yolo_name + "!");
  }
}

/**
 * @brief 检测接口实现：**纯虚函数转发调用**
 * @param img 输入检测图像（cv::Mat格式，BGR通道），为相机采集的原始/预处理图像
 * @param frame_count 帧计数，透传给具体实现类，用于帧同步、日志记录、多帧跟踪关联
 * @return std::list<Armor> 检测到的有效装甲板链表，直接对接上层跟踪/决策模块
 * @details 核心逻辑：通过基类智能指针yolo_，调用指向的派生类对象的detect纯虚函数，
 *          自身不实现任何检测逻辑，仅作为**接口转发器**，实现上层与具体实现的解耦
 * @note 动态绑定：由于detect是YOLOBase的纯虚函数，派生类均重写了该函数，
 *       实际调用的是当前yolo_指向的派生类（如YOLOV5/YOLOV8）的detect实现，这是多态的核心体现
 */
std::list<Armor> YOLO::detect(const cv::Mat & img, int frame_count)
{
  // 转发调用基类指针指向的派生类detect实现，返回装甲板检测结果
  return yolo_->detect(img, frame_count);
}

/**
 * @brief 后处理接口实现：**纯虚函数转发调用**
 * @param scale 图像预处理缩放比例，用于将模型输出的归一化坐标映射回原始图像坐标
 * @param output 模型推理原始输出（cv::Mat格式），包含检测框、置信度、类别ID等原始数据
 * @param bgr_img 原始BGR格式图像，用于坐标反推、装甲板颜色/形状二次验证
 * @param frame_count 帧计数，透传给具体实现类，用于日志记录
 * @return std::list<Armor> 后处理后的有效装甲板链表，剔除误检、完成坐标映射
 * @details 核心逻辑与detect接口一致，仅转发调用派生类的postprocess纯虚函数，
 *          后处理的具体逻辑（如NMS非极大值抑制、坐标反推、装甲板有效性验证）由各派生类实现
 * @note 该接口主要供底层检测器（如MultiThreadDetector）在自定义检测流程时调用，
 *       上层业务模块一般优先使用detect整体接口（包含预处理+推理+后处理）
 */
std::list<Armor> YOLO::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  // 转发调用基类指针指向的派生类postprocess实现，返回后处理后的装甲板结果
  return yolo_->postprocess(scale, output, bgr_img, frame_count);
}

}  // namespace auto_aim
