#ifndef AUTO_AIM__YOLO_HPP
#define AUTO_AIM__YOLO_HPP

// 头文件保护：防止重复包含（唯一标识与文件名/模块名对应，避免编译冲突）
// OpenCV核心头文件：提供图像存储(cv::Mat)、图像处理、矩阵运算能力，是视觉检测的基础
#include <opencv2/opencv.hpp>

// 自动瞄准模块装甲板头文件：引入装甲板核心类Armor，包含装甲板属性、枚举等定义
#include "armor.hpp"

// 自动瞄准模块命名空间：隔离模块内代码，避免与全向感知/打符等其他模块命名冲突
namespace auto_aim
{

/**
 * @brief YOLO检测器抽象基类（纯虚类）
 * @details 该类是**所有YOLO版本装甲板检测器的统一抽象接口**，采用**模板方法模式**的设计思想，
 *          定义了YOLO检测的核心流程骨架（前向推理+后处理），将具体的实现细节交给子类（如YOLOv5/YOLOv8/YOLOv9），
 *          实现“接口与实现分离”，让上层模块无需关心具体YOLO版本，仅依赖统一接口调用。
 * @note 设计特点：1. 纯虚类设计，包含两个纯虚函数，无法实例化，仅作为子类的基类；
 *                2. 统一检测流程：detect（整体检测）、postprocess（推理结果后处理），所有子类需实现该流程；
 *                3. 与装甲板类型强绑定，返回值为std::list<Armor>，直接对接上层跟踪/决策模块。
 */
class YOLOBase
{
public:
  /**
   * @brief YOLO检测核心接口（纯虚函数）
   * @param img 输入图像：cv::Mat类型（BGR格式，与OpenCV默认读取格式一致），待检测的相机采集图像
   * @param frame_count 帧计数：int类型，当前检测帧的序号（用于帧同步、调试日志、结果记录）
   * @return std::list<Armor> 检测到的装甲板链表：存储当前帧中所有检测到的有效装甲板对象
   * @details 定义YOLO检测器的**整体检测流程**：包含图像预处理（缩放/归一化/通道转换）、
   *          模型前向推理、推理结果后处理三大核心步骤，子类需实现具体的推理与处理逻辑。
   * @note 纯虚函数：=0表示无默认实现，所有子类必须重写该函数，否则子类也为抽象类无法实例化。
   */
  virtual std::list<Armor> detect(const cv::Mat & img, int frame_count) = 0;

  /**
   * @brief YOLO推理结果后处理接口（纯虚函数）
   * @param scale 图像缩放比例：double类型，预处理时图像的缩放系数（用于将推理结果的像素坐标映射回原始图像）
   * @param output 模型推理输出：cv::Mat类型，存储YOLO模型的原始推理结果（如边界框、置信度、类别ID）
   * @param bgr_img 原始输入图像：cv::Mat类型（BGR格式），用于坐标映射、装甲板特征二次验证（如颜色/形状）
   * @param frame_count 帧计数：int类型，当前帧序号，用于日志打印与帧同步
   * @return std::list<Armor> 后处理后的有效装甲板链表：剔除误检、完成坐标映射、构造Armor对象
   * @details 定义YOLO检测的**后处理标准流程**：包含推理结果解析（边界框/置信度/类别）、
   *          非极大值抑制（NMS）去重、像素坐标反推原始图像、装甲板有效性验证（颜色/长宽比/面积）、
   *          Armor对象构造，是从“模型输出”到“可用装甲板结果”的关键步骤，子类需根据自身模型输出格式实现。
   */
  virtual std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) = 0;
};

/**
 * @brief YOLO检测器封装类（业务层调用入口）
 * @details 该类是**上层模块（检测器/决策器）访问YOLO检测功能的唯一入口**，采用**桥接模式**（也叫Pimpl惯用法），
 *          将具体的YOLO实现（YOLOv5/YOLOv8等）封装在抽象基类YOLOBase中，自身仅保留基类指针，
 *          实现“编译期隔离”与“版本灵活切换”，上层模块无需修改代码即可替换不同YOLO版本的检测器。
 * @note 设计特点：1. 封装实现细节，对外暴露简洁统一的接口，符合“最小接口原则”；
 *                2. 通过std::unique_ptr管理基类对象，实现资源自动释放，避免内存泄漏；
 *                3. 构造函数从配置文件加载参数，自动初始化对应版本的YOLO子类实例；
 *                4. 提供带默认参数的detect接口，简化上层调用。
 */
class YOLO
{
public:
  /**
   * @brief 构造函数：初始化YOLO检测器，加载配置并创建具体YOLO实现实例
   * @param config_path YAML配置文件路径：包含YOLO核心参数（模型路径、置信度阈值、NMS阈值、
   *                    图像输入尺寸、YOLO版本、调试模式参数、装甲板检测参数等）
   * @param debug 调试模式开关：bool类型（带默认值true），开启后会保留推理中间结果、
   *              绘制检测框/置信度，便于开发调试；关闭后仅输出核心检测结果，提升运行效率
   * @details 核心逻辑：1. 解析配置文件，读取YOLO版本（如v5/v8）、模型路径等参数；
   *                2. 根据YOLO版本动态创建对应的YOLOBase子类实例（如YOLOv5Impl/YOLOv8Impl）；
   *                3. 将子类实例赋值给内部的std::unique_ptr<YOLOBase>指针，完成初始化；
   *                4. 初始化调试模式相关参数，为后续检测/后处理提供配置支撑。
   */
  YOLO(const std::string & config_path, bool debug = true);

  /**
   * @brief 对外暴露的YOLO检测接口（业务层调用）
   * @param img 输入图像：cv::Mat类型（BGR格式），相机采集的原始图像/预处理后的图像
   * @param frame_count 帧计数：int类型（带默认值-1），当前检测帧的序号；默认值-1表示无需帧计数，
   *                    上层模块可根据需求传入（如多帧同步、日志记录）
   * @return std::list<Armor> 检测到的有效装甲板链表：直接对接跟踪器/决策器，无需额外类型转换
   * @details 核心逻辑：**转发调用**内部YOLOBase子类实例的detect纯虚函数，自身不实现具体检测逻辑，
   *          仅作为“接口转发器”，实现对上层的透明调用（上层无需关心具体YOLO版本）。
   * @note 设计优势：上层模块仅需调用该接口，当替换YOLO版本时，仅需修改配置文件，无需改动调用代码。
   */
  std::list<Armor> detect(const cv::Mat & img, int frame_count = -1);

  /**
   * @brief 对外暴露的YOLO后处理接口（可选，供底层/调试使用）
   * @param scale 图像缩放比例：double类型，预处理的缩放系数，用于坐标反推
   * @param output 模型推理输出：cv::Mat类型，原始模型输出结果
   * @param bgr_img 原始输入图像：cv::Mat类型（BGR格式），用于坐标映射与装甲板验证
   * @param frame_count 帧计数：int类型，当前帧序号
   * @return std::list<Armor> 后处理后的有效装甲板链表
   * @details 核心逻辑：**转发调用**内部YOLOBase子类实例的postprocess纯虚函数，
   *          主要供底层检测器模块（如MultiThreadDetector）在自定义检测流程时调用，
   *          上层业务模块一般无需直接调用，优先使用detect整体接口。
   */
  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count);

private:
  /**
   * @brief YOLO抽象基类智能指针：桥接具体YOLO实现
   * @details 1. 采用std::unique_ptr独占式智能指针，管理YOLOBase子类实例的生命周期，
   *            实现资源自动释放，避免手动new/delete导致的内存泄漏；
   *          2. 指向具体的YOLO子类实例（如YOLOv5Impl/YOLOv8Impl），由构造函数根据配置文件动态创建；
   *          3. 私有化该成员，避免上层模块直接操作底层实现，保证封装性；
   *          4. 是“接口与实现分离”的核心，通过该指针实现对不同YOLO版本的统一调用。
   */
  std::unique_ptr<YOLOBase> yolo_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLO_HPP
