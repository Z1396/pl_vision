/**
 * @file detector.hpp
 * @brief 自动瞄准系统核心装甲板检测类头文件
 * @details 实现装甲板检测全流程：从BGR图像输入到合格装甲板输出，包含灯条提取、几何校验、颜色识别、类型分类、数字识别等核心逻辑，
 *          集成调试可视化、结果保存、异常过滤功能，是自动瞄准系统的**感知层核心模块**，为后续跟踪、解算模块提供有效装甲板目标。
 * @note 依赖OpenCV库（图像处理）、自定义Armor/Lightbar数据结构（装甲板/灯条模型）、Classifier分类器（数字/类型识别）
 * @note 支持调试模式（实时可视化检测过程、保存关键结果），所有检测结果需通过多层校验（几何/颜色/类型/名称）确保有效性
 */
#ifndef AUTO_AIM__DETECTOR_HPP  // 防止头文件重复包含（经典头文件保护宏，命名规范：模块名__类名_HPP）
#define AUTO_AIM__DETECTOR_HPP

// 标准库头文件：用于容器存储、字符串处理
#include <list>         // 存储检测到的灯条/装甲板（双向链表，适合频繁增删、遍历操作）
#include <string>       // 配置文件路径、结果保存路径
#include <vector>       // 存储轮廓点、图像像素等数据

// OpenCV核心头文件：图像处理、轮廓检测、几何计算、可视化
#include <opencv2/opencv.hpp>

// 项目自定义头文件：依赖的核心数据结构和分类器
#include "armor.hpp"    // 包含Armor（装甲板）、Lightbar（灯条）、ArmorType（装甲板类型）、Color（颜色）等核心数据结构
#include "classifier.hpp" // 装甲板数字/类型分类器（如YOLO/CNN/模板匹配实现，提供分类接口）

// 自动瞄准模块命名空间：隔离模块内代码，避免与项目其他模块（如控制、通信）命名冲突
namespace auto_aim
{

/**
 * @class Detector
 * @brief 自动瞄准系统装甲板检测核心类
 * @details 封装装甲板检测全流程逻辑，提供**批量检测**和**单目标检测**两种接口，内部实现"图像预处理→灯条提取→灯条配对→装甲板构建→多层校验→分类识别"的标准化检测流程，
 *          支持调试可视化、检测结果保存，所有检测参数通过配置文件加载，具备良好的可配置性和移植性。
 * @note 类内方法按**职责分层**：数据校正、几何校验、特征提取、分类识别、调试辅助，逻辑清晰便于维护
 * @note 友元类YOLOV8：允许YOLOV8类直接访问Detector的私有成员（适用于YOLOV8作为分类器时的深度集成）
 */
class Detector
{
public:
  /**
   * @brief 构造函数：初始化检测器，加载配置参数和分类器
   * @param config_path 配置文件路径（如yaml/json），包含所有检测阈值、几何参数、调试参数等
   * @param debug 是否启用调试模式，默认true
   * @details 1. 从配置文件加载检测阈值、几何约束参数、调试相关配置；2. 初始化分类器（classifier_）；3. 配置调试模式的保存路径/可视化开关
   * @note 配置文件是检测器的核心参数来源，需按指定格式编写，包含所有私有成员参数的配置项
   */
  Detector(const std::string & config_path, bool debug = true);

  /**
   * @brief 批量装甲板检测接口（核心对外接口）
   * @param bgr_img 输入图像，OpenCV BGR格式（相机采集的原始彩色图像，无需预处理）
   * @param frame_count 帧计数，默认-1（用于调试模式下的帧号标注、结果按帧保存）
   * @return std::list<Armor> 检测并通过所有校验的**合格装甲板列表**，按检测置信度/距离排序（具体由实现决定）
   * @details 执行完整装甲板检测流程，返回所有有效目标，适用于多目标识别、目标选择场景
   * @note 输入图像需为连续的BGR格式，避免非连续图像导致的内存访问错误
   */
  std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count = -1);

  /**
   * @brief 单装甲板检测接口（轻量对外接口）
   * @param armor 输出参数，检测到的**最优合格装甲板**（如最近、置信度最高的目标）
   * @param bgr_img 输入图像，OpenCV BGR格式
   * @return bool 检测成功返回true（找到有效装甲板），失败返回false（无有效目标）
   * @details 执行检测流程后，筛选出**单个最优有效装甲板**并赋值给输出参数，适用于单目标跟踪、快速瞄准场景
   * @note 若检测到多个有效目标，内部会通过预设规则（如距离最近、置信度最高）筛选出最优目标
   */
  bool detect(Armor & armor, const cv::Mat & bgr_img);

  /**
   * @brief 友元类声明：允许YOLOV8类访问Detector的所有私有成员
   * @details 适用于YOLOV8作为核心分类器/检测器时，与Detector深度集成，直接访问私有参数/方法，减少接口调用开销
   */
  friend class YOLOV8;

private:
  Classifier classifier_;  // 装甲板分类器：负责装甲板数字识别、类型分类（如大装甲板/小装甲板/哨兵装甲板）

  // ************************** 检测核心阈值参数 **************************
  double threshold_;               // 图像二值化阈值：用于灯条轮廓提取的像素灰度阈值
  double max_angle_error_;         // 灯条配对最大角度误差：左右灯条的角度偏差允许最大值（几何约束）
  double min_lightbar_ratio_;      // 灯条最小长宽比：过滤过宽/过短的无效灯条（几何约束）
  double max_lightbar_ratio_;      // 灯条最大长宽比：过滤过窄/过长的无效灯条（几何约束）
  double min_lightbar_length_;     // 灯条最小长度（像素单位）：过滤过小的无效灯条（几何约束，适配不同距离）
  double min_armor_ratio_;         // 装甲板最小长宽比：过滤过窄的无效装甲板（几何约束）
  double max_armor_ratio_;         // 装甲板最大长宽比：过滤过宽的无效装甲板（几何约束）
  double max_side_ratio_;          // 装甲板左右灯条边长比最大误差：过滤灯条不对称的无效装甲板（几何约束）
  double min_confidence_;          // 最小置信度阈值：分类器识别结果的置信度下限，过滤低置信度目标
  double max_rectangular_error_;   // 装甲板最大矩形度误差：衡量装甲板轮廓与矩形的相似度，过滤轮廓不规则目标

  // ************************** 调试相关参数 **************************
  bool debug_;          // 调试模式开关：true-启用可视化/结果保存，false-关闭所有调试功能（提升检测效率）
  std::string save_path_; // 调试结果保存路径：存储检测到的装甲板图像、特征图等（仅debug_=true时有效）

  /**
   * @brief 灯条角点PCA校正函数（核心校正方法）
   * @param lightbar 待校正的灯条对象（输入输出参数，校正后更新角点坐标）
   * @param gray_img 输入灰度图像：用于灯条区域的像素特征提取
   * @details 基于PCA（主成分分析）算法回归灯条真实角点，修正轮廓检测导致的角点偏移，提升装甲板构建的精度
   * @note 参考自开源项目CSU-FYT-Vision/FYT2024_vision，针对工业相机采集的图像做了适配优化
   * @const 方法不修改类的任何成员变量，仅修改输入的lightbar对象
   */
  void lightbar_points_corrector(Lightbar & lightbar, const cv::Mat & gray_img) const;

  /**
   * @brief 灯条几何特征校验
   * @param lightbar 待校验的灯条对象
   * @return bool 校验通过返回true，失败返回false
   * @details 校验灯条的长宽比、长度、角度等几何特征是否符合预设阈值（min_lightbar_ratio_/max_lightbar_ratio_/min_lightbar_length_等）
   * @note 灯条是装甲板的核心组成部分，几何校验是过滤无效目标的第一道屏障
   * @const 方法不修改类的任何成员变量，仅做只读校验
   */
  bool check_geometry(const Lightbar & lightbar) const;

  /**
   * @brief 装甲板几何特征校验
   * @param armor 待校验的装甲板对象
   * @return bool 校验通过返回true，失败返回false
   * @details 校验装甲板的长宽比、灯条对称性、角度偏差、矩形度等几何特征是否符合预设阈值，过滤几何特征异常的无效装甲板
   * @note 装甲板几何校验是过滤无效目标的核心步骤，直接决定检测结果的准确性
   * @const 方法不修改类的任何成员变量，仅做只读校验
   */
  bool check_geometry(const Armor & armor) const;

  /**
   * @brief 装甲板名称/数字有效性校验
   * @param armor 待校验的装甲板对象
   * @return bool 校验通过返回true，失败返回false
   * @details 校验分类器识别出的装甲板数字/名称是否为有效目标（如敌方装甲板数字、排除友方/无效数字）
   * @note 需结合项目的敌我识别规则实现，是目标有效性的重要校验环节
   * @const 方法不修改类的任何成员变量，仅做只读校验
   */
  bool check_name(const Armor & armor) const;

  /**
   * @brief 装甲板类型有效性校验
   * @param armor 待校验的装甲板对象
   * @return bool 校验通过返回true，失败返回false
   * @details 校验装甲板类型（如大装甲板/小装甲板/哨兵装甲板）是否为当前瞄准任务的有效类型，过滤无效类型目标
   * @note 适用于不同作战阶段的目标筛选（如远程仅打大装甲板、近程打小装甲板）
   * @const 方法不修改类的任何成员变量，仅做只读校验
   */
  bool check_type(const Armor & armor) const;

  /**
   * @brief 提取轮廓区域的主颜色（红/蓝/无）
   * @param bgr_img 输入BGR图像：原始彩色图像
   * @param contour 目标轮廓点集：灯条/装甲板的轮廓点
   * @return Color 轮廓区域的主颜色（项目自定义枚举，如RED/BLUE/NONE）
   * @details 通过计算轮廓包围盒内的像素颜色直方图/均值，判断主颜色为红色或蓝色，用于敌我识别、装甲板颜色标注
   * @const 方法不修改类的任何成员变量，仅做特征提取
   */
  Color get_color(const cv::Mat & bgr_img, const std::vector<cv::Point> & contour) const;

  /**
   * @brief 提取装甲板区域的特征图案（用于分类识别）
   * @param bgr_img 输入BGR图像：原始彩色图像
   * @param armor 目标装甲板对象：已构建的装甲板（包含位置、大小信息）
   * @return cv::Mat 装甲板特征图案：裁剪并预处理后的装甲板区域图像（如灰度化、归一化、抠图）
   * @details 从原始图像中裁剪出装甲板所在区域，进行灰度化、尺寸归一化、背景抠除等预处理，得到分类器所需的特征图案
   * @note 输出的特征图案格式需与分类器（classifier_）的输入要求完全匹配
   * @const 方法不修改类的任何成员变量，仅做特征提取
   */
  cv::Mat get_pattern(const cv::Mat & bgr_img, const Armor & armor) const;

  /**
   * @brief 识别装甲板类型（大/小/哨兵等）
   * @param armor 目标装甲板对象：已提取特征图案的装甲板
   * @return ArmorType 装甲板类型（项目自定义枚举，如BIG/SMALL/SENTINEL）
   * @details 调用分类器（classifier_）对装甲板特征图案进行分类，识别出装甲板的具体类型，赋值给armor的type成员
   */
  ArmorType get_type(const Armor & armor);

  /**
   * @brief 计算装甲板中心的归一化坐标
   * @param bgr_img 输入BGR图像：原始彩色图像（用于获取图像尺寸）
   * @param center 装甲板原始中心坐标（像素单位，基于输入图像）
   * @return cv::Point2f 归一化中心坐标：x/y范围为[0,1]，以图像左上角为原点
   * @details 将像素坐标转换为归一化坐标，消除图像尺寸对后续解算（如云台控制、距离解算）的影响
   * @const 方法不修改类的任何成员变量，仅做坐标转换
   */
  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;

  /**
   * @brief 保存装甲板检测结果（仅调试模式有效）
   * @param armor 待保存的装甲板对象：通过所有校验的有效装甲板
   * @details 将装甲板的特征图案、原始图像区域、检测参数（如中心坐标、类型、置信度）保存到指定路径（save_path_），支持按帧号/时间戳命名
   * @note 保存的结果可用于离线分析、分类器训练、检测效果调优
   * @const 方法不修改类的任何成员变量，仅做调试结果保存
   */
  void save(const Armor & armor) const;

  /**
   * @brief 实时可视化检测结果（仅调试模式有效）
   * @param binary_img 灯条提取后的二值化图像：用于展示灯条检测效果
   * @param bgr_img 原始BGR图像：用于叠加绘制检测结果
   * @param lightbars 检测到的所有灯条列表：用于绘制灯条轮廓
   * @param armors 检测到的所有合格装甲板列表：用于绘制装甲板框、中心、类型/数字等信息
   * @param frame_count 帧计数：用于窗口标注、帧号显示
   * @details 在原始图像和二值化图像上叠加绘制灯条轮廓、装甲板包围框、中心坐标、类型/数字/置信度等信息，通过OpenCV窗口实时显示
   * @note 可视化窗口支持按键操作（如退出、保存），具体由实现决定
   * @const 方法不修改类的任何成员变量，仅做调试可视化
   */
  void show_result(
    const cv::Mat & binary_img, const cv::Mat & bgr_img, const std::list<Lightbar> & lightbars,
    const std::list<Armor> & armors, int frame_count) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__DETECTOR_HPP
