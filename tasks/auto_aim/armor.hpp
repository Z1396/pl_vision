#ifndef AUTO_AIM__ARMOR_HPP
#define AUTO_AIM__ARMOR_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace auto_aim
{
enum Color
{
  red,
  blue,
  extinguish,
  purple
};
const std::vector<std::string> COLORS = {"red", "blue", "extinguish", "purple"};

enum ArmorType
{
  big,
  small
};
const std::vector<std::string> ARMOR_TYPES = {"big", "small"};

enum ArmorName
{
  one,
  two,
  three,
  four,
  five,
  sentry,
  outpost,
  base,
  not_armor
};
const std::vector<std::string> ARMOR_NAMES = {"one",    "two",     "three", "four",     "five",
                                              "sentry", "outpost", "base",  "not_armor"};

enum ArmorPriority
{
  first = 1,
  second,
  third,
  forth,
  fifth
};

// clang-format off
const std::vector<std::tuple<Color, ArmorName, ArmorType>> armor_properties = {         //装甲板的属性
  {blue, sentry, small},     {red, sentry, small},     {extinguish, sentry, small},
  {blue, one, small},        {red, one, small},        {extinguish, one, small},
  {blue, two, small},        {red, two, small},        {extinguish, two, small},
  {blue, three, small},      {red, three, small},      {extinguish, three, small},
  {blue, four, small},       {red, four, small},       {extinguish, four, small},
  {blue, five, small},       {red, five, small},       {extinguish, five, small},
  {blue, outpost, small},    {red, outpost, small},    {extinguish, outpost, small},
  {blue, base, big},         {red, base, big},         {extinguish, base, big},      {purple, base, big},       
  {blue, base, small},       {red, base, small},       {extinguish, base, small},    {purple, base, small},    
  {blue, three, big},        {red, three, big},        {extinguish, three, big}, 
  {blue, four, big},         {red, four, big},         {extinguish, four, big},  
  {blue, five, big},         {red, five, big},         {extinguish, five, big}};
// clang-format on

/**
 * @brief 灯条结构体：存储单个装甲板灯条的几何特征、颜色、ID等信息
 * 作用：作为装甲板识别的基础单元，单个装甲板由左右两个灯条组成，先检测灯条再组合为装甲板
 */
struct Lightbar
{
  std::size_t id;                  // 灯条唯一标识ID（用于区分不同灯条，避免重复匹配）
  Color color;                     // 灯条颜色（如RED/BLUE，对应装甲板所属阵营，需自定义Color枚举）
  cv::Point2f center;              // 灯条中心坐标（图像平面2D坐标，单位：像素）
  cv::Point2f top;                 // 灯条顶端坐标（沿灯条长度方向的上端点）
  cv::Point2f bottom;              // 灯条底端坐标（沿灯条长度方向的下端点）
  cv::Point2f top2bottom;          // 灯条"顶端→底端"的向量（用于表示灯条方向，x/y分量为方向向量）
  std::vector<cv::Point2f> points; // 灯条轮廓点集合（存储灯条边缘的像素坐标，用于后续形状验证）
  double angle;                    // 灯条与图像水平轴（x轴）的夹角（单位：弧度/度，需看具体实现，用于判断灯条倾斜方向）
  double angle_error;              // 灯条角度误差（可能是检测角度与理论角度的偏差，用于筛选姿态合理的灯条）
  double length;                   // 灯条长度（单位：像素，沿top→bottom方向的距离）
  double width;                    // 灯条宽度（单位：像素，垂直于长度方向的最大距离）
  double ratio;                    // 灯条"长宽比"（length/width，装甲板灯条的长宽比通常固定，用于过滤非灯条干扰）
  cv::RotatedRect rotated_rect;    // 灯条的旋转矩形（OpenCV中用于表示倾斜矩形，包含中心、尺寸、旋转角，是灯条检测的核心输出）

  /**
   * @brief 灯条构造函数（核心）：由旋转矩形初始化灯条
   * @param rotated_rect 检测到的灯条旋转矩形（从图像中通过轮廓检测、形态学处理等得到）
   * @param id 灯条的唯一ID（由检测模块按顺序分配）
   */
  Lightbar(const cv::RotatedRect & rotated_rect, std::size_t id);
  
  /**
   * @brief 默认构造函数：创建空灯条对象（用于后续赋值，如数组初始化、临时变量）
   */
  Lightbar() {};
};

/**
 * @brief 装甲板结构体：存储单个装甲板的整体特征、姿态、坐标等信息
 * 作用：作为目标检测的最终结果，包含装甲板的几何特征、世界坐标、姿态角等，供后续跟踪、瞄准使用
 */
struct Armor
{
  Color color;                     // 装甲板颜色（与灯条颜色一致，RED/BLUE，标识阵营）
  Lightbar left, right;            // 装甲板的左、右灯条（装甲板的核心组成部分，从两个灯条推导装甲板整体信息）
  cv::Point2f center;              // 装甲板中心坐标（注意：注释明确"不是对角线交点"，可能是左右灯条中心连线的中点，图像2D坐标）
  cv::Point2f center_norm;         // 装甲板中心归一化坐标（将图像坐标转换为相机归一化平面坐标，用于后续3D位姿解算）
  std::vector<cv::Point2f> points; // 装甲板四个顶点坐标（图像2D坐标，顺时针/逆时针排序，用于透视变换、形状验证）

  double ratio;                    // 装甲板"灯条中点连线长度 / 灯条长度"的比值（装甲板固有比例，用于筛选合格装甲板，如标准装甲板该比值约为1.5）
  double side_ratio;               // 装甲板"长灯条长度 / 短灯条长度"的比值（正常装甲板左右灯条长度接近，该比值接近1，用于过滤灯条不匹配的干扰）
  double rectangular_error;        // 装甲板矩形度误差（灯条与灯条中点连线的夹角与90°的差值，装甲板理论上为矩形，该误差越小越合理）

  ArmorType type;                  // 装甲板类型（自定义枚举，如SMALL/LARGE，对应小装甲板/大装甲板，尺寸不同影响3D解算）
  ArmorName name;                  // 装甲板名称/编号（自定义枚举，如FRONT/LEFT/RIGHT/BACK，标识机器人上的位置，如"前装甲板"）
  ArmorPriority priority;          // 装甲板优先级（自定义枚举，如HIGH/MEDIUM/LOW，用于目标选择，如优先攻击前装甲板）
  int class_id;                    // 装甲板分类ID（可能是深度学习模型输出的类别ID，对应不同类型/位置的装甲板）
  cv::Rect box;                    // 装甲板的轴对齐矩形（不考虑旋转，仅包围装甲板的最小矩形，用于图像裁剪、显示）
  cv::Mat pattern;                 // 装甲板图案/ROI图像（可能是装甲板中心区域的截图，用于数字识别、图案匹配）
  double confidence;               // 装甲板检测置信度（0~1，用于判断检测结果的可靠性，置信度低的装甲板可能被过滤）
  bool duplicated;                 // 重复标记（是否为重复检测的装甲板，用于去重，避免同一装甲板被多次识别）

  // -------------------------- 3D坐标与姿态信息 --------------------------
  Eigen::Vector3d xyz_in_gimbal;   // 装甲板在"云台坐标系"下的3D坐标（单位：米，x/y/z轴对应云台的前后/左右/上下）
  Eigen::Vector3d xyz_in_world;    // 装甲板在"世界坐标系"下的3D坐标（单位：米，如东北天坐标系，用于全局定位）
  Eigen::Vector3d ypr_in_gimbal;   // 装甲板在"云台坐标系"下的姿态角（单位：弧度，yaw偏航/pitch俯仰/roll横滚）
  Eigen::Vector3d ypr_in_world;    // 装甲板在"世界坐标系"下的姿态角（单位：弧度，3行1列向量，用于姿态匹配、瞄准补偿）
  Eigen::Vector3d ypd_in_world;    // 装甲板在"世界坐标系"下的球坐标（yaw偏航/pitch俯仰/distance距离，简化瞄准计算）

  double yaw_raw;                  // 装甲板原始偏航角（单位：弧度，未经过优化的原始yaw值，用于对比优化前后的差异）

  /**
   * @brief 装甲板构造函数1（核心）：由左右两个灯条组合初始化装甲板
   * @param left 左灯条（已检测并初始化的Lightbar对象）
   * @param right 右灯条（已检测并初始化的Lightbar对象）
   * 逻辑：通过左右灯条的位置、角度、长度等信息，计算装甲板的中心、顶点、比例等特征
   */
  Armor(const Lightbar & left, const Lightbar & right);

  /**
   * @brief 装甲板构造函数2：由分类ID、置信度、包围盒、关键点初始化
   * @param class_id 分类ID（如深度学习输出的类别）
   * @param confidence 检测置信度
   * @param box 轴对齐包围盒
   * @param armor_keypoints 装甲板四个顶点关键点（用于计算中心、姿态）
   * 场景：可能用于深度学习检测直接输出装甲板关键点的场景
   */
  Armor(
    int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints);

  /**
   * @brief 装甲板构造函数3：带偏移量的初始化（构造函数2的重载）
   * @param offset 坐标偏移量（可能是图像裁剪后的偏移，用于还原原始图像坐标）
   * 场景：当装甲板ROI是从裁剪后的图像中检测时，需通过offset还原到原图坐标
   */
  Armor(
    int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints,
    cv::Point2f offset);

  /**
   * @brief 装甲板构造函数4：带颜色ID和编号ID的初始化
   * @param color_id 颜色ID（如0=RED，1=BLUE，用于映射到Color枚举）
   * @param num_id 编号ID（如0=FRONT，1=LEFT，用于映射到ArmorName枚举）
   * 场景：当检测模块输出颜色和编号的ID而非枚举时，用于转换并初始化
   */
  Armor(
    int color_id, int num_id, float confidence, const cv::Rect & box,
    std::vector<cv::Point2f> armor_keypoints);

  /**
   * @brief 装甲板构造函数5：带颜色ID、编号ID和偏移量的初始化（构造函数4的重载）
   * 场景：兼顾"颜色编号ID"和"图像坐标偏移"的场景
   */
  Armor(
    int color_id, int num_id, float confidence, const cv::Rect & box,
    std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset);
};


}  // namespace auto_aim

#endif  // AUTO_AIM__ARMOR_HPP