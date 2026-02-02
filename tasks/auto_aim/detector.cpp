#include "detector.hpp"

// 第三方库：格式化输出（时间、字符串）、YAML配置文件解析
#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

// 标准库：文件系统操作（创建目录）
#include <filesystem>

// 项目工具库：图像处理工具、日志打印
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

// 自动瞄准模块命名空间，隔离模块内代码
namespace auto_aim
{
/**
 * @brief 构造函数：初始化检测器，加载配置参数并初始化分类器
 * @param config_path YAML配置文件路径，包含所有检测阈值、几何约束参数
 * @param debug 是否启用调试模式（可视化+结果保存），默认true
 * @details 1. 初始化分类器（传入配置文件路径，加载模型/参数）；2. 加载YAML配置文件解析所有检测参数；
 *          3. 初始化调试结果保存路径，自动创建保存目录；4. 角度/弧度单位转换（配置文件为角度，内部用弧度计算）
 */
Detector::Detector(const std::string & config_path, bool debug)
: classifier_(config_path), debug_(debug)  // 初始化列表：先初始化分类器和调试开关
{
  // 加载YAML配置文件，若文件不存在/格式错误会抛出异常（需上层捕获）
  auto yaml = YAML::LoadFile(config_path);

  // 加载基础检测阈值（从配置文件解析，赋值给类私有成员）
  threshold_ = yaml["threshold"].as<double>();                     // 图像二值化阈值：分割灯条高亮区域与背景
  max_angle_error_ = yaml["max_angle_error"].as<double>() / 57.3;  // 灯条角度误差上限：角度转弧度（180/π≈57.3）
  min_lightbar_ratio_ = yaml["min_lightbar_ratio"].as<double>();   // 灯条最小长宽比：过滤过宽的无效灯条
  max_lightbar_ratio_ = yaml["max_lightbar_ratio"].as<double>();   // 灯条最大长宽比：过滤过窄的无效灯条
  min_lightbar_length_ = yaml["min_lightbar_length"].as<double>(); // 灯条最小像素长度：过滤小光斑/噪点
  min_armor_ratio_ = yaml["min_armor_ratio"].as<double>();         // 装甲板最小长宽比：过滤过窄的无效装甲板
  max_armor_ratio_ = yaml["max_armor_ratio"].as<double>();         // 装甲板最大长宽比：过滤过宽的无效装甲板
  max_side_ratio_ = yaml["max_side_ratio"].as<double>();           // 装甲板左右灯条长度比最大误差：保证灯条对称
  min_confidence_ = yaml["min_confidence"].as<double>();           // 分类器最小置信度：过滤低置信度识别结果
  max_rectangular_error_ = yaml["max_rectangular_error"].as<double>() / 57.3;  // 装甲板矩形度误差上限：角度转弧度

  // 初始化调试模式下的装甲板图案保存路径
  save_path_ = "patterns";
  // 自动创建保存目录，若已存在则无操作（避免目录不存在导致保存失败）
  std::filesystem::create_directory(save_path_);
}

/**
 * @brief 批量装甲板检测核心接口：从原始BGR图像中检测所有有效装甲板
 * @param bgr_img 输入原始图像（OpenCV BGR格式，相机直接采集）
 * @param frame_count 帧计数（用于调试可视化的帧号标注，默认-1）
 * @return std::list<Armor> 所有通过几何/分类/类型校验的有效装甲板列表
 * @details 执行完整检测流程：图像预处理→灯条提取→灯条排序→灯条配对→装甲板构建→多层校验→去重→调试可视化
 *          核心流程：灰度化→二值化→轮廓检测→灯条筛选→灯条配对→装甲板校验→分类识别→去重
 */
std::list<Armor> Detector::detect(const cv::Mat & bgr_img, int frame_count)
{
  // 步骤1：图像预处理 - 彩色图转灰度图（减少计算量，便于后续二值化）
  cv::Mat gray_img;
  cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);

  // 步骤2：图像预处理 - 二值化（分割灯条高亮区域与背景，生成黑白图像）
  cv::Mat binary_img;
  // THRESH_BINARY：像素值>threshold_设为255（白），否则0（黑），精准分割灯条
  cv::threshold(gray_img, binary_img, threshold_, 255, cv::THRESH_BINARY);
  //cv::imshow("binary_img", binary_img); // 调试用：显示二值化图像，默认注释

  // 步骤3：轮廓检测 - 提取二值化图像中的所有外部轮廓（灯条候选区域）
  std::vector<std::vector<cv::Point>> contours;
  // RETR_EXTERNAL：仅提取最外层轮廓；CHAIN_APPROX_NONE：保存轮廓所有点（保证精度）
  cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

  // 步骤4：灯条提取与筛选 - 从轮廓中筛选符合几何特征的有效灯条
  std::size_t lightbar_id = 0;  // 灯条唯一ID（用于后续装甲板去重，标记灯条归属）
  std::list<Lightbar> lightbars; // 存储有效灯条（list便于后续排序和遍历）
  for (const auto & contour : contours) 
  {
    // 对每个轮廓拟合最小外接旋转矩形（适配任意角度的灯条）
    auto rotated_rect = cv::minAreaRect(contour);
    // 构造灯条对象（传入旋转矩形和唯一ID，内部自动计算长宽比、长度、角度误差等参数）
    auto lightbar = Lightbar(rotated_rect, lightbar_id);

    // 几何特征校验：过滤长宽比/长度/角度误差不符合要求的无效灯条
    if (!check_geometry(lightbar)) continue;

    // 识别灯条颜色（红/蓝，用于敌我识别和灯条配对过滤）
    lightbar.color = get_color(bgr_img, contour);
    // 将有效灯条加入列表
    lightbars.emplace_back(lightbar);
    lightbar_id += 1; // 灯条ID自增，保证唯一性
  }

  // 步骤5：灯条排序 - 按灯条中心x坐标从左到右排序（便于后续左右灯条配对，减少无效遍历）
  lightbars.sort([](const Lightbar & a, const Lightbar & b) { return a.center.x < b.center.x; });

  // 步骤6：灯条配对与装甲板构建 - 左右灯条配对，构建装甲板并进行多层校验
  std::list<Armor> armors; // 存储候选装甲板
  // 双层遍历：左灯条为基准，依次与右侧灯条配对（已排序，避免重复配对）
  for (auto left = lightbars.begin(); left != lightbars.end(); left++) {
    for (auto right = std::next(left); right != lightbars.end(); right++) {
      // 颜色过滤：左右灯条颜色必须一致（排除不同颜色灯条错误配对）
      if (left->color != right->color) continue;

      // 构造装甲板对象（传入左右灯条，内部自动计算装甲板长宽比、中心、矩形度等参数）
      auto armor = Armor(*left, *right);
      // 装甲板几何特征校验：过滤灯条不对称、矩形度差的无效装甲板
      if (!check_geometry(armor)) continue;

      // 提取装甲板特征图案（裁剪装甲板区域图像，用于分类器识别数字/类型）
      armor.pattern = get_pattern(bgr_img, armor);
      // 分类器识别：对特征图案进行数字识别，赋值armor.name和armor.confidence
      classifier_.classify(armor);
      // 名称/置信度校验：过滤非装甲板、低置信度的识别结果
      if (!check_name(armor)) continue;

      // 判定装甲板类型（大/小装甲板，基于长宽比+数字名称综合判断）
      armor.type = get_type(armor);
      // 类型校验：过滤类型与数字不匹配的异常装甲板（如小装甲板识别出1号）
      if (!check_type(armor)) continue;

      // 计算装甲板中心归一化坐标（消除图像尺寸影响，便于后续云台解算）
      armor.center_norm = get_center_norm(bgr_img, armor.center);
      // 将所有校验通过的有效装甲板加入列表
      armors.emplace_back(armor);
    }
  }

  // 步骤7：装甲板去重 - 过滤共用灯条的重复装甲板/重叠装甲板
  for (auto armor1 = armors.begin(); armor1 != armors.end(); armor1++) {
    for (auto armor2 = std::next(armor1); armor2 != armors.end(); armor2++) {
      // 检查是否共用灯条：四个灯条ID完全不同则无共用，跳过
      if (
        armor1->left.id != armor2->left.id && armor1->left.id != armor2->right.id &&
        armor1->right.id != armor2->left.id && armor1->right.id != armor2->right.id) {
        continue;
      }

      // 情况1：装甲板重叠（共用左侧/右侧灯条）- 保留特征图案尺寸更小的（更精准）
      if (armor1->left.id == armor2->left.id || armor1->right.id == armor2->right.id) {
        auto area1 = armor1->pattern.cols * armor1->pattern.rows; // 装甲板1图案面积
        auto area2 = armor2->pattern.cols * armor2->pattern.rows; // 装甲板2图案面积
        if (area1 < area2)
          armor2->duplicated = true; // 标记armor2为重复，后续删除
        else
          armor1->duplicated = true; // 标记armor1为重复，后续删除
      }

      // 情况2：装甲板相连（交叉共用灯条）- 保留分类置信度更高的（更可靠）
      if (armor1->left.id == armor2->right.id || armor1->right.id == armor2->left.id) {
        if (armor1->confidence < armor2->confidence)
          armor1->duplicated = true;
        else
          armor2->duplicated = true;
      }
    }
  }

  // 移除所有标记为重复的装甲板，得到最终有效装甲板列表
  armors.remove_if([&](const Armor & a) { return a.duplicated; });

  // 调试模式：实时可视化检测结果（二值化图、灯条、装甲板、关键参数）
  if (debug_) show_result(binary_img, bgr_img, lightbars, armors, frame_count);

  // 返回所有有效装甲板列表
  return armors;
}

/**
 * @brief 单装甲板精准检测接口：基于初始装甲板位置，精细化校正灯条角点
 * @param armor 输入输出参数：传入初始装甲板位置，输出校正后的精准装甲板
 * @param bgr_img 输入原始BGR图像
 * @return bool 校正成功返回true（找到有效灯条并完成校正），失败返回false
 * @details 适用于跟踪后的精准瞄准：基于跟踪得到的初始装甲板位置，扩大裁剪区域，重新提取灯条并校正角点，
 *          提升装甲板中心和角点的检测精度，为弹道解算提供更精准的目标位置
 */
bool Detector::detect(Armor & armor, const cv::Mat & bgr_img)
{
  // 步骤1：获取传入装甲板的原始四个角点（tl:左上, tr:右上, br:右下, bl:左下）
  auto tl = armor.points[0];
  auto tr = armor.points[1];
  auto br = armor.points[2];
  auto bl = armor.points[3];

  // 步骤2：计算灯条方向向量，扩大裁剪区域（避免初始位置偏差导致灯条被裁剪）
  auto lt2b = bl - tl;  // 左灯条方向向量（左上→左下）
  auto rt2b = br - tr;  // 右灯条方向向量（右上→右下）
  // 左灯条上下点外扩：基于灯条中心向两端延长，扩大检测范围
  auto tl1 = (tl + bl) / 2 - lt2b;
  auto bl1 = (tl + bl) / 2 + lt2b;
  // 右灯条上下点外扩：同理左灯条
  auto br1 = (tr + br) / 2 + rt2b;
  auto tr1 = (tr + br) / 2 - rt2b;
  // 计算装甲板水平方向向量，微调角点位置
  auto tl2tr = tr1 - tl1;
  auto bl2br = br1 - bl1;
  // 最终外扩后的四个角点（平衡外扩范围，避免过度扩大导致引入过多噪声）
  auto tl2 = (tl1 + tr) / 2 - 0.75 * tl2tr;
  auto tr2 = (tl1 + tr) / 2 + 0.75 * tl2tr;
  auto bl2 = (bl1 + br) / 2 - 0.75 * bl2br;
  auto br2 = (bl1 + br) / 2 + 0.75 * bl2br;

  // 步骤3：构造外扩后的角点集，拟合旋转矩形并裁剪ROI（感兴趣区域）
  std::vector<cv::Point> points = {tl2, tr2, br2, bl2};
  auto armor_rotaterect = cv::minAreaRect(points); // 拟合外扩区域的旋转矩形
  cv::Rect boundingBox = armor_rotaterect.boundingRect(); // 转换为轴对齐矩形（便于裁剪）

  // 步骤4：边界检查：确保裁剪的ROI不超出原始图像范围（避免数组越界）
  if (
    boundingBox.x < 0 || boundingBox.y < 0 || boundingBox.x + boundingBox.width > bgr_img.cols ||
    boundingBox.y + boundingBox.height > bgr_img.rows) {
    return false;
  }

  // 步骤5：裁剪装甲板ROI区域（从原始图像中提取目标区域，减少计算量）
  cv::Mat armor_roi = bgr_img(boundingBox);
  if (armor_roi.empty()) { // 裁剪失败（ROI无像素），直接返回false
    return false;
  }

  // 步骤6：ROI区域内重新提取灯条（流程与批量检测一致：灰度化→二值化→轮廓检测→灯条筛选）
  cv::Mat gray_img;
  cv::cvtColor(armor_roi, gray_img, cv::COLOR_BGR2GRAY); // 灰度化
  cv::Mat binary_img;
  cv::threshold(gray_img, binary_img, threshold_, 255, cv::THRESH_BINARY); // 二值化
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE); // 轮廓检测

  // 步骤7：筛选ROI内的有效灯条（几何校验+颜色识别）
  std::size_t lightbar_id = 0;
  std::list<Lightbar> lightbars;
  for (const auto & contour : contours) {
    auto rotated_rect = cv::minAreaRect(contour);
    auto lightbar = Lightbar(rotated_rect, lightbar_id);

    if (!check_geometry(lightbar)) continue; // 几何特征校验

    lightbar.color = get_color(bgr_img, contour); // 颜色识别
    // lightbar_points_corrector(lightbar, gray_img); // 关闭PCA角点校正（精准检测时可开启）
    lightbars.emplace_back(lightbar);
    lightbar_id += 1;
  }

  if (lightbars.size() < 2) return false; // 有效灯条数量不足2，无法构成装甲板，返回失败

  // 步骤8：灯条排序（从左到右），便于匹配左右灯条
  lightbars.sort([](const Lightbar & a, const Lightbar & b) { return a.center.x < b.center.x; });

  // 步骤9：精准匹配左右灯条 - 基于原始角点距离筛选最近的灯条（提升匹配精度）
  Lightbar * closest_left_lightbar = nullptr;  // 匹配的左灯条指针
  Lightbar * closest_right_lightbar = nullptr; // 匹配的右灯条指针
  // 初始化最小距离为浮点型最大值（确保首次比较能更新）
  float min_distance_tl_bl = std::numeric_limits<float>::max();
  float min_distance_br_tr = std::numeric_limits<float>::max();

  for (auto & lightbar : lightbars) {
    // 计算当前灯条与原始左灯条（tl/bl）的距离和：距离越小越匹配
    float distance_tl_bl =
      cv::norm(tl - (lightbar.top + cv::Point2f(boundingBox.x, boundingBox.y))) +
      cv::norm(bl - (lightbar.bottom + cv::Point2f(boundingBox.x, boundingBox.y)));
    if (distance_tl_bl < min_distance_tl_bl) {
      min_distance_tl_bl = distance_tl_bl;
      closest_left_lightbar = &lightbar;
    }
    // 计算当前灯条与原始右灯条（br/tr）的距离和：同理左灯条
    float distance_br_tr =
      cv::norm(br - (lightbar.bottom + cv::Point2f(boundingBox.x, boundingBox.y))) +
      cv::norm(tr - (lightbar.top + cv::Point2f(boundingBox.x, boundingBox.y)));
    if (distance_br_tr < min_distance_br_tr) {
      min_distance_br_tr = distance_br_tr;
      closest_right_lightbar = &lightbar;
    }
  }

  // 步骤10：灯条匹配有效性校验，校正装甲板角点坐标
  // 距离和阈值15：经验值，确保匹配的灯条与原始位置偏差在合理范围内
  if (
    closest_left_lightbar && closest_right_lightbar &&
    min_distance_br_tr + min_distance_tl_bl < 15) {
    // 将ROI内的灯条角点转换为原始图像坐标系（加上ROI左上角偏移量）
    armor.points[0] = closest_left_lightbar->top + cv::Point2f(boundingBox.x, boundingBox.y);  // 左上
    armor.points[1] = closest_right_lightbar->top + cv::Point2f(boundingBox.x, boundingBox.y); // 右上
    armor.points[2] = closest_right_lightbar->bottom + cv::Point2f(boundingBox.x, boundingBox.y);//右下
    armor.points[3] = closest_left_lightbar->bottom + cv::Point2f(boundingBox.x, boundingBox.y);//左下
    return true; // 校正成功，返回true
  }

  return false; // 匹配失败/距离过大，返回false
}

/**
 * @brief 灯条几何特征校验：判断单条灯条是否符合预设几何约束
 * @param lightbar 待校验的灯条对象
 * @return bool 所有约束满足返回true，否则false
 * @const 只读方法，不修改类成员和入参对象
 */
bool Detector::check_geometry(const Lightbar & lightbar) const
{
  auto angle_ok = lightbar.angle_error < max_angle_error_; // 角度误差在允许范围内
  auto ratio_ok = lightbar.ratio > min_lightbar_ratio_ && lightbar.ratio < max_lightbar_ratio_; // 长宽比合法
  auto length_ok = lightbar.length > min_lightbar_length_; // 长度超过最小阈值
  return angle_ok && ratio_ok && length_ok; // 三个条件同时满足才为有效灯条
}

/**
 * @brief 装甲板几何特征校验：判断装甲板是否符合预设几何约束
 * @param armor 待校验的装甲板对象
 * @return bool 所有约束满足返回true，否则false
 * @const 只读方法，不修改类成员和入参对象
 */
bool Detector::check_geometry(const Armor & armor) const
{
  auto ratio_ok = armor.ratio > min_armor_ratio_ && armor.ratio < max_armor_ratio_; // 装甲板长宽比合法
  auto side_ratio_ok = armor.side_ratio < max_side_ratio_; // 左右灯条长度比在允许范围内（对称）
  auto rectangular_error_ok = armor.rectangular_error < max_rectangular_error_; // 矩形度误差小（接近标准矩形）
  return ratio_ok && side_ratio_ok && rectangular_error_ok; // 三个条件同时满足才为有效装甲板
}

/**
 * @brief 装甲板名称/置信度校验：过滤非装甲板和低置信度识别结果
 * @param armor 待校验的装甲板对象
 * @return bool 名称有效且置信度达标返回true，否则false
 * @details 1. 过滤分类器识别为"非装甲板"的结果；2. 过滤置信度低于阈值的结果；
 *          3. 保存置信度不达标但名称有效的图案（用于分类器模型迭代训练）；
 *          4. 识别到5号装甲板时打印调试日志（便于针对性调试）
 * @const 只读方法，不修改类成员
 */
bool Detector::check_name(const Armor & armor) const
{
  auto name_ok = armor.name != ArmorName::not_armor; // 名称有效：不是"非装甲板"
  auto confidence_ok = armor.confidence > min_confidence_; // 置信度达标：超过最小阈值

  // 保存不确定样本：名称有效但置信度低，用于后续分类器模型优化
  if (name_ok && !confidence_ok) save(armor);

  // 调试日志：识别到5号装甲板时打印（5号为常见易误识别目标）
  if (armor.name == ArmorName::five) tools::logger()->debug("See pattern 5");

  return name_ok && confidence_ok;
}

/**
 * @brief 装甲板类型校验：过滤类型与数字名称不匹配的异常装甲板
 * @param armor 待校验的装甲板对象
 * @return bool 类型与名称匹配返回true，否则false
 * @details 1. 小装甲板：不能是1号/基地装甲板（1号/基地仅为大装甲板）；
 *          2. 大装甲板：必须是1号/基地装甲板；
 *          3. 保存类型不匹配的异常图案（用于分类器模型迭代）；
 *          4. 打印异常装甲板的调试日志（便于问题定位）
 * @const 只读方法，不修改类成员
 */
bool Detector::check_type(const Armor & armor) const
{
  // 类型与名称匹配规则：基于机器人装甲板实际定义（25赛季规则）
  auto name_ok = armor.type == ArmorType::small
                   ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
                   : (armor.name == ArmorName::one || armor.name == ArmorName::base);

  // 保存异常样本：类型与名称不匹配，用于后续分类器和检测规则优化
  if (!name_ok) {
    tools::logger()->debug(
      "see strange armor: {} {}", ARMOR_TYPES[armor.type], ARMOR_NAMES[armor.name]);
    save(armor);
  }

  return name_ok;
}

/**
 * @brief 提取轮廓区域的主颜色（红/蓝）：基于轮廓像素的颜色通道求和判断
 * @param bgr_img 原始BGR图像（用于获取像素颜色值）
 * @param contour 灯条/装甲板的轮廓点集
 * @return Color 轮廓区域的主颜色（blue/red）
 * @details 遍历轮廓所有像素，分别累加红、蓝通道的像素值，值大的即为主颜色；
 *          BGR格式：通道0=蓝，通道2=红（注意通道顺序，避免颜色识别错误）
 * @const 只读方法，不修改类成员和入参
 */
Color Detector::get_color(const cv::Mat & bgr_img, const std::vector<cv::Point> & contour) const
{
  int red_sum = 0, blue_sum = 0; // 红、蓝通道像素值累加和

  // 遍历轮廓所有像素，累加颜色通道值
  for (const auto & point : contour) {
    blue_sum += bgr_img.at<cv::Vec3b>(point)[0]; // BGR[0]：蓝色通道
    red_sum += bgr_img.at<cv::Vec3b>(point)[2];  // BGR[2]：红色通道
  }

  // 比较累加和，返回主颜色（蓝和红相等时默认返回红，可根据实际场景调整）
  return blue_sum > red_sum ? Color::blue : Color::red;
}

/**
 * @brief 提取装甲板特征图案：裁剪并返回装甲板完整区域的BGR图像
 * @param bgr_img 原始BGR图像
 * @param armor 待提取图案的装甲板对象
 * @return cv::Mat 装甲板特征图案（ROI裁剪后的BGR图像）
 * @details 1. 基于灯条中心和方向向量，向两端延长灯条（比例1.125为经验值，适配实际装甲板尺寸）；
 *          2. 计算延长后的装甲板四个角点，确定ROI裁剪范围；
 *          3. 边界约束：确保ROI不超出原始图像范围；
 *          4. 裁剪并返回装甲板ROI区域（用于分类器识别）
 * @note 1.125 = 0.5 * 装甲板高度 / 灯条长度 = 0.5 * 126mm / 56mm（基于实际装甲板物理尺寸）
 * @const 只读方法，不修改类成员和入参
 */
cv::Mat Detector::get_pattern(const cv::Mat & bgr_img, const Armor & armor) const
{
  // 步骤1：延长灯条，计算装甲板完整区域的四个角点（适配实际装甲板尺寸，包含数字区域）
  auto tl = armor.left.center - armor.left.top2bottom * 1.125;  // 左上：左灯条中心向上延长
  auto bl = armor.left.center + armor.left.top2bottom * 1.125;  // 左下：左灯条中心向下延长
  auto tr = armor.right.center - armor.right.top2bottom * 1.125; // 右上：右灯条中心向上延长
  auto br = armor.right.center + armor.right.top2bottom * 1.125; // 右下：右灯条中心向下延长

  // 步骤2：计算ROI裁剪的上下左右边界（取极值，确定轴对齐矩形范围）
  auto roi_left = std::max<int>(std::min(tl.x, bl.x), 0);        // 左边界：不小于0
  auto roi_top = std::max<int>(std::min(tl.y, tr.y), 0);         // 上边界：不小于0
  auto roi_right = std::min<int>(std::max(tr.x, br.x), bgr_img.cols);  // 右边界：不大于图像宽度
  auto roi_bottom = std::min<int>(std::max(bl.y, br.y), bgr_img.rows); // 下边界：不大于图像高度

  // 步骤3：构造ROI矩形并裁剪
  auto roi_tl = cv::Point(roi_left, roi_top);
  auto roi_br = cv::Point(roi_right, roi_bottom);
  auto roi = cv::Rect(roi_tl, roi_br);

  // 返回裁剪后的装甲板特征图案
  return bgr_img(roi);
}

/**
 * @brief 判定装甲板类型（大/小）：基于长宽比+数字名称综合判断
 * @param armor 待判定类型的装甲板对象
 * @return ArmorType 装甲板类型（big/small）
 * @details 1. 优先基于装甲板长宽比判断（比例>3.0为大装甲板，<2.5为小装甲板）；
 *          2. 比例在2.5~3.0之间时，基于数字名称判断（1号/基地为大装甲板，其余为小装甲板）；
 *          3. 包含TODO注释：后续可优化为直接通过分类器图案识别类型，减少规则依赖
 */
ArmorType Detector::get_type(const Armor & armor)
{
  /// 优先根据当前armor.ratio判断
  /// TODO: 25赛季是否还需要根据比例判断大小装甲？能否根据图案直接判断？

  // 长宽比>3.0：判定为大装甲板（物理尺寸：大装甲板更宽）
  if (armor.ratio > 3.0) {
    return ArmorType::big;
  }

  // 长宽比<2.5：判定为小装甲板
  if (armor.ratio < 2.5) {
    return ArmorType::small;
  }

  // 长宽比模糊（2.5~3.0）：基于数字名称判断（规则定义）
  // 英雄、基地装甲板只能是大装甲板（1号/基地）
  if (armor.name == ArmorName::one || armor.name == ArmorName::base) {
    return ArmorType::big;
  }

  // 其他所有（工程、哨兵、前哨站、步兵）都是小装甲板
  /// TODO: 基地顶装甲是小装甲板，后续可补充规则
  return ArmorType::small;
}

/**
 * @brief 计算装甲板中心的归一化坐标：将像素坐标转换为[0,1]范围的相对坐标
 * @param bgr_img 原始BGR图像（用于获取图像宽高）
 * @param center 装甲板中心的像素坐标（x:列，y:行）
 * @return cv::Point2f 归一化后的中心坐标（x/w, y/h）
 * @details 消除图像分辨率/尺寸的影响，便于后续云台控制、弹道解算等模块的统一处理，
 *          无论图像尺寸如何变化，归一化坐标范围始终为[0,1]
 * @const 只读方法，不修改类成员和入参
 */
cv::Point2f Detector::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const
{
  auto h = bgr_img.rows; // 图像高度（像素）
  auto w = bgr_img.cols; // 图像宽度（像素）
  return {center.x / w, center.y / h}; // 归一化计算：x/宽，y/高
}

/**
 * @brief 保存装甲板特征图案：用于分类器模型迭代训练（仅调试/优化阶段使用）
 * @param armor 待保存的装甲板对象（包含特征图案pattern）
 * @details 1. 以当前系统时间为文件名（避免重复）；2. 文件名格式：{数字名称}_{时间}.jpg；
 *          3. 保存路径为构造函数初始化的save_path_（patterns目录）；
 *          4. 使用fmt库格式化字符串和时间，兼容性好
 * @const 只读方法，不修改类成员和入参
 */
void Detector::save(const Armor & armor) const
{
  // 格式化当前时间：年-月-日_时-分-秒（避免文件名重复）
  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  // 构造保存路径：保存目录/装甲板名称/时间.jpg
  auto img_path = fmt::format("{}/{}_{}.jpg", save_path_, armor.name, file_name);
  // 保存图像到指定路径
  cv::imwrite(img_path, armor.pattern);
}

/**
 * @brief 调试模式：实时可视化检测结果，在图像上叠加绘制关键信息
 * @param binary_img 二值化图像（灯条分割结果）
 * @param bgr_img 原始BGR图像
 * @param lightbars 所有有效灯条列表
 * @param armors 所有有效装甲板列表
 * @param frame_count 帧计数（用于窗口标注）
 * @details 1. 克隆原始图像避免修改原数据；2. 绘制帧计数；3. 绘制灯条信息（角度误差、长宽比、长度、颜色）；
 *          4. 绘制装甲板信息（长宽比、灯条比、矩形度、置信度、名称、类型）；5. 缩小图像尺寸便于显示；
 *          6. 显示检测结果窗口（detection）
 * @const 只读方法，不修改类成员和入参
 */
void Detector::show_result(
  const cv::Mat & binary_img, const cv::Mat & bgr_img, const std::list<Lightbar> & lightbars,
  const std::list<Armor> & armors, int frame_count) const
{
  // 克隆原始图像，用于绘制检测结果（避免修改输入图像）
  auto detection = bgr_img.clone();
  // 绘制帧计数：窗口左上角，白色字体
  tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});

  // 绘制所有有效灯条：标注灯条关键参数+角点
  for (const auto & lightbar : lightbars) {
    // 格式化灯条信息：角度误差（弧度转角度）、长宽比、长度、颜色
    auto info = fmt::format(
      "{:.1f} {:.1f} {:.1f} {}", lightbar.angle_error * 57.3, lightbar.ratio, lightbar.length,
      COLORS[lightbar.color]);
    // 绘制灯条信息：灯条顶部位置，黄色字体
    tools::draw_text(detection, info, lightbar.top, {0, 255, 255});
    // 绘制灯条角点：顶部+底部，黄色点，半径3
    tools::draw_points(detection, lightbar.points, {0, 255, 255}, 3);
  }

  // 绘制所有有效装甲板：标注装甲板关键参数+四个角点
  for (const auto & armor : armors) {
    // 格式化装甲板信息：长宽比、灯条长度比、矩形度（弧度转角度）、置信度、名称、类型
    auto info = fmt::format(
      "{:.2f} {:.2f} {:.1f} {:.2f} {} {}", armor.ratio, armor.side_ratio,
      armor.rectangular_error * 57.3, armor.confidence, ARMOR_NAMES[armor.name],
      ARMOR_TYPES[armor.type]);
    // 绘制装甲板四个角点：绿色点，默认半径
    tools::draw_points(detection, armor.points, {0, 255, 0});
    // 绘制装甲板信息：左灯条底部位置，绿色字体
    tools::draw_text(detection, info, armor.left.bottom, {0, 255, 0});
  }

  // 缩小图像尺寸（0.5倍），便于在小屏幕上显示，减少窗口占用
  cv::Mat binary_img2;
  cv::resize(binary_img, binary_img2, {}, 0.5, 0.5);
  cv::resize(detection, detection, {}, 0.5, 0.5);

  // 显示检测结果窗口（二值化图默认注释，可开启调试）
  // cv::imshow("threshold", binary_img2);
  cv::imshow("detection", detection);
}

/**
 * @brief 灯条角点PCA校正：基于主成分分析回归灯条真实角点，修正轮廓检测的角点偏移
 * @param lightbar 输入输出参数：待校正的灯条对象，校正后更新top/bottom角点
 * @param gray_img 灯条所在区域的灰度图像（用于亮度分析）
 * @details 参考自开源项目CSU-FYT-Vision/FYT2024_vision，针对装甲板灯条做了适配优化；
 *          核心原理：通过PCA分析灯条像素的主分布方向，沿主方向搜索亮度跳变点，确定灯条真实上下角点，
 *          解决轮廓检测因光照/反光导致的角点偏移问题，提升装甲板检测精度；
 *          目前代码中默认关闭，可根据实际场景开启
 * @const 只读方法，不修改类成员
 */
void Detector::lightbar_points_corrector(Lightbar & lightbar, const cv::Mat & gray_img) const
{
  // 配置参数：PCA校正相关经验值，可根据实际场景微调
  constexpr float MAX_BRIGHTNESS = 25;  // 归一化最大亮度值（提升亮度对比）
  constexpr float ROI_SCALE = 0.07;     // ROI扩展比例：避免灯条边缘被裁剪
  constexpr float SEARCH_START = 0.4;   // 角点搜索起始位置：灯条中心向两端40%处
  constexpr float SEARCH_END = 0.6;     // 角点搜索结束位置：灯条中心向两端60%处

  // 步骤1：扩展灯条ROI区域，裁剪并做边界约束
  cv::Rect roi_box = lightbar.rotated_rect.boundingRect(); // 灯条原始包围盒
  // 向四周扩展ROI（基于原始宽度/高度的比例）
  roi_box.x -= roi_box.width * ROI_SCALE;
  roi_box.y -= roi_box.height * ROI_SCALE;
  roi_box.width += 2 * roi_box.width * ROI_SCALE;
  roi_box.height += 2 * roi_box.height * ROI_SCALE;
  // 边界约束：确保ROI不超出原始灰度图像范围
  roi_box &= cv::Rect(0, 0, gray_img.cols, gray_img.rows);

  // 步骤2：ROI区域亮度归一化：提升灯条与背景的对比度，减少光照影响
  cv::Mat roi = gray_img(roi_box); // 裁剪灯条扩展ROI
  const float mean_val = cv::mean(roi)[0]; // 计算ROI平均亮度
  roi.convertTo(roi, CV_32F); // 转换为浮点型，便于后续计算
  // 亮度归一化：将像素值缩放到[0, MAX_BRIGHTNESS]范围
  cv::normalize(roi, roi, 0, MAX_BRIGHTNESS, cv::NORM_MINMAX);

  // 步骤3：计算ROI像素质心：灯条像素的中心位置，作为PCA分析的基准
  const cv::Moments moments = cv::moments(roi); // 计算图像矩
  const cv::Point2f centroid(
    moments.m10 / moments.m00 + roi_box.x, // 质心x坐标（转换为原始图像坐标系）
    moments.m01 / moments.m00 + roi_box.y); // 质心y坐标（转换为原始图像坐标系）

  // 步骤4：生成灯条像素稀疏点云：仅保留亮度非极小值的像素，提升PCA计算效率
  std::vector<cv::Point2f> points;
  for (int i = 0; i < roi.rows; ++i) {
    for (int j = 0; j < roi.cols; ++j) {
      const float weight = roi.at<float>(i, j); // 归一化后的亮度值
      if (weight > 1e-3) {          // 忽略亮度极小值的像素（背景/噪声）
        points.emplace_back(j, i);  // 保存像素坐标（ROI局部坐标系）
      }
    }
  }

  // 步骤5：PCA主成分分析：计算灯条像素的主分布方向（灯条的实际延伸方向）
  cv::PCA pca(cv::Mat(points).reshape(1), cv::Mat(), cv::PCA::DATA_AS_ROW); // 初始化PCA
  // 获取第一主成分（主方向向量）：灯条的延伸方向
  cv::Point2f axis(pca.eigenvectors.at<float>(0, 0), pca.eigenvectors.at<float>(0, 1));
  axis /= cv::norm(axis); // 单位化方向向量
  if (axis.y > 0) axis = -axis;  // 统一方向：确保方向向量向上（便于后续角点搜索）

  // 步骤6：定义角点搜索lambda函数：沿PCA主方向搜索亮度跳变点，确定灯条真实角点
  // direction=1：搜索顶部角点；direction=-1：搜索底部角点
  const auto find_corner = [&](int direction) -> cv::Point2f {
    const float dx = axis.x * direction; // 主方向x步长
    const float dy = axis.y * direction; // 主方向y步长
    const float search_length = lightbar.length * (SEARCH_END - SEARCH_START); // 角点搜索长度

    std::vector<cv::Point2f> candidates; // 存储角点候选点（提升鲁棒性）

    // 横向多线采样：沿灯条宽度方向采样多个候选线，避免单一线搜索的偶然性
    const int half_width = (lightbar.width - 2) / 2; // 灯条半宽（像素）
    for (int i_offset = -half_width; i_offset <= half_width; ++i_offset) {
      // 计算当前采样线的搜索起点（质心为基准，向搜索方向偏移）
      cv::Point2f start_point(
        centroid.x + lightbar.length * SEARCH_START * dx + i_offset,
        centroid.y + lightbar.length * SEARCH_START * dy + i_offset);

      // 沿主方向逐步搜索，寻找亮度跳变点（灯条边缘）
      cv::Point2f corner = start_point;
      float max_diff = 0; // 最大亮度差值（标记跳变点）
      bool found = false; // 是否找到有效跳变点

      for (float step = 0; step < search_length; ++step) {
        // 计算当前搜索点坐标
        const cv::Point2f cur_point(start_point.x + dx * step, start_point.y + dy * step);

        // 边界检查：确保搜索点在原始图像范围内
        if (
          cur_point.x < 0 || cur_point.x >= gray_img.cols || cur_point.y < 0 ||
          cur_point.y >= gray_img.rows) {
          break;
        }

        // 计算亮度差值：当前点与上一点的亮度差，跳变点即为灯条边缘
        const auto prev_val = gray_img.at<uchar>(cv::Point2i(cur_point - cv::Point2f(dx, dy)));
        const auto cur_val = gray_img.at<uchar>(cv::Point2i(cur_point));
        const float diff = prev_val - cur_val; // 亮度差（灯条→背景为正）

        // 筛选有效跳变点：亮度差最大，且上一点亮度高于ROI平均亮度（确保是灯条边缘）
        if (diff > max_diff && prev_val > mean_val) {
          max_diff = diff;
          corner = cur_point - cv::Point2f(dx, dy);  // 跳变发生在上一位置，修正角点
          found = true;
        }
      }

      // 若找到有效跳变点，加入候选点列表
      if (found) {
        candidates.push_back(corner);
      }
    }

    // 返回候选点的均值：减少单一线搜索的误差，提升角点精度
    return candidates.empty()
             ? cv::Point2f(-1, -1) // 未找到候选点，返回无效坐标
             : std::accumulate(candidates.begin(), candidates.end(), cv::Point2f(0, 0)) /
                 static_cast<float>(candidates.size());
  };

  // 步骤7：并行搜索灯条顶部和底部角点，更新灯条对象的角点坐标
  lightbar.top = find_corner(1);   // 搜索顶部角点（direction=1）
  lightbar.bottom = find_corner(-1); // 搜索底部角点（direction=-1）
}

}  // namespace auto_aim
