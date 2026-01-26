#include "yolov5.hpp"

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace auto_aim
{
YOLOV5::YOLOV5(const std::string & config_path, bool debug)
: debug_(debug), detector_(config_path, false)
{
  auto yaml = YAML::LoadFile(config_path);

  // 加载 INT8 模型路径（对应配置文件中的 yolov5_int8_model_path）
  model_path_ = "/home/pldx/Desktop/sp_vision_25-main/assets/yolov5.xml";
  device_ = yaml["device"].as<std::string>();
  binary_threshold_ = yaml["threshold"].as<double>();
  min_confidence_ = yaml["min_confidence"].as<double>();
  int x = 0, y = 0, width = 0, height = 0;
  x = yaml["roi"]["x"].as<int>();
  y = yaml["roi"]["y"].as<int>();
  width = yaml["roi"]["width"].as<int>();
  height = yaml["roi"]["height"].as<int>();
  use_roi_ = yaml["use_roi"].as<bool>();
  use_traditional_ = yaml["use_traditional"].as<bool>();
  roi_ = cv::Rect(x, y, width, height);
  offset_ = cv::Point2f(x, y);

  save_path_ = "imgs";
  std::filesystem::create_directory(save_path_);
  auto model = core_.read_model(model_path_);
  ov::preprocess::PrePostProcessor ppp(model); 

  auto & input = ppp.input();

  input.tensor()
      .set_element_type(ov::element::u8)    // 输入为 INT8 无符号（U8），触发量化优化
      .set_shape({1, 640, 640, 3})          // 输入形状：[批次=1, 高=640, 宽=640, 通道=3]
      .set_layout("NHWC")                    // 原始数据布局（OpenCV 图像格式）
      .set_color_format(ov::preprocess::ColorFormat::BGR);  // 原始图像为 BGR（OpenCV 默认）

  input.model().set_layout("NCHW");  // 模型期望的输入布局（深度学习模型常用）

  input.preprocess()
    .convert_element_type(ov::element::f32)           // U8→F32 转换（模型内部计算用浮点）
    .convert_color(ov::preprocess::ColorFormat::RGB)  // BGR→RGB 通道转换（适配模型训练格式）
    .scale({255.0, 255.0, 255.0});                    // 归一化：[0,255]→[0,1]（显式三通道）

  model = ppp.build();  // 烘焙预处理逻辑到模型

  // 编译模型（仅设置低延迟优化，不指定推理精度，避免 CPU 不兼容）
  // 编译模型（启用 INT8 量化+多核并行+吞吐量优化，帧率最大化）
  compiled_model_ = core_.compile_model(
    model, 
    device_,
    ov::hint::performance_mode(ov::hint::PerformanceMode::THROUGHPUT),  // 帧率优先
    ov::inference_num_threads(std::thread::hardware_concurrency()),     // 用满所有 CPU 核心
    ov::cache_dir("./openvino_cache")                                   // 缓存编译结果（后续启动更快）
  );

  tools::logger()->info("[YOLOV5] INT8 量化模型初始化成功！模型路径：{}，设备：{}", model_path_, device_);
}

std::list<Armor> YOLOV5::detect(const cv::Mat & raw_img, int frame_count)
{
  if (raw_img.empty()) 
  {
    tools::logger()->warn("Empty img!, camera drop!");
    return std::list<Armor>();
  }

  cv::Mat bgr_img;
  if (use_roi_) 
  {
    if (roi_.width == -1) 
    {
      roi_.width = raw_img.cols;
    }
    if (roi_.height == -1) 
    {
      roi_.height = raw_img.rows;
    }
    bgr_img = raw_img(roi_);
  } else 
  {
    bgr_img = raw_img;
  }

  // 等比例缩放图像到 640×640（黑色填充）
  auto x_scale = static_cast<double>(640) / bgr_img.rows;
  auto y_scale = static_cast<double>(640) / bgr_img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(bgr_img.rows * scale);
  auto w = static_cast<int>(bgr_img.cols * scale);
  auto input = cv::Mat(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(bgr_img, input(roi), {w, h});

  // 执行推理
  auto infer_request = compiled_model_.create_infer_request();
  ov::Tensor input_tensor(ov::element::u8, {1, 640, 640, 3}, input.data);
  infer_request.set_input_tensor(input_tensor);
  infer_request.infer();

  // 获取输出并兼容 INT8/FP32 类型
  auto output_tensor = infer_request.get_output_tensor();    
  auto output_shape = output_tensor.get_shape();
  cv::Mat output;

  // 自动判断输出类型，适配量化/非量化模型
  if (output_tensor.get_element_type() == ov::element::i8) {
    // 若输出为 INT8 有符号，反量化为 FP32
    cv::Mat output_int8(output_shape[1], output_shape[2], CV_8S, output_tensor.data());
    output_int8.convertTo(output, CV_32F);
  } else {
    // 若输出为 FP32，直接转换（模型已通过输入 U8 触发量化优化）
    output = cv::Mat(output_shape[1], output_shape[2], CV_32F, output_tensor.data());
  }

  return parse(scale, output, raw_img, frame_count);
}

std::list<Armor> YOLOV5::parse(
  double scale,
  cv::Mat & output,
  const cv::Mat & bgr_img,
  int frame_count
)
{
  std::vector<int> color_ids, num_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> armors_key_points;

  for (int r = 0; r < output.rows; r++)
  {
    // 提取装甲板置信度并 sigmoid 激活
    double score = output.at<float>(r, 8);
    score = sigmoid(score);
    if (score < score_threshold_) continue;

    // 提取颜色和编号分类分数
    cv::Mat color_scores = output.row(r).colRange(9, 13);
    cv::Mat classes_scores = output.row(r).colRange(13, 22);

    // 找到最高分对应的类别 ID
    cv::Point class_id, color_id;
    int _class_id, _color_id;
    double score_color, score_num;
    cv::minMaxLoc(classes_scores, NULL, &score_num, NULL, &class_id);
    cv::minMaxLoc(color_scores, NULL, &score_color, NULL, &color_id);
    _class_id = class_id.x;
    _color_id = color_id.x;

    // 提取 4 个关键点坐标（映射回原图尺寸）
    std::vector<cv::Point2f> armor_key_points;
    armor_key_points.push_back(cv::Point2f(output.at<float>(r, 0) / scale, output.at<float>(r, 1) / scale));
    armor_key_points.push_back(cv::Point2f(output.at<float>(r, 6) / scale, output.at<float>(r, 7) / scale));
    armor_key_points.push_back(cv::Point2f(output.at<float>(r, 4) / scale, output.at<float>(r, 5) / scale));
    armor_key_points.push_back(cv::Point2f(output.at<float>(r, 2) / scale, output.at<float>(r, 3) / scale));

    // 计算最小外接矩形
    float min_x = armor_key_points[0].x, max_x = armor_key_points[0].x;
    float min_y = armor_key_points[0].y, max_y = armor_key_points[0].y;
    for (int i = 1; i < armor_key_points.size(); i++) 
    {
      min_x = std::min(min_x, armor_key_points[i].x);
      max_x = std::max(max_x, armor_key_points[i].x);
      min_y = std::min(min_y, armor_key_points[i].y);
      max_y = std::max(max_y, armor_key_points[i].y);
    }
    cv::Rect rect(min_x, min_y, max_x - min_x, max_y - min_y);

    // 保存检测结果
    color_ids.emplace_back(_color_id);
    num_ids.emplace_back(_class_id);
    boxes.emplace_back(rect);
    confidences.emplace_back(score);
    armors_key_points.emplace_back(armor_key_points);
  }

  // NMS 去除冗余框
  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

  // 构造 Armor 列表并二次筛选
  std::list<Armor> armors;
  for (const auto & i : indices)
  {
    if (use_roi_)
    {
      armors.emplace_back(color_ids[i], num_ids[i], confidences[i], boxes[i], armors_key_points[i], offset_);
    }else
    {
      armors.emplace_back(color_ids[i], num_ids[i], confidences[i], boxes[i], armors_key_points[i]);
    }
  }

  tmp_img_ = bgr_img;
  for (auto it = armors.begin(); it != armors.end();)
  {
    if (!check_name(*it)) { it = armors.erase(it); continue; }
    if (!check_type(*it)) { it = armors.erase(it); continue; }
    if (use_traditional_) detector_.detect(*it, bgr_img);
    it->center_norm = get_center_norm(bgr_img, it->center);
    ++it;
  }

  // 调试模式绘制检测结果
  if (debug_) draw_detections(bgr_img, armors, frame_count);

  return armors;
}

bool YOLOV5::check_name(const Armor & armor) const
{
  auto name_ok = armor.name != ArmorName::not_armor;
  auto confidence_ok = armor.confidence > min_confidence_;
  return name_ok && confidence_ok;
}

bool YOLOV5::check_type(const Armor & armor) const
{
  auto name_ok = (armor.type == ArmorType::small)
                   ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
                   : (armor.name != ArmorName::two && armor.name != ArmorName::sentry &&
                      armor.name != ArmorName::outpost);
  return name_ok;
}

cv::Point2f YOLOV5::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const
{
  auto h = bgr_img.rows;
  auto w = bgr_img.cols;
  return {center.x / w, center.y / h};
}

void YOLOV5::draw_detections(
  const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const
{
  auto detection = img.clone();
  tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});
  for (const auto & armor : armors) {
    auto info = fmt::format(
      "{:.2f} {} {} {}", armor.confidence, COLORS[armor.color], ARMOR_NAMES[armor.name],
      ARMOR_TYPES[armor.type]);
    tools::draw_points(detection, armor.points, {0, 255, 0});
    tools::draw_text(detection, info, armor.center, {0, 255, 0});
  }

  if (use_roi_) {
    cv::Scalar green(0, 255, 0);
    cv::rectangle(detection, roi_, green, 2);
  }
  cv::resize(detection, detection, {}, 1.0, 1.0);
  cv::imshow("detection", detection);
}

void YOLOV5::save(const Armor & armor) const
{
  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  auto img_path = fmt::format("{}/{}_{}.jpg", save_path_, armor.name, file_name);
  cv::imwrite(img_path, tmp_img_);
}

double YOLOV5::sigmoid(double x)
{
  if (x > 0)
    return 1.0 / (1.0 + exp(-x));
  else
    return exp(x) / (1.0 + exp(x));
}

std::list<Armor> YOLOV5::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return parse(scale, output, bgr_img, frame_count);
}

}  // namespace auto_aim