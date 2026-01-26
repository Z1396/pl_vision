#include "mt_detector.hpp"

#include <yaml-cpp/yaml.h>

namespace auto_aim
{
namespace multithread
{

/**
 * @brief 多线程检测器类构造函数
 * @param config_path 配置文件路径
 * @param debug 是否启用调试模式
 * 初始化YOLO检测器，并配置OpenVINO推理模型
 */
MultiThreadDetector::MultiThreadDetector(const std::string & config_path, bool debug)
: yolo_(config_path, debug)  // 初始化YOLO检测器成员
{
  // 加载YAML配置文件
  auto yaml = YAML::LoadFile(config_path);
  
  // 从配置文件读取YOLO模型名称和路径
  auto yolo_name = yaml["yolo_name"].as<std::string>();
  auto model_path = yaml[yolo_name + "_model_path"].as<std::string>();
  
  // 读取推理设备配置（如CPU、GPU等）
  device_ = yaml["device"].as<std::string>();

  // 读取OpenVINO模型
  auto model = core_.read_model(model_path);
  
  // 创建预处理/后处理处理器
  ov::preprocess::PrePostProcessor ppp(model);
  auto & input = ppp.input();  // 获取模型输入信息

  // 配置输入张量参数
  input.tensor()
    .set_element_type(ov::element::u8)  // 输入数据类型为无符号8位整数
    .set_shape({1, 640, 640, 3})        // 输入形状：1张640x640的3通道图像（NHWC格式）
    .set_layout("NHWC")                 // 数据布局：[批次, 高度, 宽度, 通道]
    .set_color_format(ov::preprocess::ColorFormat::BGR);  // 颜色格式为BGR

  // 配置模型期望的输入布局
  input.model().set_layout("NCHW");  // 模型期望的布局：[批次, 通道, 高度, 宽度]

  // 配置预处理步骤
  input.preprocess()
    .convert_element_type(ov::element::f32)  // 转换为32位浮点数
    .convert_color(ov::preprocess::ColorFormat::RGB)  // 转换颜色空间为RGB
    // .resize(ov::preprocess::ResizeAlgorithm::RESIZE_LINEAR)  // 注释掉的 resize 步骤
    .scale(255.0);  // 缩放因子（将0-255范围转换为0-1范围）

  // 构建预处理模型
  model = ppp.build();
  
  // 编译模型到指定设备，使用吞吐量优先的性能模式
  compiled_model_ = core_.compile_model(
    model, device_, 
    ov::hint::performance_mode(ov::hint::PerformanceMode::THROUGHPUT));

  // 输出初始化完成日志
  tools::logger()->info("[MultiThreadDetector] initialized !");
}

/**
 * @brief 推送图像到处理队列，进行异步推理预处理
 * @param img 输入图像
 * @param t 图像对应的时间戳
 * 将图像预处理后提交给OpenVINO进行异步推理
 */
void MultiThreadDetector::push(cv::Mat img, std::chrono::steady_clock::time_point t)
{
  // 计算图像缩放比例（保持比例缩放到640x640）
  auto x_scale = static_cast<double>(640) / img.rows;  // 高度方向缩放比例
  auto y_scale = static_cast<double>(640) / img.cols;  // 宽度方向缩放比例
  auto scale = std::min(x_scale, y_scale);             // 取最小比例保持图像比例
  auto h = static_cast<int>(img.rows * scale);         // 缩放后的高度
  auto w = static_cast<int>(img.cols * scale);         // 缩放后的宽度

  // 预处理：创建640x640的空白图像（黑色背景）
  auto input = cv::Mat(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  auto roi = cv::Rect(0, 0, w, h);  // 定义感兴趣区域（缩放后的图像区域）
  cv::resize(img, input(roi), {w, h});  // 将原图缩放到ROI区域

  // 准备推理请求
  auto input_port = compiled_model_.input();  // 获取输入端口
  auto infer_request = compiled_model_.create_infer_request();  // 创建推理请求
  
  // 创建输入张量并关联预处理后的图像数据
  ov::Tensor input_tensor(ov::element::u8, {1, 640, 640, 3}, input.data);

  // 设置输入张量并启动异步推理
  infer_request.set_input_tensor(input_tensor);
  infer_request.start_async();  // 异步执行推理，不阻塞当前线程
  
  // 将图像、时间戳和推理请求推入处理队列
  queue_.push({img.clone(), t, std::move(infer_request)});
}

/**
 * @brief 从队列中取出处理结果
 * @return 包含装甲板列表和对应时间戳的元组
 * 等待异步推理完成，进行后处理并返回检测结果
 */
std::tuple<std::list<Armor>, std::chrono::steady_clock::time_point> MultiThreadDetector::pop()
{
  // 从队列取出图像、时间戳和推理请求
  auto [img, t, infer_request] = queue_.pop();
  
  // 等待异步推理完成
  infer_request.wait();

  // 后处理：获取推理输出张量
  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();  // 获取输出形状
  
  // 将输出张量转换为OpenCV矩阵
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());
  
  // 计算缩放比例（与push中保持一致）
  auto x_scale = static_cast<double>(640) / img.rows;
  auto y_scale = static_cast<double>(640) / img.cols;
  auto scale = std::min(x_scale, y_scale);
  
  // 调用YOLO的后处理函数获取装甲板检测结果（暂不支持ROI）
  auto armors = yolo_.postprocess(scale, output, img, 0);

  // 返回装甲板列表和时间戳
  return {std::move(armors), t};
}

/**
 * @brief 调试模式下从队列中取出处理结果
 * @return 包含原始图像、装甲板列表和对应时间戳的元组
 * 与pop功能类似，但额外返回原始图像用于调试可视化
 */
std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point>
MultiThreadDetector::debug_pop()
{
  // 从队列取出图像、时间戳和推理请求
  auto [img, t, infer_request] = queue_.pop();
  
  // 等待异步推理完成
  infer_request.wait();

  // 后处理：获取推理输出张量
  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  
  // 将输出张量转换为OpenCV矩阵
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());
  
  // 计算缩放比例
  auto x_scale = static_cast<double>(640) / img.rows;
  auto y_scale = static_cast<double>(640) / img.cols;
  auto scale = std::min(x_scale, y_scale);
  
  // 调用YOLO的后处理函数获取装甲板检测结果
  auto armors = yolo_.postprocess(scale, output, img, 0);  // 暂不支持ROI

  // 返回原始图像、装甲板列表和时间戳（用于调试）
  return {img, std::move(armors), t};
}

}  // namespace multithread

}  // namespace auto_aim