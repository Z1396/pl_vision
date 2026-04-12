/**
 * @file split_video.cpp
 * @brief 视频分帧工具程序
 *
 * 功能：将视频文件按指定帧范围分割，并保存对应的时间戳数据
 *
 * 使用场景：
 * - 从录制视频中提取标定所需的帧
 * - 筛选特定时间段的视频数据
 * - 批量处理视频数据
 *
 * 输入格式：
 * - 视频文件：{输入路径}.avi
 * - 时间戳文件：{输入路径}.txt (格式: timestamp w x y z)
 *
 * 输出格式：
 * - 视频文件：{输出路径}.avi
 * - 时间戳文件：{输出路径}.txt
 *
 * 使用方法：
 *   ./split_video <输入路径> [-s <起始帧>] [-e <结束帧>] [-p <输出路径>]
 *   示例：./split_video records/Big/2024-05-14_11-6-26 -s 0 -e 1000 -p output/Big_clip
 */

#include <fmt/format.h>

#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

/**
 * @brief 命令行参数定义
 */
const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{start-index s  | 0                      | 视频起始帧下标    }"
  "{end-index e    | -1                     | 视频结束帧下标(-1表示到末尾)}"
  "{output-path p  | records/Big/2024-05-14_11-6-26_output | avi和txt文件的输出路径}"
  "{@input-path    |                        | avi和txt文件的路径}";

/**
 * @brief 读取时间戳数据
 *
 * @param text_path 时间戳文件路径
 * @return std::vector<std::tuple<double, double, double, double, double>> 时间戳列表
 *         每个元素为 (timestamp, w, x, y, z)
 */
std::vector<std::tuple<double, double, double, double, double>> read_timestamps(
  const std::string & text_path)
{
  std::vector<std::tuple<double, double, double, double, double>> timestamps;
  std::ifstream text(text_path);
  std::string line;

  while (std::getline(text, line)) {
    std::istringstream iss(line);
    double t, w, x, y, z;
    if (iss >> t >> w >> x >> y >> z) {
      timestamps.emplace_back(t, w, x, y, z);
    }
  }

  return timestamps;
}

/**
 * @brief 写入时间戳数据
 *
 * @param text_path 输出文件路径
 * @param timestamps 时间戳数据列表
 */
void write_timestamps(
  const std::string & text_path,
  const std::vector<std::tuple<double, double, double, double, double>> & timestamps)
{
  std::ofstream outtext(text_path);
  for (const auto & [t, w, x, y, z] : timestamps) {
    outtext << fmt::format("{} {} {} {} {}\n", t, w, x, y, z);
  }
  outtext.close();
}

/**
 * @brief 显示进度条
 *
 * @param current 当前进度
 * @param total 总进度
 * @param bar_width 进度条宽度
 */
void print_progress_bar(int current, int total, int bar_width = 50)
{
  float progress = static_cast<float>(current) / total;
  int pos = static_cast<int>(bar_width * progress);

  std::cout << "[";
  for (int i = 0; i < bar_width; ++i) {
    if (i < pos)
      std::cout << "=";
    else if (i == pos)
      std::cout << ">";
    else
      std::cout << " ";
  }
  std::cout << "] " << int(progress * 100.0) << "% (" << current << "/" << total << ")\r";
  std::cout.flush();
}

/**
 * @brief 主函数 - 视频分帧工具入口
 *
 * @param argc 命令行参数数量
 * @param argv 命令行参数列表
 * @return int 程序退出码
 *
 * @note 输入文件格式：
 *       - 视频：{input_path}.avi
 *       - 时间戳：{input_path}.txt (每行格式: timestamp w x y z)
 *
 * @note 时间戳文件格式说明：
 *       - timestamp: 时间戳（秒）
 *       - w, x, y, z: 四元数分量
 */
int main(int argc, char * argv[])
{
  fmt::print("===========================================\n");
  fmt::print("        视频分帧工具 v1.0\n");
  fmt::print("===========================================\n\n");

  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto input_path = cli.get<std::string>(0);
  auto output_path = cli.get<std::string>("output-path");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");

  fmt::print("输入路径: {}\n", input_path);
  fmt::print("输出路径: {}\n", output_path);
  fmt::print("起始帧: {}\n", start_index);
  fmt::print("结束帧: {}\n", end_index == -1 ? "末尾" : std::to_string(end_index));
  fmt::print("\n");

  // 构造视频和文本文件路径
  auto video_path = fmt::format("{}.avi", input_path);
  auto text_path = fmt::format("{}.txt", input_path);

  // 打开视频文件
  cv::VideoCapture video(video_path);
  if (!video.isOpened()) {
    fmt::print("错误：无法打开视频文件: {}\n", video_path);
    return -1;
  }

  // 读取时间戳数据
  auto all_timestamps = read_timestamps(text_path);
  if (all_timestamps.empty()) {
    fmt::print("错误：无法读取时间戳文件: {}\n", text_path);
    return -1;
  }

  // 获取视频参数
  int frameWidth = static_cast<int>(video.get(cv::CAP_PROP_FRAME_WIDTH));
  int frameHeight = static_cast<int>(video.get(cv::CAP_PROP_FRAME_HEIGHT));
  double fps = video.get(cv::CAP_PROP_FPS);
  int fourcc = static_cast<int>(video.get(cv::CAP_PROP_FOURCC));
  int total_frames = static_cast<int>(video.get(cv::CAP_PROP_FRAME_COUNT));

  fmt::print("=== 视频信息 ===\n");
  fmt::print("分辨率: {}×{}\n", frameWidth, frameHeight);
  fmt::print("帧率: {:.2f} fps\n", fps);
  fmt::print("总帧数: {}\n", total_frames);
  fmt::print("时间戳数: {}\n", all_timestamps.size());

  // 验证帧索引范围
  if (start_index < 0) {
    start_index = 0;
    fmt::print("警告：起始帧索引无效，已调整为0\n");
  }
  if (end_index == -1 || end_index >= total_frames) {
    end_index = total_frames - 1;
    fmt::print("警告：结束帧索引无效，已调整为{}\n", end_index);
  }
  if (start_index > end_index) {
    fmt::print("错误：起始帧({})大于结束帧({})\n", start_index, end_index);
    return -1;
  }

  // 设置视频起始帧
  video.set(cv::CAP_PROP_POS_FRAMES, start_index);

  // 跳过起始帧对应的时间戳
  int timestamp_skip = start_index;
  for (int i = 0; i < timestamp_skip && i < static_cast<int>(all_timestamps.size()); i++) {
    (void)all_timestamps[i];  // 消耗时间戳
  }

  // 创建输出文件
  auto outvideo_path = fmt::format("{}.avi", output_path);
  auto outtext_path = fmt::format("{}.txt", output_path);
  cv::VideoWriter outvideo(outvideo_path, fourcc, fps, cv::Size(frameWidth, frameHeight));

  if (!outvideo.isOpened()) {
    fmt::print("错误：无法创建输出视频文件: {}\n", outvideo_path);
    return -1;
  }

  // 准备输出时间戳数据
  std::vector<std::tuple<double, double, double, double, double>> output_timestamps;

  // 处理视频帧
  cv::Mat img;
  int frame_count = start_index;
  int processed_frames = 0;
  int total_process = end_index - start_index + 1;

  fmt::print("\n=== 开始处理 ===\n");

  while (video.read(img)) {
    if (frame_count > end_index) break;
    if (img.empty()) break;

    // 写入输出视频
    outvideo.write(img);

    // 复制对应时间戳
    if (frame_count < static_cast<int>(all_timestamps.size())) {
      output_timestamps.push_back(all_timestamps[frame_count]);
    }

    // 显示进度
    processed_frames++;
    if (processed_frames % 10 == 0 || processed_frames == total_process) {
      print_progress_bar(processed_frames, total_process);
    }

    frame_count++;
  }

  std::cout << std::endl;

  // 写入时间戳文件
  write_timestamps(outtext_path, output_timestamps);

  // 释放资源
  video.release();
  outvideo.release();

  // 输出处理结果
  fmt::print("\n=== 处理完成 ===\n");
  fmt::print("输入: {} ({}帧)\n", video_path, total_frames);
  fmt::print("输出: {} ({}帧)\n", outvideo_path, processed_frames);
  fmt::print("帧范围: [{}, {}]\n", start_index, end_index);
  fmt::print("时间戳: {} 条\n", output_timestamps.size());

  fmt::print("\n===========================================\n");
  fmt::print("视频分帧工具结束\n");
  fmt::print("===========================================\n");

  cv::destroyAllWindows();
  return 0;
}
