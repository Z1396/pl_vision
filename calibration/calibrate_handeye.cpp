/**
 * @file calibrate_handeye.cpp
 * @brief 手眼标定程序（圆点标定板版本）
 *
 * 功能：使用圆点标定板进行相机标定，获取相机内参和畸变系数
 *
 * 坐标系说明：
 * - 标定板坐标系：Z轴垂直于标定板平面，X/Y轴沿圆点中心分布
 * - 相机坐标系：原点位于相机光心，Z轴沿光轴方向
 * - 图像坐标系：原点位于图像左上角，X轴向右，Y轴向下
 *
 * 标定原理：
 * 1. 使用圆点标定板（对称圆点图案）
 * 2. 提取圆点的2D像素坐标
 * 3. 根据已知的3D空间坐标，建立相机投影模型
 * 4. 通过最小二乘法求解相机内参和畸变系数
 *
 * 使用方法：
 *   ./calibrate_handeye <输入文件夹路径> [-c <配置文件路径>]
 *   示例：./calibrate_handeye assets/img_with_q -c configs/calibration.yaml
 *
 * @note 与calibrate_camera.cpp的区别：使用圆点标定板而非棋盘格
 * @note 圆点标定板优点：对光照更鲁棒，检测更稳定
 */

#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <opencv2/opencv.hpp>

#include "tools/img_tools.hpp"

/**
 * @brief 命令行参数定义
 */
const std::string keys =
  "{help h usage ? |                          | 输出命令行参数说明}"
  "{config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{@input-folder  | assets/img_with_q         | 输入文件夹路径   }";

/**
 * @brief 生成圆点标定板的3D坐标（对称分布）
 *
 * @param pattern_size 标定板尺寸（列数×行数）
 * @param center_distance 圆点中心间距（单位：mm）
 * @return std::vector<cv::Point3f> 3D圆点中心坐标列表
 *
 * @note 坐标系定义（对称分布）：
 *       - 原点位于标定板几何中心
 *       - X轴向右，Y轴向下，Z轴垂直于标定板平面（向外）
 *
 * @note 圆点标定板特点：
 *       - 对称圆点图案 (cv::CALIB_CB_SYMMETRIC_GRID)
 *       - 相邻圆点间距相等
 */
std::vector<cv::Point3f> centers_3d(const cv::Size & pattern_size, const float center_distance)
{
  std::vector<cv::Point3f> centers_3d;

  for (int i = 0; i < pattern_size.height; i++) 
  {
    for (int j = 0; j < pattern_size.width; j++) 
    {
      float x = (j - 0.5 * (pattern_size.width - 1)) * center_distance;
      float y = (i - 0.5 * (pattern_size.height - 1)) * center_distance;
      float z = 0;
      centers_3d.push_back({x, y, z});
    }
  }

  return centers_3d;
}

/**
 * @brief 从指定文件夹加载标定图像
 *
 * @param input_folder 输入文件夹路径（包含编号的.jpg图像）
 * @param config_path 配置文件路径
 * @param img_size 输出：图像尺寸
 * @param obj_points 输出：所有视角的3D坐标
 * @param img_points 输出：所有视角的2D像素坐标
 *
 * @return bool 是否成功加载数据
 *
 * @note 支持圆点标定板的自动检测
 * @note 自动过滤识别失败的图像
 */
void load(
  const std::string & input_folder, const std::string & config_path, cv::Size & img_size,
  std::vector<std::vector<cv::Point3f>> & obj_points,
  std::vector<std::vector<cv::Point2f>> & img_points)
{
  // 读取yaml参数
  auto yaml = YAML::LoadFile(config_path);
  auto pattern_cols = yaml["pattern_cols"].as<int>(15);
  auto pattern_rows = yaml["pattern_rows"].as<int>(15);
  auto center_distance_mm = yaml["chessboard_square_size_mm"].as<double>(10.0);
  cv::Size pattern_size(pattern_cols, pattern_rows);

  fmt::print("圆点标定板参数：{}×{} 圆点，圆点间距 {} mm\n", pattern_cols, pattern_rows, center_distance_mm);
  fmt::print("正在加载图像...\n");

  int success_count = 0;
  int failure_count = 0;

  for (int i = 1; true; i++) {
    // 读取图片
    auto img_path = fmt::format("{}/{}.jpg", input_folder, i);
    auto img = cv::imread(img_path);
    if (img.empty()) {
      if (i == 1) 
      {
        fmt::print("错误：无法读取图像 {}\n", img_path);
      }
      break;
    }

    // 设置图片尺寸
    img_size = img.size();

    // 识别标定板 - 使用圆点标定板检测
    std::vector<cv::Point2f> centers_2d;
    auto success = cv::findCirclesGrid(img, pattern_size, centers_2d, cv::CALIB_CB_SYMMETRIC_GRID);

    // 显示识别结果
    auto drawing = img.clone();
    cv::drawChessboardCorners(drawing, pattern_size, centers_2d, success);
    cv::resize(drawing, drawing, {}, 0.5, 0.5);  // 缩小图片尺寸便于显示完全
    cv::imshow("Press any to continue", drawing);
    cv::waitKey(10);  // 减少等待时间

    // 输出识别结果
    fmt::print("[{}] {}\n", success ? "success" : "failure", img_path);
    if (success) {
      success_count++;
    } else {
      failure_count++;
      continue;
    }

    // 记录所需的数据
    img_points.emplace_back(centers_2d);
    obj_points.emplace_back(centers_3d(pattern_size, center_distance_mm));
  }

  fmt::print("\n加载完成：成功 {} 张，失败 {} 张\n", success_count, failure_count);

  // 数据验证
  if (success_count < 10) {
    fmt::print("警告：有效样本数少于10张，标定结果可能不够准确\n");
  }
}

/**
 * @brief 将标定结果输出为YAML格式
 *
 * @param camera_matrix 相机内参矩阵 (3×3)
 * @param distort_coeffs 畸变系数 (1×5)
 * @param error 重投影误差（像素）
 */
void print_yaml(const cv::Mat & camera_matrix, const cv::Mat & distort_coeffs, double error)
{
  YAML::Emitter result;
  std::vector<double> camera_matrix_data(
    camera_matrix.begin<double>(), camera_matrix.end<double>());
  std::vector<double> distort_coeffs_data(
    distort_coeffs.begin<double>(), distort_coeffs.end<double>());

  result << YAML::BeginMap;
  result << YAML::Comment(fmt::format("重投影误差: {:.4f}px", error));
  result << YAML::Key << "camera_matrix";
  result << YAML::Value << YAML::Flow << camera_matrix_data;
  result << YAML::Key << "distort_coeffs";
  result << YAML::Value << YAML::Flow << distort_coeffs_data;
  result << YAML::Newline;
  result << YAML::EndMap;

  fmt::print("\n标定结果（圆点标定板）：\n{}\n", result.c_str());

  // 打印详细参数说明
  fmt::print("\n=== 相机内参矩阵说明 ===\n");
  fmt::print("fx = {:.2f} (焦距 x方向)\n", camera_matrix.at<double>(0, 0));
  fmt::print("fy = {:.2f} (焦距 y方向)\n", camera_matrix.at<double>(1, 1));
  fmt::print("cx = {:.2f} (主点 x坐标)\n", camera_matrix.at<double>(0, 2));
  fmt::print("cy = {:.2f} (主点 y坐标)\n", camera_matrix.at<double>(1, 2));

  fmt::print("\n=== 畸变系数说明 ===\n");
  fmt::print("k1 = {:.6f} (径向畸变)\n", distort_coeffs.at<double>(0, 0));
  fmt::print("k2 = {:.6f} (径向畸变)\n", distort_coeffs.at<double>(0, 1));
  fmt::print("p1 = {:.6f} (切向畸变)\n", distort_coeffs.at<double>(0, 2));
  fmt::print("p2 = {:.6f} (切向畸变)\n", distort_coeffs.at<double>(0, 3));
  fmt::print("k3 = {:.6f} (径向畸变)\n", distort_coeffs.at<double>(0, 4));
}

/**
 * @brief 计算重投影误差
 *
 * @param obj_points 3D标定板坐标
 * @param img_points 2D图像坐标
 * @param rvecs 旋转向量列表
 * @param tvecs 平移向量列表
 * @param camera_matrix 相机内参矩阵
 * @param distort_coeffs 畸变系数
 * @return double 平均重投影误差
 */
double calculate_reprojection_error(
  const std::vector<std::vector<cv::Point3f>> & obj_points,
  const std::vector<std::vector<cv::Point2f>> & img_points,
  const std::vector<cv::Mat> & rvecs,
  const std::vector<cv::Mat> & tvecs,
  const cv::Mat & camera_matrix,
  const cv::Mat & distort_coeffs)
{
  double error_sum = 0;
  size_t total_points = 0;

  for (size_t i = 0; i < obj_points.size(); i++) {
    std::vector<cv::Point2f> reprojected_points;
    cv::projectPoints(
      obj_points[i], rvecs[i], tvecs[i], camera_matrix, distort_coeffs, reprojected_points);

    total_points += reprojected_points.size();
    for (size_t j = 0; j < reprojected_points.size(); j++)
      error_sum += cv::norm(img_points[i][j] - reprojected_points[j]);
  }

  return error_sum / total_points;
}

/**
 * @brief 主函数 - 圆点标定板相机标定程序入口
 *
 * @param argc 命令行参数数量
 * @param argv 命令行参数列表
 * @return int 程序退出码
 *
 * @note 配置文件格式 (calibration.yaml):
 *   pattern_cols: 10        # 圆点列数
 *   pattern_rows: 7         # 圆点行数
 *   center_distance_mm: 40  # 圆点中心间距（毫米）
 */
int main(int argc, char * argv[])
{
  fmt::print("===========================================\n");
  fmt::print("  圆点标定板相机标定程序 v1.0\n");
  fmt::print("===========================================\n\n");

  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_folder = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");

  fmt::print("输入文件夹: {}\n", input_folder);
  fmt::print("配置文件: {}\n\n", config_path);

  // 从输入文件夹中加载标定所需的数据
  cv::Size img_size;
  std::vector<std::vector<cv::Point3f>> obj_points;
  std::vector<std::vector<cv::Point2f>> img_points;
  load(input_folder, config_path, img_size, obj_points, img_points);

  // 检查数据有效性
  if (obj_points.empty() || img_points.empty()) {
    fmt::print("错误：没有有效的标定数据！\n");
    return -1;
  }

  fmt::print("图像尺寸: {}×{}\n", img_size.width, img_size.height);
  fmt::print("标定样本数: {}\n\n", obj_points.size());

  // 相机标定
  cv::Mat camera_matrix, distort_coeffs;
  std::vector<cv::Mat> rvecs, tvecs;
  auto criteria = cv::TermCriteria(
    cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100,
    DBL_EPSILON);

  fmt::print("正在执行相机标定...\n");
  cv::calibrateCamera(
    obj_points, img_points, img_size, camera_matrix, distort_coeffs, rvecs, tvecs, cv::CALIB_FIX_K3,
    criteria);
  fmt::print("标定完成！\n\n");

  // 计算重投影误差
  double error = calculate_reprojection_error(
    obj_points, img_points, rvecs, tvecs, camera_matrix, distort_coeffs);

  fmt::print("=== 重投影误差分析 ===\n");
  fmt::print("平均误差: {:.4f} px\n", error);

  // 误差评估
  if (error < 0.5) {
    fmt::print("评估: 标定精度优秀\n\n");
  } else if (error < 1.0) {
    fmt::print("评估: 标定精度良好\n\n");
  } else if (error < 2.0) {
    fmt::print("评估: 标定精度一般，建议重新标定\n\n");
  } else {
    fmt::print("评估: 标定精度较差，需要重新标定\n\n");
  }

  // 输出yaml
  print_yaml(camera_matrix, distort_coeffs, error);

  fmt::print("\n===========================================\n");
  fmt::print("圆点标定板标定程序结束\n");
  fmt::print("===========================================\n");

  cv::destroyAllWindows();
  return 0;
}
