/**
 * @file calibrate_robotworld_handeye.cpp
 * @brief 机器人世界坐标系手眼标定程序
 *
 * 功能：实现"眼在手上"(Eye-in-Hand)模型的手眼标定，同时计算标定板在世界坐标系中的位置
 *
 * 坐标系定义：
 * ┌─────────────────────────────────────────────────────────────────────────────┐
 * │ 坐标系层级关系                                                              │
 * ├─────────────────────────────────────────────────────────────────────────────┤
 * │                                                                             │
 * │   ┌─────────────────┐                                                     │
 * │   │  世界坐标系(World) │ ←── 地面固定坐标系                                  │
 * │   │  原点：云台旋转中心  │                                                    │
 * │   │  Z轴：垂直向上     │                                                    │
 * │   │  X轴：云台正前方   │                                                    │
 * │   │  Y轴：云台正右方   │                                                    │
 * │   └────────┬────────┘                                                     │
 * │            │                                                                │
 * │            │ R_gimbal2world (云台旋转，由IMU四元数计算)                       │
 * │            ▼                                                                │
 * │   ┌─────────────────┐                                                     │
 * │   │  云台坐标系(Gimbal) │ ←── 原点：云台旋转中心                             │
 * │   └────────┬────────┘                                                     │
 * │            │                                                                │
 * │            │ R_camera2gimbal (手眼标定结果)                                  │
 * │            ▼                                                                │
 * │   ┌─────────────────┐                                                     │
 * │   │  相机坐标系(Camera)│ ←── 原点：相机光心                                 │
 * │   │  Z轴：光轴方向   │                                                    │
 * │   │  X轴：图像X正向 │                                                    │
 * │   │  Y轴：图像Y负向 │                                                    │
 * │   └────────┬────────┘                                                     │
 * │            │                                                                │
 * │            │ 针孔模型投影                                                   │
 * │            ▼                                                                │
 * │   ┌─────────────────┐                                                     │
 * │   │  图像坐标系(Pixel)│ ←── 原点：图像左上角                                │
 * │   └─────────────────┘                                                     │
 * │                                                                             │
 * └─────────────────────────────────────────────────────────────────────────────┘
 *
 * 手眼标定数学模型：
 * ┌─────────────────────────────────────────────────────────────────────────────┐
 * │  "眼在手上"问题描述：                                                        │
 * │                                                                             │
 * │  已知：                                                                     │
 * │    ^C T_B: 相机→标定板（通过PnP求解）                                       │
 * │    ^W T_G: 世界→云台（通过IMU四元数计算）                                   │
 * │                                                                             │
 * │  求解：                                                                     │
 * │    ^G T_C: 云台→相机（手眼关系）- 相机固联在云台上                          │
 * │    ^W T_B: 世界→标定板（标定板位置）                                        │
 * │                                                                             │
 * │  数学方程：                                                                 │
 * │    ^G T_C · ^W T_G = ^W T_B · ^C T_B                                      │
 * │                                                                             │
 * │  其中 R_gimbal2imubody = [1,0,0, 0,1,0, 0,0,1] (单位矩阵)                  │
 * │  即云台坐标系与IMU机体坐标系对齐，无需额外旋转                                │
 * └─────────────────────────────────────────────────────────────────────────────┘
 *
 * 使用方法：
 *   ./calibrate_robotworld_handeye <输入文件夹> [-c <配置文件>]
 *   示例：./calibrate_robotworld_handeye assets/img_with_q -c configs/calibration.yaml
 */

#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <fstream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "tools/img_tools.hpp"
#include "tools/math_tools.hpp"

/**
 * @brief 命令行参数定义
 */
const std::string keys =
  "{help h usage ? |                          | 输出命令行参数说明}"
  "{config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{@input-folder | assets/img_with_q        | 输入文件夹路径   }";

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
 */
std::vector<cv::Point3f> centers_3d(const cv::Size & pattern_size, const float center_distance)
{
  std::vector<cv::Point3f> centers_3d;

  for (int i = 0; i < pattern_size.height; i++) {
    for (int j = 0; j < pattern_size.width; j++) {
      float x = (j - 0.5 * (pattern_size.width - 1)) * center_distance;
      float y = (i - 0.5 * (pattern_size.height - 1)) * center_distance;
      float z = 0;
      centers_3d.push_back({x, y, z});
    }
  }

  return centers_3d;
}

/**
 * @brief 从文件读取IMU四元数
 *
 * @param q_path 四元数文件路径
 * @return Eigen::Quaterniond 四元数
 *
 * @note 文件格式：w x y z（空格分隔）
 * @note 使用Eigen库的四元数类表示旋转
 */
Eigen::Quaterniond read_q(const std::string & q_path)
{
  std::ifstream q_file(q_path);
  double w, x, y, z;
  q_file >> w >> x >> y >> z;
  return {w, x, y, z};
}

/**
 * @brief 从指定文件夹加载标定数据
 *
 * @param input_folder 输入文件夹路径
 * @param config_path 配置文件路径
 * @param R_gimbal2imubody_data 输出：云台→IMU旋转矩阵数据
 * @param R_world2gimbal_list 输出：世界→云台旋转矩阵列表
 * @param t_world2gimbal_list 输出：世界→云台平移向量列表
 * @param rvecs 输出：标定板→相机旋转向量列表
 * @param tvecs 输出：标定板→相机平移向量列表
 *
 * @note 数据格式：
 *       - 图像：1.jpg, 2.jpg, ...
 *       - 四元数：1.txt, 2.txt, ... (格式: w x y z)
 * @note 云台→世界转换：R_gimbal2world = R_gimbal2imubody^T × R_imubody2imuabs × R_gimbal2imubody
 *       当R_gimbal2imubody为单位矩阵时，简化为：R_gimbal2world = R_imubody2imuabs
 */
void load(
  const std::string & input_folder, const std::string & config_path,
  std::vector<double> & R_gimbal2imubody_data, std::vector<cv::Mat> & R_world2gimbal_list,
  std::vector<cv::Mat> & t_world2gimbal_list, std::vector<cv::Mat> & rvecs,
  std::vector<cv::Mat> & tvecs)
{
  auto yaml = YAML::LoadFile(config_path);
  auto pattern_cols = yaml["pattern_cols"].as<int>(15);
  auto pattern_rows = yaml["pattern_rows"].as<int>(15);
  auto center_distance_mm = yaml["chessboard_square_size_mm"].as<double>(10.0);
  R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();

  cv::Size pattern_size(pattern_cols, pattern_rows);

  // 构建旋转矩阵
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_gimbal2imubody(R_gimbal2imubody_data.data());
  cv::Matx33d camera_matrix(camera_matrix_data.data());
  cv::Mat distort_coeffs(distort_coeffs_data);

  fmt::print("=== 手眼标定参数 ===\n");
  fmt::print("标定板参数：{}×{} 圆点，间距 {} mm\n", pattern_cols, pattern_rows, center_distance_mm);
  fmt::print("云台→IMU旋转矩阵：\n");
  fmt::print("  [{:.4f}, {:.4f}, {:.4f}]\n",
             R_gimbal2imubody_data[0], R_gimbal2imubody_data[1], R_gimbal2imubody_data[2]);
  fmt::print("  [{:.4f}, {:.4f}, {:.4f}]\n",
             R_gimbal2imubody_data[3], R_gimbal2imubody_data[4], R_gimbal2imubody_data[5]);
  fmt::print("  [{:.4f}, {:.4f}, {:.4f}]\n\n",
             R_gimbal2imubody_data[6], R_gimbal2imubody_data[7], R_gimbal2imubody_data[8]);

  // 检查是否为单位矩阵
  bool is_identity = true;
  for (int i = 0; i < 9; i++) {
    if (i % 4 == 0) {  // 对角线元素
      if (std::abs(R_gimbal2imubody_data[i] - 1.0) > 1e-6) is_identity = false;
    } else {
      if (std::abs(R_gimbal2imubody_data[i]) > 1e-6) is_identity = false;
    }
  }
  if (is_identity) {
    fmt::print("注意：R_gimbal2imubody 为单位矩阵\n");
    fmt::print("      云台→世界转换简化为：R_gimbal2world = R_imubody2imuabs\n\n");
  }

  fmt::print("正在加载标定数据...\n");

  int success_count = 0;
  int failure_count = 0;

  for (int i = 1; true; i++) {
    // 读取图片和对应四元数
    auto img_path = fmt::format("{}/{}.jpg", input_folder, i);
    auto q_path = fmt::format("{}/{}.txt", input_folder, i);
    auto img = cv::imread(img_path);
    Eigen::Quaterniond q = read_q(q_path);
    if (img.empty()) break;

    // 计算云台的欧拉角
    // R_imubody2imuabs: IMU机体到绝对坐标系的旋转（由四元数转换）
    Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();

    // R_gimbal2world: 云台→世界
    // 公式：R_gimbal2world = R_gimbal2imubody^T × R_imubody2imuabs × R_gimbal2imubody
    // 当R_gimbal2imubody为单位矩阵时，简化为：R_gimbal2world = R_imubody2imuabs
    Eigen::Matrix3d R_gimbal2world =
      R_gimbal2imubody.transpose() * R_imubody2imuabs * R_gimbal2imubody;
    Eigen::Vector3d ypr = tools::eulers(R_gimbal2world, 2, 1, 0) * 57.3;  // degree

    // 在图片上显示云台的欧拉角，用来检验R_gimbal2imubody是否正确
    auto drawing = img.clone();
    tools::draw_text(drawing, fmt::format("yaw   {:.2f}", ypr[0]), {40, 40}, {0, 0, 255});
    tools::draw_text(drawing, fmt::format("pitch {:.2f}", ypr[1]), {40, 80}, {0, 0, 255});
    tools::draw_text(drawing, fmt::format("roll  {:.2f}", ypr[2]), {40, 120}, {0, 0, 255});

    // 识别标定板
    std::vector<cv::Point2f> centers_2d;
    auto success = cv::findCirclesGrid(img, pattern_size, centers_2d);  // 默认是对称圆点图案

    // 显示识别结果
    cv::drawChessboardCorners(drawing, pattern_size, centers_2d, success);
    cv::resize(drawing, drawing, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("Press any to continue", drawing);
    cv::waitKey(10);

    // 输出识别结果
    fmt::print("[{}] yaw:{:.1f} pitch:{:.1f} roll:{:.1f} | {}\n",
                success ? "OK  " : "FAIL", ypr[0], ypr[1], ypr[2], img_path);
    if (success) {
      success_count++;
    } else {
      failure_count++;
      continue;
    }

    // 计算所需的数据
    Eigen::Matrix3d R_world2gimbal = R_gimbal2world.transpose();
    cv::Mat t_world2gimbal = (cv::Mat_<double>(3, 1) << 0, 0, 0);
    cv::Mat R_world2gimbal_cv;
    cv::eigen2cv(R_world2gimbal, R_world2gimbal_cv);
    cv::Mat rvec, tvec;
    auto centers_3d_ = centers_3d(pattern_size, center_distance_mm);
    cv::solvePnP(
      centers_3d_, centers_2d, camera_matrix, distort_coeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE);

    // 记录所需的数据
    R_world2gimbal_list.emplace_back(R_world2gimbal_cv);
    t_world2gimbal_list.emplace_back(t_world2gimbal);
    rvecs.emplace_back(rvec);
    tvecs.emplace_back(tvec);
  }

  fmt::print("\n加载完成：成功 {} 组，失败 {} 组\n", success_count, failure_count);

  // 数据验证
  if (success_count < 15) {
    fmt::print("警告：有效样本数少于15组，手眼标定结果可能不够准确\n");
    fmt::print("建议：采集20-30组不同姿态的数据\n\n");
  }
}

/**
 * @brief 将标定结果输出为YAML格式
 *
 * @param R_gimbal2imubody_data 云台→IMU旋转矩阵数据
 * @param R_camera2gimbal 相机→云台旋转矩阵
 * @param t_camera2gimbal 相机→云台平移向量
 * @param camera_ypr 相机相对于理想安装姿态的偏角
 * @param distance 标定板到世界坐标系原点的水平距离
 * @param board_ypr 标定板相对于竖直状态的偏角
 */
void print_yaml(
  const std::vector<double> & R_gimbal2imubody_data, const cv::Mat & R_camera2gimbal,
  const cv::Mat & t_camera2gimbal, const Eigen::Vector3d & camera_ypr, double distance,
  const Eigen::Vector3d & board_ypr)
{
  YAML::Emitter result;
  std::vector<double> R_camera2gimbal_data(
    R_camera2gimbal.begin<double>(), R_camera2gimbal.end<double>());
  std::vector<double> t_camera2gimbal_data(
    t_camera2gimbal.begin<double>(), t_camera2gimbal.end<double>());

  result << YAML::BeginMap;
  result << YAML::Key << "R_gimbal2imubody";
  result << YAML::Value << YAML::Flow << R_gimbal2imubody_data;
  result << YAML::Newline;
  result << YAML::Newline;
  result << YAML::Comment(fmt::format(
    "相机同理想情况的偏角: yaw{:.2f} pitch{:.2f} roll{:.2f} degree", camera_ypr[0], camera_ypr[1],
    camera_ypr[2]));
  result << YAML::Newline;
  result << YAML::Comment(fmt::format("标定板到世界坐标系原点的水平距离: {:.2f} m", distance));
  result << YAML::Newline;
  result << YAML::Comment(fmt::format(
    "标定板同竖直摆放时的偏角: yaw{:.2f} pitch{:.2f} roll{:.2f} degree", board_ypr[0], board_ypr[1],
    board_ypr[2]));
  result << YAML::Key << "R_camera2gimbal";
  result << YAML::Value << YAML::Flow << R_camera2gimbal_data;
  result << YAML::Key << "t_camera2gimbal";
  result << YAML::Value << YAML::Flow << t_camera2gimbal_data;
  result << YAML::Newline;
  result << YAML::EndMap;

  fmt::print("\n================== 标定结果 ==================\n");
  fmt::print("{}\n", result.c_str());

  // 打印详细说明
  fmt::print("\n=== 标定结果说明 ===\n\n");

  fmt::print("1. 云台→IMU旋转矩阵 R_gimbal2imubody:\n");
  fmt::print("   用于将IMU测量的姿态转换到云台坐标系\n\n");

  fmt::print("2. 相机→云台旋转矩阵 R_camera2gimbal:\n");
  fmt::print("   用于将相机坐标系转换到云台坐标系\n");
  fmt::print("   物理含义：相机相对于云台的安装角度\n\n");

  fmt::print("3. 相机→云台平移向量 t_camera2gimbal (m):\n");
  fmt::print("   [{:.4f}, {:.4f}, {:.4f}]\n\n",
             t_camera2gimbal_data[0], t_camera2gimbal_data[1], t_camera2gimbal_data[2]);

  fmt::print("4. 相机相对于理想安装姿态的偏角:\n");
  fmt::print("   yaw   = {:.2f}°\n", camera_ypr[0]);
  fmt::print("   pitch = {:.2f}°\n", camera_ypr[1]);
  fmt::print("   roll  = {:.2f}°\n\n", camera_ypr[2]);

  fmt::print("5. 标定板位置:\n");
  fmt::print("   到原点距离 = {:.2f} m\n", distance);
  fmt::print("   偏角: yaw={:.2f}° pitch={:.2f}° roll={:.2f}°\n\n",
             board_ypr[0], board_ypr[1], board_ypr[2]);

  fmt::print("=== 坐标系转换公式 ===\n\n");
  fmt::print("1. 云台→世界: R_gimbal2world = R_imubody2imuabs (当R_gimbal2imubody为单位矩阵时)\n\n");
  fmt::print("2. 相机→云台: xyz_gimbal = R_camera2gimbal × xyz_camera + t_camera2gimbal\n\n");
  fmt::print("3. 云台→世界: xyz_world = R_gimbal2world × xyz_gimbal\n");
}

/**
 * @brief 主函数 - 机器人世界坐标系手眼标定程序入口
 *
 * @param argc 命令行参数数量
 * @param argv 命令行参数列表
 * @return int 程序退出码
 *
 * @note 配置文件必须包含：
 *       - R_gimbal2imubody: 云台→IMU旋转矩阵
 *       - camera_matrix: 相机内参矩阵
 *       - distort_coeffs: 畸变系数
 *       - pattern_cols/rows: 标定板尺寸
 *       - center_distance_mm: 圆点间距
 */
int main(int argc, char * argv[])
{
  fmt::print("========================================================\n");
  fmt::print("     机器人世界坐标系手眼标定程序 v1.0\n");
  fmt::print("     Eye-in-Hand Calibration\n");
  fmt::print("========================================================\n\n");

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
  std::vector<double> R_gimbal2imubody_data;
  std::vector<cv::Mat> R_world2gimbal_list, t_world2gimbal_list;
  std::vector<cv::Mat> rvecs, tvecs;
  load(
    input_folder, config_path, R_gimbal2imubody_data, R_world2gimbal_list, t_world2gimbal_list,
    rvecs, tvecs);

  // 检查数据有效性
  if (rvecs.empty()) {
    fmt::print("错误：没有有效的标定数据！\n");
    return -1;
  }

  fmt::print("\n正在执行手眼标定...\n");

  // 手眼标定
  cv::Mat R_gimbal2camera, t_gimbal2camera;
  cv::Mat R_world2board, t_world2board;
  cv::calibrateRobotWorldHandEye(
    rvecs, tvecs, R_world2gimbal_list, t_world2gimbal_list, R_world2board, t_world2board,
    R_gimbal2camera, t_gimbal2camera);
  t_gimbal2camera /= 1e3;  // mm to m
  t_world2board /= 1e3;    // mm to m

  fmt::print("标定完成！\n\n");

  // 计算所需的数据
  cv::Mat R_camera2gimbal, t_camera2gimbal;
  cv::Mat R_board2world, t_board2world;
  cv::transpose(R_gimbal2camera, R_camera2gimbal);
  cv::transpose(R_world2board, R_board2world);
  t_camera2gimbal = -R_camera2gimbal * t_gimbal2camera;
  t_board2world = -R_board2world * t_world2board;

  // 计算相机同理想情况的偏角
  Eigen::Matrix3d R_camera2gimbal_eigen;
  cv::cv2eigen(R_camera2gimbal, R_camera2gimbal_eigen);

  // 理想安装姿态：相机光轴与云台Z轴重合
  // R_gimbal2ideal: [[0, -1, 0], [0, 0, -1], [1, 0, 0]]
  Eigen::Matrix3d R_gimbal2ideal{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}};
  Eigen::Matrix3d R_camera2ideal = R_gimbal2ideal * R_camera2gimbal_eigen;
  Eigen::Vector3d camera_ypr = tools::eulers(R_camera2ideal, 1, 0, 2) * 57.3;  // degree

  // 计算标定板到世界坐标系原点的水平距离
  auto x = t_board2world.at<double>(0);
  auto y = t_board2world.at<double>(1);
  auto distance = std::sqrt(x * x + y * y);

  // 计算标定板同竖直摆放时的偏角
  Eigen::Matrix3d R_board2world_eigen;
  cv::cv2eigen(R_board2world, R_board2world_eigen);
  Eigen::Vector3d board_ypr = tools::eulers(R_board2world_eigen, 2, 1, 0) * 57.3;  // degree

  // 输出yaml
  print_yaml(
    R_gimbal2imubody_data, R_camera2gimbal, t_camera2gimbal, camera_ypr, distance, board_ypr);

  fmt::print("========================================================\n");
  fmt::print("手眼标定程序结束\n");
  fmt::print("========================================================\n");

  cv::destroyAllWindows();
  return 0;
}
