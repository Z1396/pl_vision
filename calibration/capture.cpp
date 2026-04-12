/**
 * @file capture.cpp
 * @brief 图像与IMU数据同步采集程序
 *
 * 功能：采集标定所需的图像和IMU四元数数据
 *
 * 采集流程：
 * 1. 初始化相机和CBoard（控制板）
 * 2. 实时显示相机画面和IMU姿态
 * 3. 自动检测标定板并显示识别结果
 * 4. 用户按's'键保存当前帧，按'q'键退出
 *
 * 输出数据格式：
 * - 图像：{序号}.jpg
 * - 四元数：{序号}.txt (格式: w x y z)
 *
 * 使用方法：
 *   ./capture [-c <配置文件>] [-o <输出文件夹>]
 *   示例：./capture -c configs/calibration.yaml -o assets/img_with_q
 */

#include <fmt/core.h>

#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

/**
 * @brief 命令行参数定义
 */
const std::string keys =
  "{help h usage ? |                          | 输出命令行参数说明}"
  "{config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{output-folder o |      assets/img_with_q   | 输出文件夹路径   }";

/**
 * @brief 将四元数写入文件
 *
 * @param q_path 输出文件路径
 * @param q 四元数
 *
 * @note 文件格式：w x y z（空格分隔）
 * @note 四元数存储顺序为[w, x, y, z]
 */
void write_q(const std::string q_path, const Eigen::Quaterniond & q)
{
  std::ofstream q_file(q_path);
  Eigen::Vector4d xyzw = q.coeffs();
  // 输出顺序为 w x y z
  q_file << fmt::format("{} {} {} {}", xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
  q_file.close();
}

/**
 * @brief 主采集循环
 *
 * @param config_path 配置文件路径
 * @param can CAN总线接口名称
 * @param output_folder 输出文件夹路径
 *
 * @note 标定板要求：15×15 圆点标定板
 * @note 按's'保存当前帧，按'q'退出程序
 */
void capture_loop(
  const std::string & config_path, const std::string & can, const std::string & output_folder)
{
  // 初始化CBoard（读取IMU数据）和相机
  io::CBoard cboard(config_path);
  io::Camera camera(config_path);
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  int count = 0;
  fmt::print("\n=== 采集模式 ===\n");
  fmt::print("按 's' 保存当前帧\n");
  fmt::print("按 'q' 退出程序\n");
  fmt::print("按其他键跳过当前帧\n\n");

  // 打印IMU坐标系说明
  fmt::print("=== IMU坐标系说明 ===\n");
  fmt::print("Z: 偏航角 (yaw)\n");
  fmt::print("Y: 俯仰角 (pitch)\n");
  fmt::print("X: 滚转角 (roll)\n\n");

  while (true) {
    // 1. 同步读取相机图像和时间戳
    camera.read(img, timestamp);

    // 2. 获取对应时刻的IMU四元数
    Eigen::Quaterniond q = cboard.imu_at(timestamp);

    // 3. 在图像上显示欧拉角
    // 用途：判断IMU的姿态是否存在零漂
    auto img_with_ypr = img.clone();
    Eigen::Vector3d zyx = tools::eulers(q, 2, 1, 0) * 57.3;  // degree
    tools::draw_text(img_with_ypr, fmt::format("Z {:.2f}", zyx[0]), {40, 40}, {0, 0, 255});
    tools::draw_text(img_with_ypr, fmt::format("Y {:.2f}", zyx[1]), {40, 80}, {0, 0, 255});
    tools::draw_text(img_with_ypr, fmt::format("X {:.2f}", zyx[2]), {40, 120}, {0, 0, 255});

    // 4. 检测标定板（15×15圆点标定板）
    std::vector<cv::Point2f> centers_2d;
    // 使用圆点标定板检测（与calibrate_robotworld_handeye.cpp一致）
    auto success = cv::findCirclesGrid(img, cv::Size(15, 15), centers_2d);

    // 5. 显示识别结果
    cv::drawChessboardCorners(img_with_ypr, cv::Size(15, 15), centers_2d, success);
    cv::resize(img_with_ypr, img_with_ypr, {}, 0.5, 0.5);  // 显示时缩小图片尺寸

    // 6. 显示帧号和检测状态
    std::string status = success ? "OK" : "FAIL";
    tools::draw_text(img_with_ypr, fmt::format("Frame: {} [{}]", count, status),
                     {40, 160}, success ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255));

    // 显示操作提示
    tools::draw_text(img_with_ypr, "Press: s=save, q=quit", {40, 200}, {255, 255, 255});

    // 7. 显示图像并等待按键
    cv::imshow("Press s to save, q to quit", img_with_ypr);
    auto key = cv::waitKey(1);

    // 按'q'退出程序
    if (key == 'q' || key == 'Q') {
      fmt::print("\n用户退出程序\n");
      break;
    }
    // 按's'保存当前帧
    else if (key == 's' || key == 'S') {
      count++;
      auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
      auto q_path = fmt::format("{}/{}.txt", output_folder, count);

      // 保存图像和四元数
      cv::imwrite(img_path, img);
      write_q(q_path, q);

      // 输出保存状态
      if (success) {
        fmt::print("[{}.保存成功] yaw:{:.1f} pitch:{:.1f} roll:{:.1f}\n",
                    count, zyx[0], zyx[1], zyx[2]);
      } else {
        fmt::print("[{}.警告-标定板未识别] yaw:{:.1f} pitch:{:.1f} roll:{:.1f}\n",
                    count, zyx[0], zyx[1], zyx[2]);
      }
    }
    // 按其他键不做任何操作
  }

  // 离开作用域时，camera和cboard会自动关闭
  fmt::print("\n共保存 {} 组数据\n", count);
  fmt::print("输出目录: {}\n", output_folder);
}

/**
 * @brief 主函数 - 数据采集程序入口
 *
 * @param argc 命令行参数数量
 * @param argv 命令行参数列表
 * @return int 程序退出码
 *
 * @note 配置文件格式 (calibration.yaml):
 *   camera_name: "hikrobot"    # 相机类型
 *   exposure_ms: 10            # 曝光时间（毫秒）
 *   gain: 10.0                # 增益
 *   quaternion_canid: 0x100   # 四元数CAN ID
 *   can_interface: "can0"     # CAN总线接口
 *
 * @note 注意事项：
 *   1. 四元数输出顺序为 w x y z
 *   2. 标定板尺寸必须为15×15
 *   3. 采集时应覆盖云台的不同姿态（至少15-20组）
 */
int main(int argc, char * argv[])
{
  fmt::print("===========================================\n");
  fmt::print("     图像与IMU同步采集程序 v1.0\n");
  fmt::print("===========================================\n\n");

  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>("config-path");
  auto output_folder = cli.get<std::string>("output-folder");

  fmt::print("配置文件: {}\n", config_path);
  fmt::print("输出文件夹: {}\n\n", output_folder);

  // 创建输出文件夹（如果不存在）
  if (!std::filesystem::exists(output_folder)) {
    std::filesystem::create_directory(output_folder);
    fmt::print("创建输出文件夹: {}\n\n", output_folder);
  }

  tools::logger()->info("默认标定板尺寸为15列15行");

  // 主循环，保存图片和对应四元数
  capture_loop(config_path, "can0", output_folder);

  fmt::print("\n===========================================\n");
  fmt::print("采集程序结束\n");
  fmt::print("===========================================\n");

  return 0;
}
