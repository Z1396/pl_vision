#include <fmt/core.h>               // 格式化输出

#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>        // 数据序列化（用于调试输出）
#include <opencv2/opencv.hpp>
#include "tasks/auto_aim/detector.hpp" //传统识别
#include "tasks/auto_aim/aimer.hpp" // 瞄准器
#include "tasks/auto_aim/solver.hpp"// 坐标解算器
#include "tasks/auto_aim/tracker.hpp"// 目标跟踪器
#include "tasks/auto_aim/yolo.hpp"  // 装甲板检测器
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"        // 数据可视化工具

const std::string keys =
  "{help h usage ? |                   | 输出命令行参数说明 }"
  "{config-path c  | configs/standard3.yaml | yaml配置文件的路径}"
  "{start-index s  | 0                | 视频起始帧下标    }"
  "{end-index e    | 280               | 视频结束帧下标    }"
  "{@input-path    | assets/demo/3m | avi和txt文件的路径}";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) 
  {
    cli.printMessage();
    return 0;
  }

  auto input_path = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");

  tools::Plotter plotter;       // 初始化绘图工具，用于记录算法数据（如目标位置、指令）
  tools::Exiter exiter;         // 初始化退出检测器，监听程序终止信号（如Ctrl+C）

  // 拼接视频和数据文件路径
  auto video_path = fmt::format("{}.avi", input_path);  // 离线视频路径（.avi）
  auto text_path = fmt::format("{}.txt", input_path);   // 离线数据路径（.txt，含IMU等信息）
  cv::VideoCapture video(video_path);  // 打开视频文件
  std::ifstream text(text_path);       // 打开数据文件

  auto_aim::Detector yolo(config_path, false);          // 初始化YOLO检测器（加载模型和配置）
  auto_aim::Solver solver(config_path);      // 初始化解算器（加载相机参数、坐标转换配置）
  auto_aim::Tracker tracker(config_path, solver);  // 初始化追踪器（依赖解算器进行坐标转换）
  auto_aim::Aimer aimer(config_path);        // 初始化瞄准器（加载弹道参数、补偿配置）

  cv::Mat img, drawing;  // 图像变量（当前帧图像、绘制调试信息的图像）
  auto t0 = std::chrono::steady_clock::now();  // 程序启动时间（用于模拟时间戳）

  auto_aim::Target last_target;  // 上一帧的目标信息
  io::Command last_command;      // 上一帧的控制指令
  double last_t = -1;            // 上一帧的时间戳（用于计算时间差）

  // -------------------------- 新增：帧率计算相关变量 --------------------------
  std::chrono::steady_clock::time_point last_frame_time = std::chrono::steady_clock::now();  // 上一帧开始时间
  double frame_delay_ms = 0.0;  // 每帧总耗时（毫秒）
  double fps = 0.0;             // 实时帧率

  // 定位视频到起始帧
  video.set(cv::CAP_PROP_POS_FRAMES, start_index);
  // 跳过数据文件中起始帧之前的内容
  for (int i = 0; i < start_index; i++) 
  {
    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;  // 读取但不处理，仅移动文件指针
  }

  // 循环处理每一帧，直到退出信号或视频结束
  for (int frame_count = start_index; !exiter.exit(); frame_count++) 
  {
    // -------------------------- 记录当前帧开始时间 --------------------------
    auto current_frame_start = std::chrono::steady_clock::now();

    if (end_index > 0 && frame_count > end_index) break;  // 到达结束帧则退出

    // 读取当前帧图像和数据
    video.read(img);  // 从视频中读取一帧图像
    if (img.empty()) break;  // 视频读取完毕则退出

    // 从数据文件中读取当前帧的时间和IMU姿态（w,x,y,z为四元数）
    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
    // 生成模拟的绝对时间戳（基准时间+t）
    auto timestamp = t0 + std::chrono::microseconds(int(t * 1e6));

    /// 自瞄核心逻辑
    // 设置解算器的云台到世界坐标系的旋转矩阵（从IMU四元数转换）
    solver.set_R_gimbal2world({w, x, y, z});

    // 1. YOLO检测装甲板
    auto yolo_start = std::chrono::steady_clock::now();  // 记录检测开始时间
    auto armors = yolo.detect(img, frame_count);  // 输入图像，输出检测到的装甲板列表

    // 2. 追踪目标（关联历史目标，更新状态）
    auto tracker_start = std::chrono::steady_clock::now();  // 记录追踪开始时间
    auto targets = tracker.track(armors, timestamp);  // 输入装甲板和时间戳，输出追踪目标

    // 3. 计算瞄准指令（根据目标预测位置和弹道）
    auto aimer_start = std::chrono::steady_clock::now();  // 记录瞄准开始时间
    auto command = aimer.aim(targets, timestamp, 27, false);  // 输入目标、时间戳、弹速，输出指令

    // 判断是否满足发射条件（目标存在+瞄准稳定）
    if(
      !targets.empty() && aimer.debug_aim_point.valid &&
      std::abs(command.yaw - last_command.yaw) * 57.3 < 2)  // 航向角变化小于2度（稳定）
    {
      command.shoot = true;  // 满足条件则允许发射
    }   
    if(command.control) 
    {
      last_command = command;  // 若指令有效，更新上一帧指令
    }
    
    /// 调试输出
    auto finish = std::chrono::steady_clock::now();  // 记录当前帧处理结束时间

    // -------------------------- 计算每帧总耗时和FPS --------------------------
    frame_delay_ms = std::chrono::duration_cast<std::chrono::duration<double>>(finish - current_frame_start).count() * 1e3;  // 总耗时（毫秒）
    fps = 1000.0 / frame_delay_ms;  // 帧率 = 1000ms / 每帧耗时（毫秒）
    last_frame_time = current_frame_start;  // 更新上一帧开始时间

    // 输出各模块耗时 + 每帧总耗时 + FPS（毫秒）
    tools::logger()->info(
      "[{}] 总耗时: {:.2f}ms | FPS: {:.2f} | yolo: {:.2f}ms, tracker: {:.2f}ms, aimer: {:.2f}ms", 
      frame_count,
      frame_delay_ms,  // 新增：每帧总耗时
      fps,             // 新增：实时帧率
      std::chrono::duration_cast<std::chrono::duration<double>>(tracker_start - yolo_start).count() * 1e3,  // YOLO耗时
      std::chrono::duration_cast<std::chrono::duration<double>>(aimer_start - tracker_start).count() * 1e3,  // Tracker耗时
      std::chrono::duration_cast<std::chrono::duration<double>>(finish - aimer_start).count() * 1e3          // Aimer耗时
    );

    // // 在图像上绘制控制指令（yaw/pitch/是否发射）
    // tools::draw_text(
    //   img,
    //   fmt::format(
    //     "command is {},{:.2f},{:.2f},shoot:{}", command.control, command.yaw * 57.3,  // 角度转换为度
    //     command.pitch * 57.3, command.shoot),
    //   {10, 60}, {154, 50, 205}  // 位置和颜色
    // );

    // // 绘制云台当前yaw角（从四元数转换为欧拉角）
    // Eigen::Quaternion gimbal_q = {w, x, y, z};
    // tools::draw_text(
    //   img,
    //   fmt::format(
    //     "gimbal yaw{:.2f} | gimbal pitch{:.2f}", (tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0) * 57.3)[0], (tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0) * 57.3)[1]),
    //   {10, 90}, {255, 255, 255}
    // );

    // // -------------------------- 新增：在图像上绘制总耗时和FPS --------------------------
    // tools::draw_text(
    //   img,
    //   fmt::format("Frame Time: {:.1f}ms | FPS: {:.1f}", frame_delay_ms, fps),
    //   {10, 120}, {0, 255, 255}  // 位置：(10,120)，颜色：黄色
    // );

    tools::draw_text(img, fmt::format("[FPS:{:.3}]",fps ), {100, 150}, {255, 255, 255});

     nlohmann::json data;  // 用于记录当前帧的算法数据

    // // 记录装甲板原始观测数据
    data["armor_num"] = armors.size();  // 检测到的装甲板数量
    if (!armors.empty()) {
      const auto & armor = armors.front();  // 取第一个装甲板
      data["armor_x"] = armor.xyz_in_world[0];  // 世界坐标系x坐标
      data["armor_y"] = armor.xyz_in_world[1];  // 世界坐标系y坐标
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;  // 航向角（度）
      data["armor_yaw_raw"] = armor.yaw_raw * 57.3;  // 原始航向角
      data["armor_center_x"] = armor.center_norm.x;  // 归一化中心x
      data["armor_center_y"] = armor.center_norm.y;  // 归一化中心y
    }

    // -------------------------- 新增：记录总耗时和FPS到日志数据 --------------------------
    data["frame_delay_ms"] = frame_delay_ms;
    data["fps"] = fps;

    // 记录云台和指令的yaw角
    Eigen::Quaternion q{w, x, y, z};
    auto yaw = tools::eulers(q, 2, 1, 0)[0];  // 云台yaw角（弧度）
    auto pitch = tools::eulers(q, 2, 1, 0)[1];  // 云台yaw角（弧度）
    data["gimbal_yaw"] = yaw * 57.3;  // 转换为度
    data["gimbal_pitch"] = pitch * 57.3;  // 转换为度
    data["cmd_yaw"] = command.yaw * 57.3;  // 指令yaw角（度）
    data["cmd_pitch"] = command.pitch * 57.3;  // 指令yaw角（度）
    data["shoot"] = command.shoot;  // 是否发射

    // // 记录追踪器和卡尔曼滤波的内部数据（若有目标）
     if (!targets.empty()) 
     {
      auto target = targets.front();
      tools::draw_text(img, fmt::format("[{}] ", target.outpost_state()), {10, 30}, {255, 255, 255});  

      // 首次处理目标时初始化历史数据
      if (last_t == -1) 
      {
        last_target = target;
        last_t = t;
        continue;
      }

      // 绘制装甲板在图像上的投影（绿色）
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) 
      {
        auto image_points = solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});  // 绿色点
      }

      // 绘制瞄准点在图像上的投影（红色）
      auto aim_point = aimer.debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto image_points = solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      if (aim_point.valid) tools::draw_points(img, image_points, {0, 0, 255});  // 红色点

      // 记录卡尔曼滤波的状态（位置、速度、尺寸等）
      Eigen::VectorXd x = target.ekf_x();  // 滤波后的状态向量
      data["x"] = x[0];    // x位置
      data["vx"] = x[1];   // x方向速度
      data["y"] = x[2];    // y位置
      data["vy"] = x[3];   // y方向速度
      data["z"] = x[4];    // z位置
      data["vz"] = x[5];   // z方向速度
      data["a"] = x[6] * 57.3;  // 角度（度）
      data["w"] = x[7];    // 旋转速度
      data["r"] = x[8];    // 目标半径
      data["l"] = x[9];    // 目标长度
      data["h"] = x[10];   // 目标高度
      data["last_id"] = target.last_id;  // 目标ID

      // 记录卡尔曼滤波的残差和检验数据（用于评估滤波效果）
      data["residual_yaw"] = target.ekf().data.at("residual_yaw");  // yaw残差
      data["residual_pitch"] = target.ekf().data.at("residual_pitch");  // pitch残差
      data["residual_distance"] = target.ekf().data.at("residual_distance");  // 距离残差
      data["residual_angle"] = target.ekf().data.at("residual_angle");  // 角度残差
      data["nis"] = target.ekf().data.at("nis");  // 归一化 innovations 平方
      data["nees"] = target.ekf().data.at("nees");  // 归一化估计误差平方
      data["nis_fail"] = target.ekf().data.at("nis_fail");  // NIS检验失败次数
      data["nees_fail"] = target.ekf().data.at("nees_fail");  // NEES检验失败次数
      data["recent_nis_failures"] = target.ekf().data.at("recent_nis_failures");  // 近期NIS失败次数
    }

    // 将当前帧数据交给plotter记录（后续可生成曲线）
    plotter.plot(data);

    // 缩小图像尺寸（便于显示）并展示
    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("reprojection", img);
    // 等待30ms，若按下'q'则退出
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  return 0;
}