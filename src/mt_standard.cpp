#include <chrono>       // 用于时间相关操作（如时间点、时间间隔）
#include <opencv2/opencv.hpp>  // OpenCV库，用于图像处理和显示
#include <thread>       // 用于多线程编程

// 自定义IO模块头文件：相机、IMU、控制板等硬件接口
#include "io/camera.hpp"
#include "io/dm_imu/dm_imu.hpp"

// 自瞄任务相关头文件：检测器、解算器、追踪器等算法模块
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"

// 打符任务相关头文件：打符专用的检测器、解算器等
#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"
#include "tasks/auto_buff/buff_type.hpp"

// 工具类头文件：退出控制、图像工具、日志、数学工具等
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

// 命令行参数定义：指定配置文件路径和帮助信息
const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | /home/pldx/Desktop/sp_vision_25-main/configs/standard3.yaml | yaml配置文件路径 }";

// 简化chrono库的命名空间使用（如1ms代表1毫秒）
using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  // 解析命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");  // 获取配置文件路径
  if (cli.has("help") || !cli.has("@config-path")) {  // 若请求帮助或未指定配置文件
    cli.printMessage();  // 打印参数说明
    return 0;
  }

  // 初始化工具类
  tools::Exiter exiter;        // 程序退出控制器（处理退出信号）
  tools::Plotter plotter;      // 数据绘图工具（用于调试数据可视化）
  tools::Recorder recorder;    // 数据录制器（用于保存图像、传感器数据等）

  // 初始化硬件接口
  io::Camera camera(config_path);       // 相机接口（读取图像）
  io::CBoard cboard(config_path);       // 控制板接口（与下位机通信，获取IMU、发送指令）

  // 初始化自瞄算法模块
  auto_aim::multithread::MultiThreadDetector detector(config_path, true);  // 多线程装甲板检测器
  auto_aim::Solver solver(config_path);                              // 坐标解算器（将图像坐标转换为世界坐标）
  auto_aim::Tracker tracker(config_path, solver);                    // 目标追踪器（使用EKF等算法预测目标运动）
  auto_aim::Aimer aimer(config_path);                                // 瞄准器（计算云台控制量）
  auto_aim::Shooter shooter(config_path);                            // 发射器（控制发射时机）

  // 初始化打符算法模块（能量机关相关）
  auto_buff::Buff_Detector buff_detector(config_path);  // 能量机关检测器
  auto_buff::Solver buff_solver(config_path);            // 能量机关解算器
  auto_buff::SmallTarget buff_small_target;              // 小能量机关目标
  auto_buff::BigTarget buff_big_target;                  // 大能量机关目标
  auto_buff::Aimer buff_aimer(config_path);              // 能量机关瞄准器

  // 初始化命令生成器（多线程环境下生成控制命令）
  auto_aim::multithread::CommandGener commandgener(shooter, aimer, cboard, plotter, true);

  // 模式控制变量：原子变量确保多线程安全访问当前工作模式
  std::atomic<io::Mode> mode{io::Mode::idle};  // 当前模式（默认空闲）
  auto last_mode{io::Mode::idle};              // 上一帧模式（用于检测模式切换）

  // 初始化第一个时间点
  std::chrono::steady_clock::time_point t_prev = std::chrono::steady_clock::now();

  // 启动检测线程：独立线程处理图像检测，避免阻塞主逻辑
  auto detect_thread = std::thread([&]() 
  {
    cv::Mat img;  // 存储当前帧图像
    std::chrono::steady_clock::time_point t;  // 图像时间戳

    while (!exiter.exit())  // 循环直到程序退出
    {
      // 仅在空闲模式（实际代码中可能应为自瞄模式，此处可能是调试设置）下处理图像
      if (mode.load() == io::Mode::idle) 
      {
        camera.read(img, t);          // 从相机读取图像和时间戳
        detector.push(img, t);        // 将图像和时间戳推入检测器处理队列
      } else
        continue;  // 非目标模式下不处理图像
    }
  });

  // 主循环：处理算法逻辑和模式切换
  while (!exiter.exit()) 
  {
   
    mode = cboard.mode;  // 从控制板获取当前模式（如自瞄、打符、空闲等）

    // 检测模式切换并打印日志
    if (last_mode != mode) 
    {
      tools::logger()->info("Switch to {}", io::MODES[mode]);  // 打印模式切换信息
      last_mode = mode.load();  // 更新上一帧模式
    }
    

    /// 自瞄模式逻辑（当前代码中误用idle模式，实际应为auto_aim模式）
    // if (mode.load() == io::Mode::idle) 
    // {
      // 从检测器获取处理后的图像、装甲板检测结果和时间戳
      auto [img, armors, t] = detector.debug_pop();
      // 根据图像时间戳获取IMU数据（滞后1毫秒，补偿时间同步误差）
      Eigen::Quaterniond q = cboard.imu_at(t - 1ms);
      
      // 可选：录制图像、IMU数据和时间戳（调试用）
      // recorder.record(img, q, t);

      // 设置云台到世界坐标系的旋转矩阵（用于坐标转换）
      solver.set_R_gimbal2world(q);

      // 将旋转矩阵转换为欧拉角（yaw, pitch, roll），单位弧度
      Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
      // 打印云台角度（转换为度）
      tools::logger()->info("z{:.2f} y{:.2f} x{:.2f} degree", ypr[0], ypr[1], ypr[2]);

      // 追踪装甲板目标（使用EKF预测目标状态）
      auto targets = tracker.track(armors, t);

      // 将目标信息推送给命令生成器，生成并发送控制指令
      commandgener.push(targets, t, cboard.bullet_speed, ypr);

      // 计算FPS
      std::chrono::steady_clock::time_point t_current = std::chrono::steady_clock::now();
      auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(t_current - t_prev).count();
      t_prev = t_current;  // 更新上一帧时间点
      
      if (dt > 0) 
      {  // 避免除以零
          tools::logger()->info("FPS: {:.2f}", 1.0 / dt);
      }

      nlohmann::json data;

      if (!targets.empty()) 
      {
        auto target = targets.front();
        tools::draw_text(img, fmt::format("[{}] ", target.outpost_state()), {10, 30}, {255, 255, 255}); 

        // 当前帧target更新后
        std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
        for (const Eigen::Vector4d & xyza : armor_xyza_list) 
        {
          auto image_points =
            solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
          tools::draw_points(img, image_points, {0, 255, 0});
        }

        // aimer瞄准位置
        auto aim_point = aimer.debug_aim_point;
        Eigen::Vector4d aim_xyza = aim_point.xyza;
        auto image_points =
          solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
        if (aim_point.valid)
          tools::draw_points(img, image_points, {0, 0, 255});//b,g,r
        else
          tools::draw_points(img, image_points, {255, 0, 0});

        

        // 装甲板原始观测数据
        data["armor_num"] = armors.size();
        if (!armors.empty()) 
        {
          auto min_x = 1e10;
          auto & armor = armors.front();
          for (auto & a : armors) 
          {
            if (a.center.x < min_x) 
            {
              min_x = a.center.x;
              armor = a;
            }
          }  //always left
          solver.solve(armor);
          data["distance"] = armor.ypd_in_world[2];
          data["armor_x"] = armor.xyz_in_world[0];
          data["armor_y"] = armor.xyz_in_world[1];
          data["armor_z"] = armor.xyz_in_world[2] * 100;
          data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
          data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
        }

        // 观测器内部数据
        Eigen::VectorXd x = target.ekf_x();
        data["x"] = x[0];
        data["vx"] = x[1];
        data["y"] = x[2];
        data["vy"] = x[3];
        data["z"] = x[4] * 100;
        data["vz"] = x[5];
        data["a"] = x[6] * 57.3;
        data["w"] = x[7];
        data["r"] = x[8];
        data["l"] = x[9];
        data["h"] = x[10];
        data["last_id"] = target.last_id;

        // 卡方检验数据
        data["residual_yaw"] = target.ekf().data.at("residual_yaw");
        data["residual_pitch"] = target.ekf().data.at("residual_pitch");
        data["residual_distance"] = target.ekf().data.at("residual_distance");
        data["residual_angle"] = target.ekf().data.at("residual_angle");
        data["nis"] = target.ekf().data.at("nis");
        data["nees"] = target.ekf().data.at("nees");
        data["nis_fail"] = target.ekf().data.at("nis_fail");
        data["nees_fail"] = target.ekf().data.at("nees_fail");
        data["recent_nis_failures"] = target.ekf().data.at("recent_nis_failures");
      }

      // 云台响应情况
      data["gimbal_yaw"] = ypr[0] * 57.3;
      data["gimbal_pitch"] = ypr[1] * 57.3;
      data["gimbal_roll"] = ypr[2] * 57.3;
      data["bullet_speed"] = cboard.bullet_speed;
      plotter.plot(data);

    //}

    /// 打符模式逻辑（小能量机关或大能量机关）
    // else if (mode.load() == io::Mode::small_buff || mode.load() == io::Mode::big_buff) 
    // {
    //   cv::Mat img;  // 存储当前帧图像
    //   Eigen::Quaterniond q;  // 存储IMU姿态
    //   std::chrono::steady_clock::time_point t;  // 时间戳

    //   //camera.read(img, t);          // 读取图像和时间戳
    //   q = cboard.imu_at(t - 1ms);   // 获取对应时间的IMU数据

    //   // 可选：录制数据
    //   // recorder.record(img, q, t);

    //   // 设置能量机关解算器的坐标系转换矩阵
    //   buff_solver.set_R_gimbal2world(q);

    //   // 检测能量机关（识别能量机关装甲板）
    //   auto power_runes = buff_detector.detect(img);

    //   // 解算能量机关的位置和姿态
    //   buff_solver.solve(power_runes);

    //   // 根据模式（小/大能量机关）计算控制命令
    //   io::Command buff_command;
    //   if (mode.load() == io::Mode::small_buff) 
    //   {
    //     buff_small_target.get_target(power_runes, t);  // 获取小能量机关目标
    //     auto target_copy = buff_small_target;
    //     // 计算瞄准命令
    //     buff_command = buff_aimer.aim(target_copy, t, cboard.bullet_speed, true);
    //   } else if (mode.load() == io::Mode::big_buff) 
    //   {
    //     buff_big_target.get_target(power_runes, t);  // 获取大能量机关目标
    //     auto target_copy = buff_big_target;
    //     // 计算瞄准命令
    //     buff_command = buff_aimer.aim(target_copy, t, cboard.bullet_speed, true);
    //   }
    //   cboard.send(buff_command);  // 发送打符控制命令到下位机

    // } else
      //continue;  // 其他模式不处理
  }

  // 等待检测线程结束
  detect_thread.join();

  return 0;
}