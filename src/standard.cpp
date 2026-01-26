// 引入第三方库：格式化输出、JSON解析、OpenCV图像处理
#include <fmt/core.h>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/detector.hpp" //传统识别

// 引入自定义模块头文件：IO（相机、控制板）、自瞄任务（检测、跟踪、瞄准、射击）、工具类（日志、退出、绘图）
#include "io/camera.hpp"       // 相机数据采集模块
#include "io/cboard.hpp"       // 控制板（CBoard）模块：读取IMU、发送控制指令、获取系统模式
#include "tasks/auto_aim/aimer.hpp"          // 瞄准器模块：计算云台控制指令
#include "tasks/auto_aim/multithread/commandgener.hpp"  // 指令生成辅助模块（未直接使用，预留扩展）
#include "tasks/auto_aim/shooter.hpp"        // 射击器模块（未直接使用，预留射击控制）
#include "tasks/auto_aim/solver.hpp"         // 坐标解算模块：转换相机/云台/世界坐标系
#include "tasks/auto_aim/tracker.hpp"        // 目标跟踪模块：稳定跟踪多装甲板目标
#include "tasks/auto_aim/yolo.hpp"           // YOLO目标检测模块：检测图像中的装甲板
#include "tools/exiter.hpp"                  // 退出控制模块：响应Ctrl+C信号
#include "tools/img_tools.hpp"               // 图像处理工具：图像预处理、绘制等
#include "tools/logger.hpp"                  // 日志模块：输出运行信息（INFO/WARN/ERROR）
#include "tools/math_tools.hpp"              // 数学工具：矩阵运算、欧拉角/四元数转换等
#include "tools/plotter.hpp"                 // 绘图模块：实时绘制目标框、跟踪轨迹等
#include "tools/recorder.hpp"                // 录制模块：录制图像、姿态数据用于后续分析

//#include "../dahua/dhua.h"  // 大华相机SDK头文件（注释：当前使用默认相机模块，预留扩展）

// 使用chrono命名空间：处理时间戳（如图像采集时间、IMU数据时间）
using namespace std::chrono;

//DHUA dahua;  // 大华相机实例（注释：当前未启用，预留扩展）

// 命令行参数定义：使用OpenCV的CommandLineParser解析参数
// 格式：{参数名 | 默认值 | 说明}，@表示位置参数（无需参数名，直接传值）
const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"  // -help/-h：显示帮助信息
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }";  // 第一个位置参数：配置文件路径

// 主函数：程序入口
int main(int argc, char * argv[])
{
  // 1. 解析命令行参数
  cv::CommandLineParser cli(argc, argv, keys);  // 初始化参数解析器
  auto config_path = cli.get<std::string>(0);   // 获取位置参数0：配置文件路径
  if (cli.has("help") || config_path.empty())   // 若用户输入-help，或配置文件路径为空
  {
    cli.printMessage();  // 打印参数说明
    return 0;            // 退出程序
  }

  /* 2. 初始化核心工具类 */
  tools::Exiter exiter;       // 退出控制器：监听Ctrl+C信号，设置exit()标志
  tools::Plotter plotter;     // 绘图器：实时在图像上绘制目标框、跟踪ID、瞄准线等
  tools::Recorder recorder;   // 数据录制器：录制图像、IMU姿态、时间戳，用于赛后复盘
  //auto_aim::Target target;    // 目标信息存储：存储当前目标的状态、位置等信息

  /* 3. 初始化IO模块（硬件交互） */
  io::CBoard cboard(config_path);  // 控制板（CBoard）：初始化CAN总线，读取IMU、系统模式，发送控制指令
  io::Camera camera(config_path);  // 相机：初始化相机（如海康、大华），读取图像与采集时间戳

  /* 4. 初始化自瞄核心任务模块（基于配置文件参数） */
  //auto_aim::YOLO detector(config_path, true); //
  auto_aim::Detector detector(config_path, false);  // YOLO检测器：加载YOLO模型（如yolov5/yolov8），false=不启用调试模式
  auto_aim::Solver solver(config_path);         // 坐标解算器：初始化相机内参、外参，转换不同坐标系（相机→云台→世界）
  auto_aim::Tracker tracker(config_path, solver);  // 目标跟踪器：初始化跟踪参数（如最大丢失次数），依赖解算器做坐标转换
  auto_aim::Aimer aimer(config_path);           // 瞄准器：初始化瞄准参数（如偏航/俯仰补偿角、预测时间）
  auto_aim::Shooter shooter(config_path);       // 射击器：初始化射击参数（如射击容差、自动开火开关），预留射击控制

  /* 5. 定义变量：存储实时数据 */
  cv::Mat img;                                  // 存储相机采集的图像
  Eigen::Quaterniond q;                         // 存储IMU的四元数（表示云台姿态）
  std::chrono::steady_clock::time_point t;      // 存储图像采集的时间戳

  io::Mode mode = io::Mode::idle;               // 当前系统模式（如idle=空闲，auto_aim=自瞄，buff=能量机关）
  auto last_mode = io::Mode::idle;              // 上一帧系统模式：用于检测模式切换，打印日志

  auto t0 = std::chrono::steady_clock::now();

  // ====================== 新增：FPS计算相关变量 ======================
  auto last_frame_time = std::chrono::steady_clock::now();  // 上一帧的时间戳（初始化为主程序启动时间）
  double fps = 0.0;                                         // 存储当前FPS值
  const int FPS_SMOOTH_WINDOW = 5;                          // FPS平滑窗口（取最近5帧的平均值，减少波动）
  std::vector<double> frame_intervals;                      // 存储最近N帧的间隔时间，用于平滑FPS
  // ==================================================================

  /* 6. 主循环：持续运行直到收到退出信号（Ctrl+C） */
  while (!exiter.exit()) 
  {
    // ====================== 新增：计算FPS（放在循环开头，确保覆盖完整循环逻辑） ======================
    auto current_frame_time = std::chrono::steady_clock::now();  // 当前帧时间戳
    // 计算当前帧与上一帧的时间间隔（单位：秒）
    double frame_interval = tools::delta_time(current_frame_time, last_frame_time);
    // 更新上一帧时间戳（为下一帧做准备）
    last_frame_time = current_frame_time;

    // 平滑FPS计算（避免单帧波动导致FPS跳变）
    frame_intervals.push_back(frame_interval);
    if (frame_intervals.size() > FPS_SMOOTH_WINDOW) {
      frame_intervals.erase(frame_intervals.begin());  // 移除最早的帧间隔
    }
    // 计算平均帧间隔，再求倒数得到平滑后的FPS
    double avg_interval = 0.0;
    for (double interval : frame_intervals) {
      avg_interval += interval;
    }
    avg_interval /= frame_intervals.size();
    if (avg_interval > 1e-6) {  // 防止除以0（极端情况下帧间隔为0）
      fps = 1.0 / avg_interval;
    } else {
      fps = 0.0;
    }
    // ==================================================================

    // 6.1 采集相机图像与时间戳：camera.read()会阻塞直到获取新图像
    camera.read(img, t);  

    // 6.2 读取IMU姿态：根据图像采集时间t，获取1ms前的IMU四元数（补偿相机与IMU的时间同步误差）
    q = cboard.imu_at(t - 1ms);  
    // cv::imshow("camera", img);  // 显示采集的图像（调试用）
    //if (cv::waitKey(1) == 27) break;  // 按Esc键退出
    //Eigen::Vector3d eulers = tools::eulers(q, 2, 1, 0) * 57.3;
    
    //6.3 读取当前系统模式（由遥控器或上位机通过CBoard发送）
    mode = cboard.mode;

    //6.4 检测系统模式切换：若模式变化，打印日志
    if (last_mode != mode) 
    {
      tools::logger()->info("Switch to {}", io::MODES[mode]);  // 输出模式切换信息（如"Switch to auto_aim"）
      last_mode = mode;  // 更新上一帧模式
    }

    //（注释：数据录制功能，按需启用）
    recorder.record(img, q, t);  // 录制当前帧图像、IMU姿态、时间戳到文件

    // 6.5 坐标解算：设置云台到世界坐标系的旋转矩阵（由IMU四元数转换）
    solver.set_R_gimbal2world(q);  

    // （注释：调试用：将旋转矩阵转换为欧拉角（yaw偏航、pitch俯仰、roll滚转），按需启用）
    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);  // 2,1,0表示yaw-pitch-roll顺序

    tools::logger()->info("z{:.2f} y{:.2f} x{:.2f} degree | FPS: {:.1f}", 
                    ypr[0] * 57.3, ypr[1] * 57.3, ypr[2] * 57.3, 
                    fps);  // 新增：打印FPS（保留1位小数）

    // 6.6 目标检测：用YOLO检测器检测图像中的装甲板，返回装甲板列表（含位置、尺寸、置信度等）
     auto armors = detector.detect(img);  

    // // 6.7 目标跟踪：对检测到的装甲板进行多目标跟踪，输出稳定的目标列表（过滤瞬时误检、匹配同一目标）
    auto targets = tracker.track(armors, t);  
    
    // // 6.8 瞄准计算：根据跟踪到的目标、当前时间戳、子弹速度，计算云台控制指令（偏航角、俯仰角、是否射击）
    auto command = aimer.aim(targets, t, cboard.bullet_speed);  

    command.shoot = shooter.shoot(command, aimer, targets, ypr);

    //6.9 发送控制指令：将瞄准器计算的指令通过CBoard发送给云台执行机构（如电机）
     cboard.send(command);  

    tools::logger()->info("Control: {}, Shoot: {}, Yaw: {:.2f}, Pitch: {:.2f} | FPS: {:.1f}",
                    command.control, 
                    command.shoot, 
                    command.yaw * 57.3, 
                    command.pitch * 57.3,
                    fps);  // 新增：打印控制指令时附带FPS

    /// debug
    tools::draw_text(img, fmt::format("[{}] | FPS: {:.1f}", tracker.state(), fps), {100, 30}, {255, 255, 255});  // 新增：在图像上显示FPS
   

    nlohmann::json data;
    data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
    data["fps"] = fps;  // 新增：将FPS存入data，方便后续分析/录制

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
      data["armor_x"] = armor.xyz_in_world[0]*100;
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_z"] = armor.xyz_in_world[2] * 100;
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
      data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
    }

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

      // 观测器内部数据
      Eigen::VectorXd x = target.ekf_x();
      data["x"] = x[0];
      data["vx"] = x[1] * 10;
      data["y"] = x[2];
      data["vy"] = x[3];
      data["z"] = x[4] * 10;
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
    if (command.control) 
    {
      data["cmd_yaw"] = command.yaw * 57.3;
      data["cmd_pitch"] = command.pitch * 57.3;
      data["cmd_shoot"] = command.shoot;
    }
    plotter.plot(data);

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  
  }

  // 7. 程序正常退出
  return 0;
}