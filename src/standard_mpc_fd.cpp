// 引入第三方库：格式化输出、JSON解析、OpenCV图像处理
#include <fmt/core.h>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/detector.hpp" //传统识别

// 引入自定义模块头文件：IO（相机、控制板）、自瞄任务（检测、跟踪、瞄准、射击）、工具类（日志、退出、绘图）
#include "io/camera.hpp"       // 相机数据采集模块
#include "io/cboard.hpp"       // 控制板（CBoard）模块：读取IMU、发送控制指令、获取系统模式
#include "io/fd_cboard.hpp"    // FD控制板模块：基于CAN总线通信，获取IMU数据、云台状态，发送控制指令
#include "tasks/auto_aim/planner/planner.hpp"  // 任务规划模块：根据系统模式规划自瞄任务流程
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
  //tools::Recorder recorder;   // 数据录制器：录制图像、IMU姿态、时间戳，用于赛后复盘
  //auto_aim::Target target;    // 目标信息存储：存储当前目标的状态、位置等信息

  /* 3. 初始化IO模块（硬件交互） */
  io::FD_CBoard Fd_cboard(config_path);  // 控制板（CBoard）：初始化CAN总线，读取IMU、系统模式，发送控制指令
  io::Camera camera(config_path);  // 相机：初始化相机（如海康、大华），读取图像与采集时间戳

  /* 4. 初始化自瞄核心任务模块（基于配置文件参数） */
  //auto_aim::YOLO detector(config_path, true); //
  auto_aim::Detector detector(config_path, false);  // YOLO检测器：加载YOLO模型（如yolov5/yolov8），false=不启用调试模式
  auto_aim::Solver solver(config_path);         // 坐标解算器：初始化相机内参、外参，转换不同坐标系（相机→云台→世界）
  auto_aim::Tracker tracker(config_path, solver);  // 目标跟踪器：初始化跟踪参数（如最大丢失次数），依赖解算器做坐标转换
  //auto_aim::Aimer aimer(config_path);           // 瞄准器：初始化瞄准参数（如偏航/俯仰补偿角、预测时间）
  //auto_aim::Shooter shooter(config_path);       // 射击器：初始化射击参数（如射击容差、自动开火开关），预留射击控制
  auto_aim::Planner planner(config_path);         // 任务规划器：根据系统模式规划自瞄任务流程（如idle/auto_aim/buff）

  io::FD_Send_Command command{};  // 待发送的控制指令对象：初始化为默认值（false/0），后续由规划器更新

  // 初始化线程安全队列：实现「检测跟踪线程」向「云台规划线程」传递目标数据
  // 模板参数：存储类型（可选的自动瞄准目标）、是否覆盖旧数据（true表示队列满时覆盖头部）、队列容量（1）
  // 队列容量1：保证规划线程始终获取最新的目标检测结果，避免数据堆积
  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);  // 初始值：无目标，避免规划线程首次读取空队列

  // 原子布尔变量：控制规划线程退出（多线程安全，无需加锁）
  std::atomic<bool> quit = false;
  // 创建云台规划线程：后台执行云台运动规划与控制指令发送，与主线程（检测）并行
  // 采用Lambda表达式作为线程函数，&表示捕获外部所有变量的引用
  auto plan_thread = std::thread([&]() 
  {
    // 记录线程启动时间：用于后续计算运行时长（相对时间）
    auto t0 = std::chrono::steady_clock::now();
    // 记录上一帧云台弹数：用于判断当前帧是否开火成功（弹数增加即为开火）
    uint16_t last_bullet_count = 0;

    // 规划线程主循环：直到收到退出信号（quit=true）
    while (!quit) 
    {
      // 从线程安全队列获取最新目标数据（阻塞直到有数据，此处队列始终有初始值）
      auto target = target_queue.front();
      // 获取云台当前实时状态：位姿（yaw/pitch）、速度、弹数、子弹速度等
      auto gs = Fd_cboard.state();
      // 轨迹规划：根据目标状态、当前子弹速度，生成云台控制指令（位姿、速度、加速度、开火决策）
      auto plan = planner.plan(target, Fd_cboard.bullet_speed);  // 规划接口：输入目标状态（std::optional<Target>），输出控制指令（Plan结构体）

      command = Fd_cboard.plan_to_command(plan);
    
      // 向云台发送控制指令：执行规划结果，控制云台转动与开火
      Fd_cboard.send(command);  // 发送接口：将Plan结构体中的控制指令转换为FD_Send_Command结构体，封装为CAN帧发送给下位机

      // 判断是否开火成功：当前云台弹数 > 上一帧弹数，说明完成一次射击
    //   auto fired = gs.bullet_count > last_bullet_count;
    //   last_bullet_count = gs.bullet_count;  // 更新上一帧弹数，用于下一帧判断

      // 规划线程延时：10ms，控制规划频率（约100Hz），匹配云台控制频率
      std::this_thread::sleep_for(10ms);
    }
  });


  // 主线程（检测跟踪线程）变量定义：采集的图像、图像采集时间戳
  cv::Mat img;  // OpenCV图像矩阵，存储相机采集的原始图像
  std::chrono::steady_clock::time_point t;  // 高精度时间戳，与图像采集时刻同步

  // 主线程主循环：直到收到退出信号（exiter.exit()返回true，或按下q键）
  while (!exiter.exit()) 
  {
    // 相机采集图像：读取一帧图像到img，同时记录采集时间戳到t（时间同步关键）
    camera.read(img, t);
    // 获取云台在「图像采集时刻t」的位姿四元数：解决相机与云台的时间差问题（时间戳对齐）
    auto q = Fd_cboard.imu_at(t);

    // 解算器设置云台→世界坐标系的旋转四元数：更新坐标转换矩阵，为后续解算做准备
    solver.set_R_gimbal2world(q);
    // YOLO装甲板检测：输入原始图像，输出检测到的所有装甲板（包含位置、类型、置信度等）
    auto armors = detector.detect(img);
    // 目标跟踪：对检测到的装甲板进行多帧关联、EKF滤波预测，输出跟踪到的目标列表
    auto targets = tracker.track(armors, t);
    // 将跟踪结果推入线程安全队列：传递给规划线程
    // 有目标则推第一个目标（主目标），无目标则推空值（std::nullopt）
    if (!targets.empty())
      target_queue.push(targets.front());
    else
      target_queue.push(std::nullopt);

    // 若跟踪到有效目标，在图像上绘制重投影点（可视化调试）
    // if (!targets.empty()) 
    // {
    //   auto target = targets.front();  // 获取主跟踪目标

    //   // 绘制目标所有装甲板的重投影点：绿色
    //   // armor_xyza_list：目标装甲板在世界坐标系的位置+角度（x,y,z,angle）
    //   std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
    //   for (const Eigen::Vector4d & xyza : armor_xyza_list) 
    //   {
    //     // 装甲板重投影：世界坐标系→图像坐标系，得到图像上的像素点
    //     auto image_points =
    //       solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
    //     // 在图像上绘制重投影点：绿色（0,255,0），用于可视化装甲板定位结果
    //     tools::draw_points(img, image_points, {0, 255, 0});
    //   }

    //   // 绘制规划器的瞄准点重投影：红色
    //   // planner.debug_xyza：规划器计算的最优瞄准点（世界坐标系）
    //   Eigen::Vector4d aim_xyza = planner.debug_xyza;
    //   auto image_points =
    //     solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
    //   // 在图像上绘制瞄准点：红色（0,0,255），区分装甲板位置与瞄准点
    //   tools::draw_points(img, image_points, {0, 0, 255});
    // }

    // 图像缩放：显示时将图像缩小为原尺寸的50%，适配窗口显示，降低显示资源占用
    cv::resize(img, img, {}, 0.5, 0.5);
    // 显示图像窗口：窗口名「reprojection」，显示绘制后的图像（实时可视化）
    cv::imshow("reprojection", img);
    // 等待按键：1ms，非阻塞式，保证图像实时刷新；返回按下的键值
    auto key = cv::waitKey(1);
    // 若按下q键，退出主循环（手动退出程序）
    if (key == 'q') break;
  }

  // 程序退出处理：优雅释放资源，避免线程泄漏/云台异常
  quit = true;  // 设置规划线程退出标志，让规划线程主循环结束
  // 等待规划线程执行完毕：若线程可连接（未结束），则阻塞直到线程退出
  if (plan_thread.joinable()) plan_thread.join();
  // 向云台发送停止指令：关闭运动使能、关闭开火，所有控制参数置0，让云台归位/停止
  Fd_cboard.send(command);

  // 主程序正常退出
  return 0;
}