// 引入第三方标准库：格式化输出、原子操作、时间处理、JSON解析、OpenCV视觉库、多线程
#include <fmt/core.h>               // 现代C++格式化输出库，替代printf/stringstream，更安全高效
#include <atomic>                   // 原子变量库，保证多线程间变量操作的原子性（无数据竞争）
#include <chrono>                   // 高精度时间处理库，用于计时、延时、时间戳同步
#include <nlohmann/json.hpp>        // 轻量级JSON解析库，用于数据序列化（日志/绘图数据传输）
#include <opencv2/opencv.hpp>       // OpenCV核心视觉库，处理图像采集、绘制、显示、命令行解析
#include <thread>                   // C++11多线程库，实现云台规划线程的创建与管理

// 项目自定义模块头文件：按功能划分，模块化设计
#include "io/camera.hpp"            // 相机IO模块：相机初始化、图像采集、时间戳同步
#include "io/gimbal/gimbal.hpp"     // 云台IO模块：云台状态读取、控制指令发送、位姿解算
#include "tasks/auto_aim/planner/planner.hpp"  // 自动瞄准规划器：目标轨迹预测、云台运动规划、开火决策
#include "tasks/auto_aim/solver.hpp"           // 解算器：坐标转换（云台→世界→图像）、装甲板重投影
#include "tasks/auto_aim/tracker.hpp"          // 目标跟踪器：对检测到的装甲板进行多帧关联、EKF滤波预测
#include "tasks/auto_aim/yolo.hpp"             // YOLO检测器：基于YOLO模型实现装甲板实时检测
#include "tools/exiter.hpp"          // 工具类：程序退出控制（优雅处理退出信号）
#include "tools/img_tools.hpp"       // 工具类：图像处理工具（绘制点/线、图像缩放等）
#include "tools/logger.hpp"          // 工具类：日志记录（INFO/WARN/ERROR，未在主程序显式使用）
#include "tools/math_tools.hpp"      // 工具类：数学工具（矩阵运算、时间差计算等）
#include "tools/plotter.hpp"         // 工具类：实时绘图工具（绘制云台状态、目标参数、规划结果）
#include "tools/thread_safe_queue.hpp"  // 工具类：线程安全队列，实现检测线程与规划线程的通信

// 引入C++14时间字面量（如10ms），简化延时代码书写
using namespace std::chrono_literals;

// OpenCV命令行参数定义：支持帮助信息、配置文件路径指定
// 格式：{参数名 | 默认值 | 说明}，@表示位置参数（无需指定参数名，直接传值）
const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }";

// 主程序入口：argc参数个数，argv参数数组
int main(int argc, char * argv[])
{
  // 初始化工具类：程序退出控制器、实时绘图器
  tools::Exiter exiter;       // 管理程序退出状态，替代直接exit()，保证资源优雅释放
  tools::Plotter plotter;     // 实时绘制运行数据（如云台位姿、目标速度、开火状态）

  // 解析命令行参数：基于OpenCV的CommandLineParser，简化参数处理
  cv::CommandLineParser cli(argc, argv, keys);
  // 获取位置参数：配置文件路径（索引0对应第一个位置参数）
  auto config_path = cli.get<std::string>(0);
  // 若指定帮助参数（-h/--help）或配置文件路径为空，输出帮助信息并退出
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  // 初始化核心硬件模块：云台、相机（均从yaml配置文件读取参数：设备号、波特率、内参等）
  io::Gimbal gimbal(config_path);    // 云台初始化：建立与云台的通信、读取初始状态
  io::Camera camera(config_path);    // 相机初始化：打开相机、设置分辨率/帧率、读取相机内参

  // 初始化自动瞄准核心算法模块（均从yaml读取参数：模型路径、滤波参数、规划参数等）
  auto_aim::YOLO yolo(config_path, true);        // YOLO检测器：true表示启用实时检测（或加载模型）
  auto_aim::Solver solver(config_path);          // 解算器：加载相机内参、云台外参、坐标转换矩阵
  auto_aim::Tracker tracker(config_path, solver);// 目标跟踪器：关联解算器，用于坐标转换；加载EKF滤波参数
  auto_aim::Planner planner(config_path);        // 规划器：加载运动规划参数、开火阈值、预测时长

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
      auto gs = gimbal.state();
      // 轨迹规划：根据目标状态、当前子弹速度，生成云台控制指令（位姿、速度、加速度、开火决策）
      auto plan = planner.plan(target, gs.bullet_speed);

      // 向云台发送控制指令：执行规划结果，控制云台转动与开火
      gimbal.send(
        plan.control,  // 云台使能标志（是否允许云台运动）
        plan.fire,     // 开火标志（是否触发射击）
        plan.yaw, plan.yaw_vel, plan.yaw_acc,  // 云台横滚：目标角度、速度、加速度
        plan.pitch, plan.pitch_vel, plan.pitch_acc);  // 云台俯仰：目标角度、速度、加速度

      // 判断是否开火成功：当前云台弹数 > 上一帧弹数，说明完成一次射击
      auto fired = gs.bullet_count > last_bullet_count;
      last_bullet_count = gs.bullet_count;  // 更新上一帧弹数，用于下一帧判断

      // 构造JSON数据：存储当前帧运行状态，用于实时绘图/日志记录
      nlohmann::json data;
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);  // 程序运行相对时间（s）

      // 云台当前状态：横滚/俯仰角度、速度
      data["gimbal_yaw"] = gs.yaw;
      data["gimbal_yaw_vel"] = gs.yaw_vel;
      data["gimbal_pitch"] = gs.pitch;
      data["gimbal_pitch_vel"] = gs.pitch_vel;

      // 规划的目标位姿：云台需要到达的横滚/俯仰角度（目标位置对应的云台角度）
      data["target_yaw"] = plan.target_yaw;
      data["target_pitch"] = plan.target_pitch;

      // 云台规划结果：目标角度、速度、加速度（运动控制参数）
      data["plan_yaw"] = plan.yaw;
      data["plan_yaw_vel"] = plan.yaw_vel;
      data["plan_yaw_acc"] = plan.yaw_acc;
      data["plan_pitch"] = plan.pitch;
      data["plan_pitch_vel"] = plan.pitch_vel;
      data["plan_pitch_acc"] = plan.pitch_acc;

      // 开火状态：规划的开火指令、实际开火结果（弹数变化判断）
      data["fire"] = plan.fire ? 1 : 0;
      data["fired"] = fired ? 1 : 0;

      // 若存在有效目标，记录目标的Z轴位置、速度（EKF滤波后的值，索引4/5对应Z轴位置/速度）
      if (target.has_value()) {
        data["target_z"] = target->ekf_x()[4];   // 目标相对云台的Z轴距离（深度）
        data["target_vz"] = target->ekf_x()[5];  // 目标Z轴方向的运动速度
      }

      // 记录目标装甲板宽度（EKF滤波后，索引7），无目标则为0
      if (target.has_value()) {
        data["w"] = target->ekf_x()[7];
      } else {
        data["w"] = 0.0;
      }

      // 实时绘图：将JSON数据传入plotter，绘制曲线/数值显示
      plotter.plot(data);

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
    auto q = gimbal.q(t);

    // 解算器设置云台→世界坐标系的旋转四元数：更新坐标转换矩阵，为后续解算做准备
    solver.set_R_gimbal2world(q);
    // YOLO装甲板检测：输入原始图像，输出检测到的所有装甲板（包含位置、类型、置信度等）
    auto armors = yolo.detect(img);
    // 目标跟踪：对检测到的装甲板进行多帧关联、EKF滤波预测，输出跟踪到的目标列表
    auto targets = tracker.track(armors, t);
    // 将跟踪结果推入线程安全队列：传递给规划线程
    // 有目标则推第一个目标（主目标），无目标则推空值（std::nullopt）
    if (!targets.empty())
      target_queue.push(targets.front());
    else
      target_queue.push(std::nullopt);

    // 若跟踪到有效目标，在图像上绘制重投影点（可视化调试）
    if (!targets.empty()) 
    {
      auto target = targets.front();  // 获取主跟踪目标

      // 绘制目标所有装甲板的重投影点：绿色
      // armor_xyza_list：目标装甲板在世界坐标系的位置+角度（x,y,z,angle）
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) 
      {
        // 装甲板重投影：世界坐标系→图像坐标系，得到图像上的像素点
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        // 在图像上绘制重投影点：绿色（0,255,0），用于可视化装甲板定位结果
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      // 绘制规划器的瞄准点重投影：红色
      // planner.debug_xyza：规划器计算的最优瞄准点（世界坐标系）
      Eigen::Vector4d aim_xyza = planner.debug_xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      // 在图像上绘制瞄准点：红色（0,0,255），区分装甲板位置与瞄准点
      tools::draw_points(img, image_points, {0, 0, 255});
    }

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
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  // 主程序正常退出
  return 0;
}
