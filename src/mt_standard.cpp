// 时间相关头文件：用于时间戳记录、延时、时间点计算（std::chrono）
#include <chrono>
// OpenCV核心头文件：图像存储、处理、命令行解析
#include <opencv2/opencv.hpp>
// 线程相关头文件：创建多线程、线程管理（std::thread、std::atomic）
#include <thread>

// 硬件IO模块：相机采集、串口通信、IMU数据读取
#include "io/camera.hpp"
#include "io/dm_imu/dm_imu.hpp"
// 自动瞄准任务模块：检测、解算、跟踪、瞄准、射击、多线程检测器/指令生成
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
// 自动打符任务模块：检测、解算、目标管理、瞄准、类型定义
#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"
#include "tasks/auto_buff/buff_type.hpp"
// 通用工具模块：程序退出管理、图像绘制、日志打印、数学工具、数据绘图、数据记录
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

// OpenCV命令行参数解析器 键值定义
// 格式：{参数名 | 默认值 | 参数说明}，@表示位置参数（无需指定参数名，按顺序传递）
const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"          // 帮助参数：-h/--help/--usage
  "{@config-path   | | yaml配置文件路径 }";          // 位置参数：第一个参数为配置文件路径

// 引入C++14时间字面量（如1ms、500us），简化时间延时/偏移计算
using namespace std::chrono_literals;

/**
 * @brief 自动瞄准系统主程序入口
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出码（0为正常退出，非0为异常）
 * @details 系统核心总控流程：
 *          1. 命令行参数解析，获取YAML配置文件路径；
 *          2. 初始化全局工具模块、硬件IO模块、自瞄/打符业务模块；
 *          3. 创建**装甲板检测独立线程**，实现图像采集与检测的并行化，提升实时性；
 *          4. 主循环（非阻塞）：监听系统工作模式，根据模式切换执行自瞄/打符/空闲逻辑；
 *          5. 模式切换时打印日志，保证运行状态可追溯；
 *          6. 自瞄模式：多线程检测→IMU姿态同步→坐标解算→目标跟踪→指令生成；
 *          7. 打符模式：图像采集→IMU姿态同步→符文检测→坐标解算→目标锁定→瞄准指令发送；
 *          8. 程序退出时等待检测线程结束，保证资源正常释放；
 * @note 核心设计：**检测线程与主循环解耦**，采用生产者-消费者模型，避免检测耗时阻塞主逻辑；
 * @note 时间同步：所有模块基于**同一时间戳t**工作，保证图像、IMU、目标数据的时间一致性；
 * @note 模式管理：通过std::atomic原子变量管理系统模式，保证多线程下模式切换的线程安全。
 */
int main(int argc, char * argv[])
{
  // 1. 初始化OpenCV命令行参数解析器，解析传入的参数
  cv::CommandLineParser cli(argc, argv, keys);
  // 获取位置参数：配置文件路径（第一个命令行参数）
  auto config_path = cli.get<std::string>("@config-path");
  // 若传入帮助参数，或未传入配置文件路径，打印参数说明并退出程序
  if (cli.has("help") || !cli.has("@config-path")) 
  {
    cli.printMessage();
    return 0;
  }

  // 2. 初始化通用工具模块（全局生效，所有业务模块可调用）
  tools::Exiter exiter;       // 程序退出管理器：统一管理退出标志，支持外部触发退出
  tools::Plotter plotter;     // 数据绘图工具：实时绘制目标轨迹、角度、距离等数据，便于调试
  tools::Recorder recorder;   // 数据记录工具：记录图像、IMU姿态、时间戳，支持离线复现与调试

  // 3. 初始化硬件IO模块（与硬件交互，配置文件加载硬件参数：端口、分辨率、波特率等）
  io::Camera camera(config_path);  // 相机模块：负责图像采集，返回图像+采集时间戳
  io::CBoard cboard(config_path);  // 串口板模块：负责IMU数据读取、云台/发射机构指令发送、系统模式获取

  // 4. 初始化自动瞄准（自瞄）业务模块（配置文件加载自瞄参数：检测阈值、跟踪参数、瞄准参数等）
  auto_aim::multithread::MultiThreadDetector detector(config_path);  // 多线程检测器：独立线程执行装甲板检测
  auto_aim::Solver solver(config_path);                              // 自瞄解算器：PnP解算+多坐标系转换
  auto_aim::Tracker tracker(config_path, solver);                    // 目标跟踪器：基于解算结果实现装甲板连续跟踪
  auto_aim::Aimer aimer(config_path);                                // 自瞄瞄准器：计算云台需要转动的角度、提前量
  auto_aim::Shooter shooter(config_path);                            // 发射控制器：计算发射延时、控制发射时机

  // 5. 初始化自动打符（打符）业务模块（配置文件加载打符参数：符文检测阈值、解算参数、瞄准参数等）
  auto_buff::Buff_Detector buff_detector(config_path);  // 符文检测器：检测大/小符文中的发光条目标
  auto_buff::Solver buff_solver(config_path);          // 打符解算器：专属符文的PnP解算+坐标转换
  auto_buff::SmallTarget buff_small_target;            // 小符文目标管理器：锁定小符文、跟踪目标状态
  auto_buff::BigTarget buff_big_target;                // 大符文目标管理器：锁定大符文、跟踪目标状态
  auto_buff::Aimer buff_aimer(config_path);            // 打符瞄准器：专属符文的瞄准角度、提前量计算

  // 6. 初始化自瞄多线程指令生成器（解耦瞄准/发射与串口发送，独立处理指令逻辑）
  auto_aim::multithread::CommandGener commandgener(shooter, aimer, cboard, plotter);

  // 7. 系统工作模式管理：原子变量保证多线程下的读写安全（检测线程与主循环均会访问）
  // 初始模式为空闲（idle），支持模式：idle/auto_aim/small_buff/big_buff
  std::atomic<io::Mode> mode{io::Mode::idle};
  // 上一次模式记录：用于检测模式切换，仅在模式变化时打印日志，避免重复输出
  auto last_mode{io::Mode::idle};

  // 8. 创建**装甲板检测独立线程**：生产者-消费者模型，与主循环并行执行
  // 作用：单独处理图像采集和装甲板检测（耗时操作），避免阻塞主循环的姿态同步、跟踪、指令生成逻辑
  auto detect_thread = std::thread([&]() 
  {
    cv::Mat img;  // 存储相机采集的图像
    std::chrono::steady_clock::time_point t;  // 存储图像采集的时间戳（高精度时钟，微秒级）

    // 检测线程主循环：直到程序触发退出标志才结束
    while (!exiter.exit()) 
    {
      // 仅当系统模式为自瞄（auto_aim）时，执行图像采集和检测
      if (mode.load() == io::Mode::auto_aim) 
      {
        camera.read(img, t);  // 相机采集图像：返回图像+采集时间戳t（时间同步核心）
        detector.push(img, t); // 将图像和时间戳推入多线程检测器的任务队列，异步执行检测
      } else
        continue;  // 非自瞄模式时，线程空转，减少资源占用
    }
  });

  // 9. 主程序循环：系统核心总控，处理模式切换、自瞄/打符业务逻辑，直到程序退出
  while (!exiter.exit()) 
  {
    // 从串口板获取当前系统工作模式（由遥控器/上位机通过串口设置），更新到原子变量
    mode = cboard.mode;

    // 检测模式是否发生变化：仅在切换时打印日志，记录运行状态
    if (last_mode != mode) 
    {
      tools::logger()->info("Switch to {}", io::MODES[mode]);  // 打印模式名称（如"auto_aim"、"small_buff"）
      last_mode = mode.load();  // 更新上一次模式记录
    }

    /// -------------------------- 自动瞄准模式（auto_aim）处理逻辑 --------------------------
    if (mode.load() == io::Mode::auto_aim) 
    {
      // 1. 从多线程检测器获取检测结果：调试版弹出（含原始图像+检测到的装甲板+采集时间戳t）
      // 生产者-消费者：detect_thread生产检测结果，主循环消费检测结果
      auto [img, armors, t] = detector.debug_pop();
      // 2. 获取指定时间戳的IMU四元数：t-1ms为时间偏移补偿（消除IMU数据读取/传输的微小延时）
      // 保证IMU姿态与图像采集时间严格同步，避免时间差导致的姿态解算误差
      Eigen::Quaterniond q = cboard.imu_at(t - 1ms);

      // 可选：记录数据（图像、IMU姿态、时间戳），用于离线调试和问题复现（注释默认关闭，按需启用）
      // recorder.record(img, q, t);

      // 3. 更新自瞄解算器的云台→世界旋转矩阵：将IMU四元数转换为云台绝对姿态，完成姿态同步
      solver.set_R_gimbal2world(q);

      // 4. 将云台旋转矩阵转换为欧拉角（yaw/pitch/roll，Z-Y-X顺序），用于后续瞄准计算
      Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

      // 5. 目标跟踪：基于检测到的装甲板和时间戳t，实现连续帧装甲板匹配与跟踪，输出跟踪目标
      // 跟踪器结合解算器结果，提升目标丢失后的找回能力和跟踪稳定性
      auto targets = tracker.track(armors, t);

      // 6. 将跟踪目标、时间戳、子弹速度、云台欧拉角推入指令生成器队列
      // 指令生成器异步处理：计算瞄准角度、发射提前量，生成云台/发射机构控制指令并发送到串口板
      commandgener.push(targets, t, cboard.bullet_speed, ypr);  // 解耦主逻辑与指令生成，提升实时性

    }

    /// -------------------------- 自动打符模式（small_buff/big_buff）处理逻辑 --------------------------
    else if (mode.load() == io::Mode::small_buff || mode.load() == io::Mode::big_buff) 
    {
      // 1. 声明变量：存储采集的图像、IMU四元数、图像采集时间戳
      cv::Mat img;
      Eigen::Quaterniond q;
      std::chrono::steady_clock::time_point t;

      // 2. 相机采集图像：返回图像+高精度时间戳t（打符模式为单线程采集，无需多线程）
      camera.read(img, t);
      // 3. 获取同步的IMU四元数：t-1ms时间偏移补偿，保证IMU姿态与图像时间一致
      q = cboard.imu_at(t - 1ms);

      // 可选：记录数据，用于离线调试（注释默认关闭，按需启用）
      // recorder.record(img, q, t);

      // 4. 更新打符解算器的云台→世界旋转矩阵，完成IMU姿态与解算器同步
      buff_solver.set_R_gimbal2world(q);

      // 5. 符文检测：输入采集的图像，检测其中的符文发光条目标，返回检测结果
      auto power_runes = buff_detector.detect(img);

      // 6. 打符解算：对检测到的符文执行PnP解算+多坐标系转换，获取符文的3D坐标和姿态
      buff_solver.solve(power_runes);

      // 7. 声明打符控制指令：存储云台转动角度、发射指令等
      io::Command buff_command;
      // 小符文模式：锁定小符文目标，计算瞄准指令
      if (mode.load() == io::Mode::small_buff) 
      {
        buff_small_target.get_target(power_runes, t);  // 小符文目标锁定：跟踪最新的符文检测结果
        auto target_copy = buff_small_target;          // 复制目标对象，避免多线程访问冲突
        // 计算打符瞄准指令：传入目标、时间戳、子弹速度，是否开启提前量补偿
        buff_command = buff_aimer.aim(target_copy, t, cboard.bullet_speed, true);
      }
      // 大符文模式：锁定大符文目标，计算瞄准指令
      else if (mode.load() == io::Mode::big_buff) 
      {
        buff_big_target.get_target(power_runes, t);    // 大符文目标锁定：跟踪最新的符文检测结果
        auto target_copy = buff_big_target;            // 复制目标对象，避免多线程访问冲突
        // 计算打符瞄准指令：与小符文共用瞄准器，内部根据符文类型适配参数
        buff_command = buff_aimer.aim(target_copy, t, cboard.bullet_speed, true);
      }
      // 8. 发送打符控制指令到串口板：由串口板将指令转发给云台和发射机构，执行瞄准和发射
      cboard.send(buff_command);

    } else
      continue;  // 空闲模式（idle）时，主循环空转，减少CPU占用
  }

  // 10. 程序退出时，等待检测线程执行完毕（线程汇合），保证线程资源正常释放，避免内存泄漏
  detect_thread.join();

  // 程序正常退出，返回0
  return 0;
}
