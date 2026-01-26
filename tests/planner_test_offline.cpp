#include <chrono>         // 时间相关功能（用于计时、睡眠）
#include <nlohmann/json.hpp>  // JSON数据序列化/反序列化库（用于日志/绘图数据封装）
#include <opencv2/opencv.hpp> // OpenCV库（主要用于命令行参数解析）
#include <thread>         // 线程相关功能（用于控制循环频率）

// 自定义头文件
#include "tasks/auto_aim/planner/planner.hpp"  // 自动瞄准规划器类头文件
#include "tools/exiter.hpp"                   // 程序退出控制工具（处理退出信号/条件）
#include "tools/logger.hpp"                   // 日志工具（此处未显式使用，但可能被其他模块依赖）
#include "tools/math_tools.hpp"               // 数学工具库（包含delta_time等辅助函数）
#include "tools/plotter.hpp"                  // 绘图工具（实时绘制规划结果曲线）

// 简化chrono库时间字面量使用（如10ms表示10毫秒）
using namespace std::chrono_literals;

/**
 * @brief 命令行参数定义
 * @details 使用OpenCV的CommandLineParser解析参数，格式说明：
 * - {参数名 | 默认值 | 说明}
 * - help/h/usage/?：帮助参数，无默认值，用于输出参数说明
 * - d：目标距离参数，默认3.0米
 * - w：目标角速度参数，默认5.0弧度/秒
 * - @config-path：必选位置参数，默认路径为configs/standard3.yaml，指定MPC配置文件路径
 */
const std::string keys =
  "{help h usage ? |     | 输出命令行参数说明    }"
  "{d              | 3.0 | Target距离(m)       }"
  "{w              | 5.0 | Target角速度(rad/s) }"
  "{@config-path   |   /home/z/Desktop/sp_vision_25-main/configs/standard3.yaml  | yaml配置文件路径     }";

/**
 * @brief 主函数（自动瞄准规划器测试程序入口）
 * @details 功能：创建模拟目标，循环调用规划器生成控制指令，实时绘制规划结果
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return 程序退出状态码（0表示正常退出）
 */
int main(int argc, char * argv[])
{
  // 1. 初始化命令行参数解析器
  cv::CommandLineParser cli(argc, argv, keys);
  // 获取配置文件路径（位置参数@config-path）
  auto config_path = cli.get<std::string>("@config-path");
  // 获取目标距离（参数d）
  auto d = cli.get<double>("d");
  // 获取目标角速度（参数w）
  auto w = cli.get<double>("w");

  // 2. 处理帮助请求或缺少必选参数的情况
  if (cli.has("help") || !cli.has("@config-path")) 
  {
    cli.printMessage();  // 输出参数说明信息
    return 0;            // 退出程序
  }

  // 3. 初始化工具类
  tools::Exiter exiter;       // 退出控制器（监听退出信号，控制循环退出）
  tools::Plotter plotter;     // 绘图工具（用于实时绘制规划数据曲线）

  // 4. 初始化核心组件
  auto_aim::Planner planner(config_path);  // 创建自动瞄准规划器（传入配置文件路径初始化MPC求解器）
  // 创建模拟目标对象：参数分别为 距离(m)、角速度(rad/s)、宽度(m)、高度(m)（具体参数含义取决于Target类实现）
  auto_aim::Target target(d, w, 0.2, 0.1);

  // 5. 记录程序启动时间（用于计算运行时长）
  auto t0 = std::chrono::steady_clock::now();

  // 6. 主循环（持续运行直到收到退出信号）
  while (!exiter.exit()) 
  {
    // 模拟目标运动：预测0.01秒（与规划器控制周期DT一致）后的目标状态（位置、速度等）
    target.predict(0.01);

    // 调用规划器生成控制指令：传入当前目标状态和子弹速度（固定22m/s）
    auto plan = planner.plan(target, 22);

    // 7. 封装绘图数据（使用JSON格式组织数据）
    nlohmann::json data;
    // 记录当前运行时间（从程序启动到现在的时长，由math_tools的delta_time计算）
    data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);

    // 目标角度指令（规划器输出的期望最终角度）
    data["target_yaw"] = plan.target_yaw;    // 目标偏航角
    data["target_pitch"] = plan.target_pitch;// 目标俯仰角

    // 偏航通道规划结果（当前时刻的状态）
    data["plan_yaw"] = plan.yaw;             // 当前偏航角
    data["plan_yaw_vel"] = plan.yaw_vel;     // 当前偏航角速度
    data["plan_yaw_acc"] = plan.yaw_acc;     // 当前偏航角加速度

    // 俯仰通道规划结果（当前时刻的状态）
    data["plan_pitch"] = plan.pitch;         // 当前俯仰角
    data["plan_pitch_vel"] = plan.pitch_vel; // 当前俯仰角速度
    data["plan_pitch_acc"] = plan.pitch_acc; // 当前俯仰角加速度

    // 8. 实时绘图：将当前周期的规划数据传入绘图工具，更新曲线
    plotter.plot(data);

    // 9. 控制循环频率：睡眠10毫秒（与控制周期DT=0.01s一致，确保循环频率约100Hz）
    std::this_thread::sleep_for(10ms);
  }

  // 程序正常退出
  return 0;
}