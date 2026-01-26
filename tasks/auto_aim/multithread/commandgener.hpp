#ifndef AUTO_AIM_MULTITHREAD__HPP
#define AUTO_AIM_MULTITHREAD__HPP

#include <optional>  // 用于表示可能为空的值（可选值）

// 硬件控制板接口头文件
#include "io/cboard.hpp"
// 自瞄模块相关头文件：发射器、追踪器
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/tracker.hpp"
// 全向感知决策器头文件
#include "tasks/omniperception/decider.hpp"
// 数据绘图工具头文件（用于调试可视化）
#include "tools/plotter.hpp"

namespace auto_aim
{
namespace multithread
{

/**
 * 命令生成器类（多线程版本）
 * 功能：在独立线程中处理目标信息，生成并发送控制命令，避免主逻辑阻塞
 */
class CommandGener
{
public:
  /**
   * 构造函数
   * @param shooter 发射器对象引用（控制发射逻辑）
   * @param aimer 瞄准器对象引用（计算瞄准角度）
   * @param cboard 控制板对象引用（发送命令到下位机）
   * @param plotter 绘图工具引用（调试数据可视化）
   * @param debug 是否启用调试模式
   */
  CommandGener(
    auto_aim::Shooter & shooter, auto_aim::Aimer & aimer, io::CBoard & cboard,
    tools::Plotter & plotter, bool debug = false);

  /**
   * 析构函数
   * 功能：停止线程并释放资源
   */
  ~CommandGener();

  /**
   * 推送目标数据到命令生成器
   * @param targets 追踪到的目标列表
   * @param t 目标数据对应的时间戳
   * @param bullet_speed 当前子弹速度
   * @param gimbal_pos 云台当前姿态（欧拉角）
   */
  void push(
    const std::list<auto_aim::Target> & targets, const std::chrono::steady_clock::time_point & t,
    double bullet_speed, const Eigen::Vector3d & gimbal_pos);

private:
  /**
   * 输入数据结构体
   * 存储生成命令所需的所有信息
   */
  struct Input
  {
    std::list<auto_aim::Target> targets_;  // 目标列表
    std::chrono::steady_clock::time_point t;  // 时间戳
    double bullet_speed;  // 子弹速度
    Eigen::Vector3d gimbal_pos;  // 云台姿态
  };

  io::CBoard & cboard_;  // 控制板引用（发送命令）
  auto_aim::Shooter & shooter_;  // 发射器引用（判断是否发射）
  auto_aim::Aimer & aimer_;  // 瞄准器引用（计算瞄准角度）
  tools::Plotter & plotter_;  // 绘图工具引用（调试）

  std::optional<Input> latest_;  // 最新的输入数据（可选值，可能为空）
  std::mutex mtx_;  // 互斥锁（保护共享数据latest_）
  std::condition_variable cv_;  // 条件变量（通知线程处理新数据）
  std::thread thread_;  // 命令生成线程
  bool stop_;  // 线程停止标志
  bool debug_;  // 调试模式标志

  /**
   * 命令生成线程主函数
   * 功能：循环等待新数据，计算控制命令并发送
   */
  void generate_command();
};

}  // namespace multithread

}  // namespace auto_aim

#endif  // AUTO_AIM_MULTITHREAD__HPP