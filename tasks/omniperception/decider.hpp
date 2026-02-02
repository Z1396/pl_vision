#ifndef OMNIPERCEPTION__DECIDER_HPP
#define OMNIPERCEPTION__DECIDER_HPP

// 头文件保护：防止重复包含（OMNIPERCEPTION__DECIDER_HPP为唯一标识，与文件名对应）

// Eigen线性代数库：用于矩阵/向量计算（如云台位置、角度增量），必须在OpenCV Eigen适配头文件前引入
#include <Eigen/Dense>
// 标准输入输出：日志打印、调试信息输出
#include <iostream>
// 标准链表：存储检测到的装甲板对象（支持高效的增删/遍历）
#include <list>
// 无序哈希表：实现装甲板名称到优先级的快速映射（O(1)查找）
#include <unordered_map>

// 本地头文件：检测结果定义、硬件IO模块、自动瞄准业务模块
#include "detection.hpp"               // 检测结果通用结构体（DetectionResult）
#include "io/camera.hpp"               // 相机基类：通用相机接口
#include "io/command.hpp"              // 控制指令结构体：云台/发射机构指令定义
#include "io/usbcamera/usbcamera.hpp"  // USB相机子类：专用于USB相机的实现
#include "tasks/auto_aim/armor.hpp"    // 装甲板类：定义Armor、ArmorName、ArmorPriority、Color等
#include "tasks/auto_aim/target.hpp"   // 目标类：自动瞄准目标管理
#include "tasks/auto_aim/yolo.hpp"     // YOLO检测器：装甲板检测核心类

// 全向感知系统命名空间：隔离模块内代码，避免与其他模块命名冲突
namespace omniperception
{

/**
 * @brief 全向感知系统核心决策器类
 * @details 决策器是全向感知与自动瞄准的**核心调度与决策模块**，承接多相机检测结果，
 *          完成**装甲板过滤、目标优先级排序、最优目标选择、瞄准角度计算**等核心逻辑，
 *          最终生成云台控制指令（io::Command）传递给执行层。
 *          核心能力：1. 多相机输入适配（双USB相机+后置相机/仅后置相机）；2. 装甲板有效性过滤；
 *                    3. 可配置的目标优先级策略（双模式切换）；4. 无敌装甲板屏蔽；5. 指定目标锁定；
 *                    6. 瞄准角度增量计算；7. 多检测结果排序与最优选择。
 * @note 设计特点：1. 多重载decide接口，适配不同相机输入场景；2. 优先级策略解耦为静态映射，便于配置；
 *                3. 成员变量存储配置参数，避免重复传参；4. 接口面向检测结果/装甲板，与检测层解耦。
 */
class Decider
{
public:
  /**
   * @brief 构造函数：初始化决策器，加载配置文件参数
   * @param config_path YAML配置文件路径
   * @details 从配置文件读取核心参数：图像分辨率、相机视场角（FOV）、工作模式、敌方颜色、
   *          优先级模式等，初始化成员变量与优先级映射表，是决策器工作的基础。
   */
  Decider(const std::string & config_path);

  /**
   * @brief 决策核心接口（重载1：三相机输入）
   * @param yolo YOLO检测器对象：用于装甲板检测（若需二次检测/验证）
   * @param gimbal_pos 云台当前位置：Eigen三维向量（x,y,z），用于坐标系参考/角度计算
   * @param usbcam1 前向USB相机1：全向感知的水平相机1，提供检测图像
   * @param usbcam2 前向USB相机2：全向感知的水平相机2，提供检测图像
   * @param back_cammera 后置相机：全向感知的后置相机，补充后方视野
   * @return io::Command 云台控制指令：包含瞄准角度增量、发射指令等，传递给执行层
   * @details 适配**全向三相机**感知场景，整合三个相机的检测结果，执行过滤、排序、选目标后，
   *          计算云台需要转动的角度增量，生成最终控制指令，是全向感知模式的核心决策接口。
   * @note 参数拼写：back_cammera为笔误（正确为back_camera），工程中保持与调用端一致即可。
   */
  io::Command decide(
    auto_aim::YOLO & yolo, const Eigen::Vector3d & gimbal_pos, io::USBCamera & usbcam1,
    io::USBCamera & usbcam2, io::Camera & back_cammera);

  /**
   * @brief 决策核心接口（重载2：单后置相机输入）
   * @param yolo YOLO检测器对象：用于装甲板检测/验证
   * @param gimbal_pos 云台当前位置：Eigen三维向量，坐标系参考
   * @param back_cammera 后置相机：唯一输入相机，提供检测图像
   * @return io::Command 云台控制指令：瞄准角度增量+发射指令
   * @details 适配**单相机**简化场景，仅处理后置相机的检测结果，流程与三相机版一致，
   *          适用于全向感知关闭、仅后置相机工作的场景，降低系统资源占用。
   */
  io::Command decide(
    auto_aim::YOLO & yolo, const Eigen::Vector3d & gimbal_pos, io::Camera & back_cammera);

  /**
   * @brief 决策核心接口（重载3：直接接收检测结果队列）
   * @param detection_queue 检测结果队列：多个相机/帧的检测结果集合（std::vector<DetectionResult>）
   * @return io::Command 云台控制指令：瞄准角度增量+发射指令
   * @details 最通用的决策接口，**与相机硬件解耦**，直接接收上层传递的检测结果队列，
   *          无需关心检测来源（单相机/多相机），仅执行决策逻辑，便于单元测试/模块解耦。
   * @note 推荐使用：该接口解耦了决策器与相机/检测器，符合“依赖倒置原则”，便于扩展。
   */
  io::Command decide(const std::vector<DetectionResult> & detection_queue);

  /**
   * @brief 计算云台瞄准角度增量
   * @param armors 过滤/排序后的装甲板链表：待选择的有效装甲板集合
   * @param camera 相机名称/标识：字符串（如"usbcam1"、"back_camera"），用于获取相机参数（FOV/分辨率）
   * @return Eigen::Vector2d 角度增量：二维向量（yaw偏航角, pitch俯仰角），单位为弧度/角度
   * @details 决策器核心计算接口，根据选中的最优装甲板位置、相机视场角、图像分辨率，
   *          结合云台当前位置，计算云台需要**转动的yaw/pitch角度增量**，是生成控制指令的关键步骤。
   */
  Eigen::Vector2d delta_angle(
    const std::list<auto_aim::Armor> & armors, const std::string & camera);

  /**
   * @brief 装甲板有效性过滤
   * @param armors 输入输出参数：待过滤的装甲板链表（引用传递，原地修改）
   * @return bool 过滤结果：true表示过滤成功（有有效装甲板），false表示无有效装甲板
   * @details 对检测到的装甲板执行**有效性筛选**，剔除无效目标：
   *          1. 非敌方颜色装甲板；2. 面积过小/过大的装甲板（离焦/误检）；
   *          3. 位置超出相机视场的装甲板；4. 无敌状态的装甲板（结合invincible_armor_）；
   *          过滤后仅保留有效装甲板，减少后续决策的计算量与错误率。
   */
  bool armor_filter(std::list<auto_aim::Armor> & armors);

  /**
   * @brief 为装甲板设置优先级（核心决策逻辑）
   * @param armors 输入输出参数：有效装甲板链表（引用传递，原地修改）
   * @details 根据当前工作模式（mode_），从对应的优先级映射表（mode1/mode2）中，
   *          为每个装甲板（按ArmorName）分配对应的ArmorPriority优先级，
   *          是后续“最优目标选择”的基础，优先级可通过配置文件切换模式。
   */
  void set_priority(std::list<auto_aim::Armor> & armors);

  /**
   * @brief 检测结果队列排序与过滤
   * @param detection_queue 输入输出参数：原始检测结果队列（引用传递，原地修改）
   * @details 对多相机/多帧的检测结果队列执行**全流程预处理**：
   *          1. 遍历每个DetectionResult，提取装甲板并执行armor_filter过滤；
   *          2. 为过滤后的装甲板执行set_priority设置优先级；
   *          3. 按**优先级从高到低**对检测结果队列排序，优先级相同则按距离/面积排序；
   *          处理后队列头部即为最优检测结果，便于快速选择目标。
   */
  void sort(std::vector<DetectionResult> & detection_queue);

  /**
   * @brief 获取目标核心信息
   * @param armors 有效装甲板链表：已过滤/设优先级的装甲板集合
   * @param targets 已跟踪的目标链表：自动瞄准的跟踪目标（结合跟踪器结果）
   * @return Eigen::Vector4d 目标四元信息：包含[距离, 角度, 装甲板类型, 优先级]（具体维度按业务定义）
   * @details 整合装甲板检测结果与跟踪器目标结果，提取目标的**关键特征信息**，
   *          为角度计算、发射提前量计算提供数据支撑，是检测与跟踪融合的接口。
   */
  Eigen::Vector4d get_target_info(
    const std::list<auto_aim::Armor> & armors, const std::list<auto_aim::Target> & targets);

  /**
   * @brief 更新无敌状态装甲板（从外部接收无敌机器人ID）
   * @param invincible_enemy_ids 无敌机器人ID列表：int8_t类型（英雄1/哨兵6/其他编号）
   * @details 接收来自上层/裁判系统的**无敌机器人ID**，将其转换为对应的ArmorName，
   *          存入invincible_armor_成员变量，后续armor_filter会自动屏蔽这些无敌装甲板，
   *          避免瞄准无敌目标造成无效射击。
   * @note 装甲板编号映射：英雄机器人→ArmorName::one，哨兵机器人→ArmorName::sentry（6），
   *       其他机器人按编号对应ArmorName::two~five。
   */
  void get_invincible_armor(const std::vector<int8_t> & invincible_enemy_ids);

  /**
   * @brief 锁定指定自动瞄准目标（从外部接收指定目标ID）
   * @param armors 输入输出参数：有效装甲板链表（引用传递，原地修改）
   * @param auto_aim_target 指定瞄准目标ID列表：int8_t类型，需锁定的机器人ID
   * @details 支持**人工指定瞄准目标**（如遥控器/上位机指定打击英雄/哨兵），
   *          遍历装甲板链表，仅保留与指定目标ID匹配的装甲板，剔除其他所有目标，
   *          实现“定点打击”，适用于战术性瞄准需求。
   */
  void get_auto_aim_target(
    std::list<auto_aim::Armor> & armors, const std::vector<int8_t> & auto_aim_target);

private:
  // 相机内参/配置参数：从配置文件加载，用于角度计算、视场角判断
  int img_width_;    // 相机图像宽度（像素）：如640/1280
  int img_height_;   // 相机图像高度（像素）：如480/720
  double fov_h_;     // 相机原始水平视场角（FOV，单位：度/弧度）
  double new_fov_h_; // 相机校正后水平视场角：适配图像裁剪/缩放后的实际视场
  double fov_v_;     // 相机原始垂直视场角
  double new_fov_v_; // 相机校正后垂直视场角

  int mode_;         // 优先级工作模式：1→MODE_ONE，2→MODE_TWO（对应PriorityMode枚举）
  int count_;        // 计数变量：用于帧计数/状态切换计数（如连续N帧无目标则复位）

  auto_aim::Color enemy_color_; // 敌方装甲板颜色：红/蓝（从配置文件加载，过滤非敌方目标）
  auto_aim::YOLO detector_;     // 内置YOLO检测器：用于装甲板二次检测/验证，避免误检

  std::vector<auto_aim::ArmorName> invincible_armor_; // 无敌状态装甲板名称列表：屏蔽这些目标

  // 类型别名：简化装甲板名称→优先级的映射类型定义，提升代码可读性
  using PriorityMap = std::unordered_map<auto_aim::ArmorName, auto_aim::ArmorPriority>;

  /**
   * @brief 优先级映射表：模式1（MODE_ONE）
   * @details 战术模式1的目标优先级策略：**优先打击三号/四号机器人**（first），
   *          次选一号（second），再次五号/哨兵（third），最后二号/前哨/基地（forth/fifth），
   *          适用于常规作战，优先打击高威胁/易打击目标。
   * @note ArmorPriority枚举：first（最高）→second→third→forth→fifth（最低）
   */
  const PriorityMap mode1 = {
    {auto_aim::ArmorName::one, auto_aim::ArmorPriority::second},    // 1号（英雄）→二级
    {auto_aim::ArmorName::two, auto_aim::ArmorPriority::forth},     // 2号→四级
    {auto_aim::ArmorName::three, auto_aim::ArmorPriority::first},   // 3号→一级（最高）
    {auto_aim::ArmorName::four, auto_aim::ArmorPriority::first},    // 4号→一级（最高）
    {auto_aim::ArmorName::five, auto_aim::ArmorPriority::third},    // 5号→三级
    {auto_aim::ArmorName::sentry, auto_aim::ArmorPriority::third},  // 哨兵（6号）→三级
    {auto_aim::ArmorName::outpost, auto_aim::ArmorPriority::fifth}, // 前哨→五级（最低）
    {auto_aim::ArmorName::base, auto_aim::ArmorPriority::fifth},    // 基地→五级
    {auto_aim::ArmorName::not_armor, auto_aim::ArmorPriority::fifth}// 非装甲板→五级
  };

  /**
   * @brief 优先级映射表：模式2（MODE_TWO）
   * @details 战术模式2的目标优先级策略：**优先打击二号机器人**（first），
   *          其余作战机器人（1/3/4/5）均为二级，哨兵/前哨/基地为三级，
   *          适用于特定战术场景（如二号机器人为高价值目标/敌方主力）。
   * @note 模式切换：通过配置文件修改mode_，或外部指令动态切换，无需修改代码。
   */
  const PriorityMap mode2 = {
    {auto_aim::ArmorName::two, auto_aim::ArmorPriority::first},     // 2号→一级（最高）
    {auto_aim::ArmorName::one, auto_aim::ArmorPriority::second},    // 1号→二级
    {auto_aim::ArmorName::three, auto_aim::ArmorPriority::second},  // 3号→二级
    {auto_aim::ArmorName::four, auto_aim::ArmorPriority::second},   // 4号→二级
    {auto_aim::ArmorName::five, auto_aim::ArmorPriority::second},   // 5号→二级
    {auto_aim::ArmorName::sentry, auto_aim::ArmorPriority::third},  // 哨兵→三级
    {auto_aim::ArmorName::outpost, auto_aim::ArmorPriority::third}, // 前哨→三级
    {auto_aim::ArmorName::base, auto_aim::ArmorPriority::third},    // 基地→三级
    {auto_aim::ArmorName::not_armor, auto_aim::ArmorPriority::third}// 非装甲板→三级
  };
};

/**
 * @brief 优先级工作模式枚举
 * @details 定义决策器的两种战术优先级模式，与mode1/mode2映射表一一对应，
 *          替代魔法数字，提升代码可读性与可维护性，支持后续扩展更多模式。
 */
enum PriorityMode
{
  MODE_ONE = 1,  // 模式1：常规战术（优先3/4号）
  MODE_TWO = 2   // 模式2：特殊战术（优先2号）
};

}  // namespace omniperception

#endif // OMNIPERCEPTION__DECIDER_HPP
