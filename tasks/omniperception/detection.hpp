#ifndef OMNIPERCEPTION__DETECTION_HPP
#define OMNIPERCEPTION__DETECTION_HPP

// 头文件保护：防止重复包含（唯一标识与文件名对应，避免编译冲突）
// 高精度时间库：用于记录检测结果的时间戳，保证多模块时间同步
#include <chrono>
// 标准链表：存储检测到的装甲板对象，支持高效的增删、遍历与后续过滤/排序操作
#include <list>

// 自动瞄准模块头文件：引入装甲板核心类（Armor），包含装甲板所有属性与枚举
#include "tasks/auto_aim/armor.hpp"

// 全向感知系统命名空间：隔离模块内数据结构，避免与其他模块命名冲突
namespace omniperception
{

/**
 * @brief 全向感知系统 单帧检测结果核心结构体
 * @details 该结构体是**检测层与决策层之间的核心数据传输载体**，封装单帧图像的完整检测结果，
 *          包含检测到的装甲板集合、检测时间戳、初始角度增量，为决策层提供全量原始数据，
 *          是多相机检测结果融合、目标优先级排序、最优目标选择的基础数据单元。
 * @note 设计特点：1. 单帧对应一个DetectionResult，保证数据的帧独立性；2. 包含时间戳实现多模块时间同步；
 *                3. 预存初始角度增量，减少决策层重复计算；4. 重载赋值运算符保证深拷贝有效性。
 */
struct DetectionResult
{
  /**
   * @brief 单帧检测到的装甲板集合
   * @details 存储当前帧图像中检测到的所有有效装甲板对象（auto_aim::Armor），采用std::list存储的优势：
   *          1. 决策层需频繁对装甲板执行过滤、删除操作，list支持O(1)节点删除，效率高于vector；
   *          2. 遍历效率满足工程需求，适配装甲板数量较少的实际场景（单帧检测到的装甲板数量有限）；
   *          3. 与决策层armor_filter、set_priority等接口的参数类型无缝对接，无需类型转换。
   */
  std::list<auto_aim::Armor> armors;

  /**
   * @brief 检测结果的高精度时间戳
   * @details 记录**检测完成时刻**的时间点（或图像采集时刻，按工程约定），采用std::chrono::steady_clock：
   *          1. 单调时钟，不受系统时间修改影响，保证时间戳的连续性与唯一性；
   *          2. 微秒级精度，满足多相机同步、IMU姿态匹配、目标跟踪的时间同步需求；
   *          3. 与系统主循环、决策层、跟踪层的时间戳类型一致，实现全链路时间同步。
   */
  std::chrono::steady_clock::time_point timestamp;

  /**
   * @brief 云台初始偏航角增量
   * @unit 弧度（rad）
   * @details 由检测层预先计算的**粗瞄准角度增量**（yaw轴，水平方向），是装甲板像素位置到云台角度的初步转换结果，
   *          决策层可基于该值做精细化调整，减少重复计算，提升决策实时性；若检测层未计算，可初始化为0.0。
   */
  double delta_yaw;    //rad

  /**
   * @brief 云台初始俯仰角增量
   * @unit 弧度（rad）
   * @details 由检测层预先计算的**粗瞄准角度增量**（pitch轴，垂直方向），与delta_yaw对应，
   *          共同表示云台从当前位置瞄准检测到的装甲板，需要转动的初始角度，单位统一为弧度，
   *          与云台控制模块的角度接口无缝对接（无需单位转换）。
   */
  double delta_pitch;  //rad

  /**
   * @brief 重载赋值运算符：实现DetectionResult的深拷贝
   * @param other 待赋值的源DetectionResult对象（const引用，避免拷贝开销）
   * @return DetectionResult& 赋值后的当前对象引用（支持链式赋值）
   * @details 核心作用：解决std::list<Armor>成员的**浅拷贝问题**，保证赋值后两个对象的armors相互独立，
   *          避免修改一个对象的armors导致另一个对象的数据被篡改。
   * @note 实现要点：1. 自赋值判断（this != &other），避免自身赋值导致的资源错误；
   *                2. 逐成员赋值，std::list的赋值运算符默认实现深拷贝，无需手动处理；
   *                3. 返回当前对象引用，符合C++赋值运算符的设计规范。
   */
  DetectionResult & operator=(const DetectionResult & other)
  {
    // 自赋值判断：若当前对象与源对象是同一个，直接返回自身，避免不必要的计算与资源操作
    if (this != &other) {
      armors = other.armors;    // 链表深拷贝：两个对象的armors指向不同的内存空间
      timestamp = other.timestamp;  // 时间戳赋值：基础类型直接拷贝
      delta_yaw = other.delta_yaw;  // 角度增量赋值：基础类型直接拷贝
      delta_pitch = other.delta_pitch;
    }
    // 返回当前对象引用，支持链式赋值（如a = b = c）
    return *this;
  }
};
}  // namespace omniperception

#endif // OMNIPERCEPTION__DETECTION_HPP
