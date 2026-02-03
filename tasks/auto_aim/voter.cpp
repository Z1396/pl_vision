// 头文件保护宏：防止因头文件重复包含导致的类/函数多重定义错误
// 命名规则：PROJECT_MODULE__FILE_HPP，符合C++工程化规范，避免宏名冲突
#ifndef AUTO_AIM__VOTER_HPP
#define AUTO_AIM__VOTER_HPP

// 引入C++标准向量容器：用于底层存储投票计数结果，动态连续内存，访问高效
#include <vector>

// 引入装甲板核心定义头文件：包含Color(颜色)、ArmorName(装甲板名称)、ArmorType(装甲板类型)
// 这三个枚举是投票器的核心入参，与装甲板检测结果强关联
#include "armor.hpp"

// 自动瞄准模块根命名空间：隔离项目自定义代码，避免全局命名空间污染
namespace auto_aim
{

/**
 * @brief 装甲板检测结果投票器类
 * @details 基于「投票计数」机制实现装甲板检测结果的**多帧一致性验证/结果筛选**，
 *          接收连续帧的装甲板检测结果（颜色/名称/类型）进行投票累加，
 *          可通过count接口获取指定类型装甲板的累计投票数，用于过滤单次检测的误检结果，
 *          提升自动瞄准系统对装甲板识别的鲁棒性（抗干扰、降低误判）
 * @note 核心依赖armor.hpp中的三个枚举：Color(红/蓝)、ArmorName(装甲板编号/名称)、ArmorType(大/小/普通等)
 */
class Voter
{
public:
  /**
   * @brief 构造函数
   * @details 初始化底层投票计数容器count_，根据Color、ArmorName、ArmorType的枚举取值范围，
   *          初始化固定大小的计数数组，所有投票项初始计数值为0
   */
  Voter();

  /**
   * @brief 核心投票接口：对指定属性的装甲板进行一次投票（计数值+1）
   * @param color 装甲板颜色：Color枚举（如RED/BLUE，对应敌方/我方装甲板）
   * @param name 装甲板名称：ArmorName枚举（如装甲板编号/车型对应装甲板名称，如HERO_1、ENGINE_2）
   * @param type 装甲板类型：ArmorType枚举（如BIG/SMALL/NORMAL，对应不同尺寸装甲板）
   * @note 入参为const值传递，避免不必要的拷贝，且保证入参不被修改；
   *       内部通过index()函数计算唯一索引，实现对count_容器的快速定位与计数累加
   */
  void vote(const Color color, const ArmorName name, const ArmorType type);

  /**
   * @brief 投票计数查询接口：获取指定属性装甲板的累计投票数
   * @param color 装甲板颜色：同vote接口
   * @param name 装甲板名称：同vote接口
   * @param type 装甲板类型：同vote接口
   * @return std::size_t 累计投票数：非负整数，数值越大表示该装甲板被检测到的次数越多
   * @note 用于后续结果筛选：通常取投票数最高的装甲板作为最终有效检测结果，过滤低投票数的误检项
   */
  std::size_t count(const Color color, const ArmorName name, const ArmorType type);

private:
  /**
   * @brief 底层投票计数容器：存储所有装甲板属性组合的累计投票数
   * @details 1. 采用std::vector<std::size_t>，连续内存布局，随机访问效率O(1)，适配高频投票/查询；
   *          2. 元素类型为std::size_t（无符号整型），符合计数非负的业务特性，避免负数溢出；
   *          3. 容器大小固定：由Color、ArmorName、ArmorType的枚举取值总数决定，无动态扩容开销；
   *          4. 私有成员：禁止外部直接修改，仅通过vote/count接口操作，保证计数数据的安全性
   */
  std::vector<std::size_t> count_;

  /**
   * @brief 私有索引计算函数：将（颜色+名称+类型）三元组转换为count_容器的唯一整型索引
   * @param color 装甲板颜色：同vote接口
   * @param name 装甲板名称：同vote接口
   * @param type 装甲板类型：同vote接口
   * @return std::size_t 唯一索引值：范围[0, count_.size()-1]，无越界
   * @note 1. const成员函数：保证函数执行时不修改类的任何成员变量，符合只读操作的语义；
   *       2. 核心映射逻辑：将三维属性组合映射为一维整型索引，实现对count_的快速定位，
   *          是投票器高效运行的关键，常见映射公式：index = color*N*M + name*M + type（N/M为对应枚举取值数）；
   *       3. 私有封装：索引计算逻辑仅内部使用，对外隐藏实现细节，便于后续枚举扩展时修改映射规则
   */
  std::size_t index(const Color color, const ArmorName name, const ArmorType type) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__VOTER_HPP：头文件保护宏结束，与开头#ifndef配对
