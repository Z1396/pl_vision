// 头文件保护宏：防止多次包含导致的函数/模板重复定义编译错误
// 命名规则：模块名__文件名_HPP（大写，双层下划线分隔，区分命名空间与文件名）
#ifndef TOOLS__YAML_HPP
#define TOOLS__YAML_HPP

// 引入yaml-cpp核心头文件：开源YAML解析库，提供YAML文件加载、节点解析、类型转换等核心功能
// 该库是C++解析YAML配置文件的工业级选择，支持YAML 1.2标准，兼容主流配置场景
#include <yaml-cpp/yaml.h>

// 引入项目工具模块日志类：实现日志打印（ERROR级别），便于配置解析失败时的问题定位与调试
#include "tools/logger.hpp"

// 工具模块命名空间：隔离YAML解析相关工具函数，避免与项目其他模块（IO/算法）命名冲突
namespace tools
{
/**
 * @brief 加载YAML配置文件并返回根节点
 * @param path YAML配置文件的绝对/相对路径（如"config/can.yaml"）
 * @return YAML::Node YAML文件的根节点，作为所有配置项解析的入口
 * @note inline内联函数：直接在头文件实现，编译器自动做内联优化，消除函数调用开销
 * @note 异常处理：捕获yaml-cpp库的两类核心异常，保证配置加载失败时的友好退出
 * @note 工程特性：加载失败直接终止程序（exit(1)），因配置文件是程序运行的基础，无有效配置时无法正常工作
 */
inline YAML::Node load(const std::string & path)
{
  try {
    // 调用yaml-cpp库核心函数，从指定路径加载YAML文件，返回根节点
    return YAML::LoadFile(path);
  } catch (const YAML::BadFile & e) {
    // 捕获文件加载异常（如文件不存在、权限不足、路径错误），打印ERROR日志并终止程序
    logger()->error("[YAML] Failed to load file: {}", e.what());
    exit(1);
  } catch (const YAML::ParserException & e) {
    // 捕获YAML语法解析异常（如格式错误、缩进问题、键值对不合法），打印ERROR日志并终止程序
    logger()->error("[YAML] Parser error: {}", e.what());
    exit(1);
  }
}

/**
 * @brief 从YAML节点中读取指定键的配置值，模板实现支持任意yaml-cpp兼容类型
 * @tparam T 配置值的目标类型（如int/double/std::string/Eigen::Vector3d等，需yaml-cpp支持类型转换）
 * @param yaml 待解析的YAML节点（可为根节点或子节点，支持嵌套解析）
 * @param key 配置项的键名（如"can_interface"、"baud_rate"）
 * @return T 解析后的配置值，自动完成从YAML节点到目标类型的转换
 * @note 模板函数：泛型实现，一套代码支持所有兼容类型，避免重复编写同逻辑的重载函数
 * @note inline内联函数：头文件内实现，配合模板编译期实例化要求，同时消除调用开销
 * @note 非空校验：若指定键不存在，直接打印ERROR日志并终止程序，避免空节点访问导致的未定义行为
 * @note 类型安全：yaml-cpp内部会做类型校验，类型不匹配时抛出异常（如字符串转整数），保证类型安全
 */
template <typename T>
inline T read(const YAML::Node & yaml, const std::string & key)
{
  // 检查指定键是否存在于当前YAML节点中，存在则调用as<T>()完成类型转换并返回
  if (yaml[key]) return yaml[key].as<T>();
  // 键不存在时，打印ERROR日志（包含缺失的键名）并终止程序
  logger()->error("[YAML] {} not found!", key);
  exit(1);
}

}  // namespace tools

#endif  // TOOLS__YAML_HPP
