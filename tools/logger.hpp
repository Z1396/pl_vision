#ifndef TOOLS__LOGGER_HPP
#define TOOLS__LOGGER_HPP

/*#include <spdlog/spdlog.h> 是 C++ 中高性能日志库 spdlog 的核心头文件，
核心作用是让你的自瞄程序能够输出「结构化、多级别、高性能」的日志（比如调试信息、错误提示、运行状态），
替代简单的 std::cout/std::cerr，是工业级 C++ 项目（尤其是实时性要求高的自瞄程序）中日志输出的首选方案。*/
#include <spdlog/spdlog.h>

namespace tools
{
/*设计逻辑（为什么这么封装？）
对比直接使用 spdlog 全局日志器（spdlog::info()），这种封装方式有三个核心优势，完全适配自瞄项目的需求：
对比维度	直接用 spdlog 全局日志器	封装为 tools::logger()
初始化控制	全局日志器的初始化逻辑散落在主程序，易遗漏	初始化逻辑封装在 logger() 函数内，首次调用时自动初始化（懒加载），保证只初始化一次
模块解耦	所有模块直接依赖 spdlog，修改日志库时需改所有代码	只有 tools 模块依赖 spdlog，其他模块只依赖 tools::logger()，更换日志库（比如改用 glog）时只需改 tools 模块
多日志器支持	全局只有一个日志器，无法区分模块日志（比如相机日志 / 云台日志）	可扩展为返回不同日志器（比如 tools::logger("camera")），实现模块级日志隔离
生命周期管理	全局日志器的销毁时机不可控	用 shared_ptr 管理，最后一个引用销毁时自动释放，资源安全*/
std::shared_ptr<spdlog::logger> logger();

}  // namespace tools

#endif  // TOOLS__LOGGER_HPP