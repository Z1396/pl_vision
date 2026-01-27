#include "logger.hpp"

#include <fmt/chrono.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <string>

namespace tools
{
/*1. 全局日志器指针：std::shared_ptr<spdlog::logger> logger_ = nullptr;

  作用：全局保存日志器实例，保证整个程序只有一个日志器（单例模式）；
  为什么用 shared_ptr：spdlog 的 logger 本身支持多线程共享，shared_ptr 能安全管理其生命周期，避免重复初始化 / 释放；
  初始化为 nullptr：配合 logger() 函数的懒加载逻辑，首次调用时才初始化。*/
std::shared_ptr<spdlog::logger> logger_ = nullptr;

// 初始化日志系统的配置函数
void set_logger()
{
  /*2. 日志文件名生成：fmt::format("logs/{:%Y-%m-%d_%H-%M-%S}.log", std::chrono::system_clock::now())

    核心依赖：<fmt/chrono.h> 让 fmt 库能直接格式化 std::chrono::system_clock::now()（系统时间戳）；
    格式说明：
        %Y-%m-%d：年 - 月 - 日（如 2026-01-28）；
        %H-%M-%S：时 - 分 - 秒（如 17-30-00）；
        最终文件名示例：logs/2026-01-28_17-30-00.log；
    关键隐患：如果 logs/ 目录不存在，文件 sink 创建会失败（程序不会崩溃，但日志无法写入文件）。*/
  auto file_name = fmt::format("logs/{:%Y-%m-%d_%H-%M-%S}.log", std::chrono::system_clock::now());

  // 2. 创建文件输出器（sink）：用于将日志写入文件
  //    basic_file_sink_mt 表示多线程安全版本（mt = multi-thread）
  //    第二个参数 true 表示追加模式（若文件已存在则追加内容，false 则覆盖）
  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(file_name, true);
  // 设置文件输出器的日志级别为 debug：只有 >= debug 级别的日志才会写入文件
  // （spdlog 级别从低到高：trace < debug < info < warn < error < critical）
  file_sink->set_level(spdlog::level::debug);

  // 3. 创建控制台输出器：用于将日志打印到终端（带颜色输出）
  //    stdout_color_sink_mt 是多线程安全的彩色控制台输出器
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  // 设置控制台输出器的日志级别为 debug：只有 >= debug 级别的日志会显示在控制台
  console_sink->set_level(spdlog::level::debug);

  // 4. 创建日志器（logger）并关联输出器
  //    将文件输出器和控制台输出器添加到日志器，实现"同时输出到文件和控制台"
  //    第一个参数为日志器名称（此处为空字符串，使用默认名称）
  logger_ = std::make_shared<spdlog::logger>("", spdlog::sinks_init_list{file_sink, console_sink});
  
  // 5. 设置日志器的全局级别为 debug：限制所有输出器的最低日志级别（比输出器级别更严格）
  //    若日志器级别设为 info，则即使输出器设为 debug，debug 级别的日志也不会输出
  logger_->set_level(spdlog::level::debug);
  
  // 6. 设置自动刷新策略：当日志级别 >= info 时，立即刷新缓冲区到输出目标（文件/控制台）
  //    避免因程序崩溃导致 info 及以上级别的重要日志未写入文件
  logger_->flush_on(spdlog::level::info);
}

std::shared_ptr<spdlog::logger> logger()
{
  if (!logger_) set_logger();
  return logger_;
}

}  // namespace tools
