#include "logger.hpp"

#include <fmt/chrono.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <string>

namespace tools
{
std::shared_ptr<spdlog::logger> logger_ = nullptr;

// 初始化日志系统的配置函数
void set_logger()
{
  // 1. 生成日志文件名：以当前系统时间为名称，格式为 "年-月-日_时-分-秒.log"
  //    日志文件将存储在 "logs/" 目录下（需确保该目录已存在，否则可能创建失败）
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
