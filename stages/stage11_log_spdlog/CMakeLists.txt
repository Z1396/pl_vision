cmake_minimum_required(VERSION 3.25)
project(stage11_log_spdlog LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(spdlog REQUIRED)

add_executable(demo
  src/demo.cpp
  src/compile_time_trim.cpp
)
target_include_directories(demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(demo PRIVATE spdlog::spdlog)
target_compile_options(demo PRIVATE -Wall -Wextra -Werror)
