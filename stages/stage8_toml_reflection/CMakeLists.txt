cmake_minimum_required(VERSION 3.25)
project(stage8_toml_reflection LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(demo src/demo.cpp)
target_include_directories(demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(demo PRIVATE -Wall -Wextra -Werror)
