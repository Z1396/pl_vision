cmake_minimum_required(VERSION 3.16.3)

# C++代码用clang++
set(CMAKE_CXX_COMPILER "clang++")
# C代码用clang
set(CMAKE_C_COMPILER "clang")

# ========== 新增：Clang 专属 OpenMP 配置 ==========
# 为 Clang 指定 OpenMP 编译标志（解决 omp.h 找不到的核心）
#-fopenmp	基础参数：告诉编译器 “启用 OpenMP 支持”	和 GCC 一致，是 OpenMP 的核心启用开关
#=libomp	指定 OpenMP 运行时库为 libomp	Clang 的 OpenMP 依赖 libomp 库（GCC 依赖 libgomp），如果不指定：
#1. Ubuntu 下的 Clang 会默认找 libgomp（GCC 的库），导致链接报错；
#. 明确指定 libomp 才能让 Clang 正确链接 OpenMP 运行时。
set(OpenMP_CXX_FLAGS "-fopenmp=libomp -Wno-unused-command-line-argument")
set(OpenMP_C_FLAGS "-fopenmp=libomp -Wno-unused-command-line-argument")
# =================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

project(sp_vision)

message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

# ========== 新增：查找并启用 OpenMP ==========
find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
    # 为所有编译目标添加 OpenMP 标志
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    # 为链接阶段添加 OpenMP 库
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()
# =============================================



# 基础格式（最常用）
#find_package(库名 [版本要求] [REQUIRED] [COMPONENTS 组件列表])
#部分	说明	示例（你的项目）
#库名	要找的库名称（大小写不敏感，惯例大写）	OpenCV、Eigen3、fmt
#版本要求	可选，指定最低版本（避免版本兼容问题）	find_package(OpenCV 4.0 REQUIRED)
#REQUIRED	可选，标记为 “必需库”：找不到就报错终止配置	你的项目里 OpenCV 是必需的，必须加 REQUIRED
#COMPONENTS 组件列表	可选，指定要找的库组件（部分库分多个组件）	find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
#一、先搞懂：为什么有的库要 link，有的不用？
#C++ 库分两种类型，这直接决定了是否需要 target_link_libraries：
#库类型	特点	是否需要 target_link_libraries	例子
#头文件库（header-only）	所有代码都写在头文件（.h/.hpp）中，编译时直接嵌入到你的程序里，无单独的 .so/.a 库文件	❌ 不需要	nlohmann/json、Eigen3
#编译型库（binary/library）	代码分为头文件（.h）和编译后的二进制文件（.so/.a），编译时只需头文件，但链接时必须关联二进制文件	✅ 必须要	fmt、OpenCV、spdlog、yaml-cpp
find_package(nlohmann_json REQUIRED)
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/")
find_package(OpenVINO REQUIRED)


#OpenCV	库名（和 find_package(OpenCV) 中的库名一致）	库名作为前缀，大小写和 find_package 中的库名对应（OpenCV 是官方库名，所以前缀是 OpenCV）
#INCLUDE	变量类型：头文件路径	固定关键词，所有库的头文件路径变量都包含 INCLUDE
#DIRS	复数形式：表示 “多个路径”	DIRS = Directories（目录），因为 OpenCV 的头文件分布在多个子目录（如 opencv2/core、opencv2/imgproc），变量存储的是 “路径列表”
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
#1. include_directories()：CMake 核心命令
#作用是：告诉编译器 “去哪里找头文件”，添加的路径会被编译器纳入「头文件搜索范围」—— 编译器遇到 #include "xxx.hpp" 时，会优先在这些路径里找。
#2. ${PROJECT_SOURCE_DIR}：CMake 内置变量
#PROJECT_SOURCE_DIR 是 CMake 自动生成的变量，含义是：你执行 project(sp_vision) 后，项目根目录的绝对路径。比如你的项目路径是 /home/pldx/Desktop/sp_vision_25-main，那么 ${PROJECT_SOURCE_DIR} 就等于这个路径。
#3. 整行代码的作用
#把你的项目根目录加入「编译器的头文件搜索路径」，这样你的代码里 #include "tools/xxx.hpp"、#include "io/serial.hpp" 等语句，编译器能直接找到对应的头文件。

#方式 1：不加 include_directories(${PROJECT_SOURCE_DIR})
#需要写相对路径（从 standard.cpp 到 utils.hpp）：
#// src/standard.cpp
#include "../tools/utils.hpp"  // 必须写 ../ 回退到上一级目录，繁琐且易出错
include_directories(${PROJECT_SOURCE_DIR})


add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)
add_subdirectory(tasks/auto_buff)
add_subdirectory(tasks/omniperception)


####################src###################
#add_executable	CMake 核心命令：创建可执行文件目标	告诉 CMake “我要生成一个可执行程序”
#standard	可执行文件的名称（目标名）	编译完成后，build 目录下会生成名为 standard 的可执行文件（Linux 下无后缀，Windows 下是 standard.exe）
#src/standard.cpp	构建可执行文件的源码文件（相对路径）	指向项目根目录下 src 文件夹里的 standard.cpp—— 这是程序的入口文件（包含 main() 函数）
add_executable(standard src/standard.cpp)
add_executable(mt_standard src/mt_standard.cpp)
add_executable(standard_mpc src/standard_mpc.cpp)
add_executable(auto_aim_debug_mpc src/auto_aim_debug_mpc.cpp)
add_executable(mt_auto_aim_debug src/mt_auto_aim_debug.cpp)
add_executable(auto_buff_debug src/auto_buff_debug.cpp)
add_executable(auto_buff_debug_mpc src/auto_buff_debug_mpc.cpp)
add_executable(uav src/uav.cpp)
add_executable(uav_debug src/uav_debug.cpp)


#这行命令的核心作用（链接流程）
#target_link_libraries() 是编译流程的最后一步，解决的是 “编译通过但链接失败” 的核心问题：
    #编译阶段：add_executable() 把 standard.cpp 编译成 standard.o（目标文件），编译器只检查头文件声明（比如 cv::imread() 存在），不关心实现；
    #链接阶段：target_link_libraries() 把 standard.o 和所有依赖库的二进制实现（如 libopencv_core.so 里的 cv::imread() 实现）合并，生成完整的可执行文件。

#四、关键细节（避坑核心）
#1. 链接顺序：“先依赖后被依赖”（Linux 特有）
#Linux 下链接器是 “从右往左” 解析依赖的，必须遵循 “被依赖的库放在后面”，否则会报链接错误。比如：
    #auto_aim 库依赖 tools 库（用了 tools 里的日志函数）；
    #正确顺序：target_link_libraries(standard auto_aim tools)（tools 在 auto_aim 后面）；
    #错误顺序：target_link_libraries(standard tools auto_aim) → 可能报 undefined reference to tools::logger::info()。

#Module 模式是 “CMake 替你找库”，Config 模式是 “库自己告诉你在哪” —— 前者标准化
target_link_libraries(standard ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io ) # 大华 SDK 库添加到这里)
target_link_libraries(mt_standard ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(standard_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(auto_aim_debug_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(mt_auto_aim_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(auto_buff_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io ${CERES_LIBRARIES})
target_link_libraries(auto_buff_debug_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io ${CERES_LIBRARIES})
target_link_libraries(uav ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(uav_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

##################calibration################
#add_executable(biaodin calibration/biaodin.cpp)
add_executable(capture calibration/capture.cpp)
add_executable(calibrate_camera calibration/calibrate_camera.cpp)
add_executable(calibrate_handeye calibration/calibrate_handeye.cpp)
add_executable(calibrate_robotworld_handeye calibration/calibrate_robotworld_handeye.cpp)
add_executable(split_video calibration/split_video.cpp)

#target_link_libraries(biaodin ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(capture ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(calibrate_camera ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_robotworld_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(split_video ${OpenCV_LIBS} fmt::fmt tools)

##################tests##################
add_executable(auto_aim_test tests/auto_aim_test.cpp)
add_executable(auto_buff_test tests/auto_buff_test.cpp)
add_executable(camera_detect_test tests/camera_detect_test.cpp)
add_executable(camera_test tests/camera_test.cpp)
add_executable(camera_thread_test tests/camera_thread_test.cpp)
add_executable(cboard_test tests/cboard_test.cpp)
add_executable(fire_test tests/fire_test.cpp)
add_executable(detector_video_test tests/detector_video_test.cpp)
add_executable(gimbal_response_test tests/gimbal_response_test.cpp)
add_executable(multi_usbcamera_test tests/multi_usbcamera_test.cpp)
add_executable(usbcamera_detect_test tests/usbcamera_detect_test.cpp)
add_executable(usbcamera_test tests/usbcamera_test.cpp)
add_executable(handeye_test tests/handeye_test.cpp)
add_executable(dm_test tests/dm_test.cpp)
add_executable(minimum_vision_system tests/minimum_vision_system.cpp)

target_link_libraries(auto_aim_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(auto_buff_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io)
target_link_libraries(camera_detect_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim io)
target_link_libraries(camera_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(camera_thread_test ${OpenCV_LIBS} fmt::fmt auto_aim tools io)
target_link_libraries(cboard_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(fire_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(detector_video_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim)
target_link_libraries(gimbal_response_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(multi_usbcamera_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(usbcamera_detect_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(usbcamera_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(handeye_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(dm_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(minimum_vision_system ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)

add_executable(gimbal_test tests/gimbal_test.cpp)
target_link_libraries(gimbal_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

add_executable(planner_test tests/planner_test.cpp)
target_link_libraries(planner_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

add_executable(planner_test_offline tests/planner_test_offline.cpp)
target_link_libraries(planner_test_offline ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

# 检查 ROS 环境
#QUIET：核心参数，含义是 **“静默查找”** ——CMake 找库时：
#✅ 找到库：正常生成变量（如 rclcpp_FOUND），但不输出 “找到库” 的日志；
#❌ 找不到库：不报错、不终止配置，仅设置 XXX_FOUND 为 FALSE，全程无日志提示。
#对比你之前用的 REQUIRED：REQUIRED 是 “强制查找”，找不到直接报错终止；QUIET 是 “可选查找”，找不到也继续执行。
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)
# 只有在没有找到 ROS 环境时才会添加 sentry 和 publish_test
if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND AND rosidl_typesupport_cpp_FOUND)
    #编译 sentry 和 publish_test
    add_executable(sentry src/sentry.cpp)
    add_executable(sentry_bp src/sentry_bp.cpp)
    add_executable(sentry_debug src/sentry_debug.cpp)
    add_executable(sentry_multithread src/sentry_multithread.cpp)
    add_executable(publish_test tests/publish_test.cpp)
    add_executable(subscribe_test tests/subscribe_test.cpp)
    add_executable(topic_loop_test tests/topic_loop_test.cpp)

    target_link_libraries(sentry ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(sentry_bp ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(sentry_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(sentry_multithread ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(publish_test ${OpenCV_LIBS} fmt::fmt tools io auto_aim)
    target_link_libraries(subscribe_test ${OpenCV_LIBS} fmt::fmt tools io)
    target_link_libraries(topic_loop_test ${OpenCV_LIBS} fmt::fmt tools io)
else()
    message(STATUS "ROS2 environment not found, skipping ROS2-related code.")
endif()
