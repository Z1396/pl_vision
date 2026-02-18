cmake_minimum_required(VERSION 3.16)

find_package(yaml-cpp REQUIRED)

add_subdirectory(serial)

# 创建目标 io
#add_library 是 CMake 中构建模块化项目的核心命令，它和你熟悉的 add_executable 是「姐妹命令」—— add_executable 生成可执行文件（比如 standard），
#而 add_library 生成库文件（静态库 / 动态库），这也是你的 sp_vision 项目能拆分成 tools/io/auto_aim 等模块的关键
add_library(io STATIC 
    dahua/dahua.cpp
    dahua/streamRetrieve.cpp
    hikrobot/hikrobot.cpp    
    #mindvision/mindvision.cpp  
    usbcamera/usbcamera.cpp  
    camera.cpp
    cboard.cpp
    dm_imu/dm_imu.cpp
    gimbal/gimbal.cpp
    fd_cboard.cpp
)

# hikrobot
#include_directories：是全局生效的指令，为当前 CMakeLists.txt 及所有子目录中所有后续的目标（target） 添加头文件搜索路径。
#target_include_directories：是目标级的指令，只为指定的单个目标（可执行文件 / 库） 添加头文件搜索路径，支持更精细的作用域控制。
target_include_directories(io PUBLIC hikrobot/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC hikrobot/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(io PUBLIC hikrobot/lib/arm64)
else()
  #FATAL_ERROR	message(FATAL_ERROR "OpenCV 未找到！")	输出红色错误信息，立即终止编译	核心依赖（如 OpenCV）未找到时，终止编译并提示
  #SEND_ERROR	message(SEND_ERROR "ROS 2 版本过低！")	输出红色错误信息，继续编译但标记构建失败	非致命错误（如可选依赖版本不兼容）
  #WARNING	message(WARNING "未找到 screen，自启功能将失效")	输出黄色警告信息，编译继续	可选依赖缺失、配置不完整，但不影响核心功能
  #STATUS（默认）	message(STATUS "编译模式：${CMAKE_BUILD_TYPE}")	输出普通信息（通常带 -- 前缀），编译继续	提示编译状态、变量值、功能开关（如 ROS 2 是否启用）
  #DEBUG	message(DEBUG "Eigen 头文件路径：${EIGEN3_INCLUDE_DIR}")	仅在 cmake --debug-output 时输出，默认隐藏	调试 CMake 配置（如找库路径、变量值）
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

# mindvision
#target_include_directories(io PUBLIC mindvision/include)
#if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  #target_link_directories(io PUBLIC mindvision/lib/amd64)
#elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  #target_link_directories(io PUBLIC mindvision/lib/arm64)
#else()
  #message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
#endif()

# dahua
target_include_directories(io PUBLIC dahua/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC dahua/lib/amd64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

target_link_libraries(io MvCameraControl MVSDK ImageConvert usb-1.0 yaml-cpp serial "/opt/DahuaTech/MVviewer/lib/libMVSDK.so")

################## 检查 ROS 环境 ##################
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)
find_package(sp_msgs QUIET)

# 如果找不到 ROS 环境，则跳过 ROS 相关的部分
if(NOT ament_cmake_FOUND OR NOT rclcpp_FOUND OR NOT std_msgs_FOUND OR NOT rosidl_typesupport_cpp_FOUND OR NOT sp_msgs_FOUND)
    message(WARNING "ROS2 not found, skipping ROS2 specific code.")
else()
    message(STATUS "ROS2 environment found, compiling ROS2-related code.")
    
    # 添加 ROS2 相关的文件
    target_sources(io PRIVATE ros2/publish2nav.cpp ros2/ros2.cpp ros2/subscribe2nav.cpp)

    # 设置 ROS2 依赖
    #ament_target_dependencies 是 ROS2 专属的 CMake 命令，它的作用是：
    #✅ 自动为 io 目标添加 ROS2 头文件路径（不用手动写 /opt/ros/humble/include）；
    #✅ 自动为 io 目标链接 rclcpp/std_msgs/sp_msgs 库（不用手动 target_link_libraries）；
    #✅ 自动处理 ROS2 库的依赖关系（比手动加路径 / 链接更可靠）。
    include_directories(${std_msgs_INCLUDE_DIRS})
    ament_target_dependencies(io rclcpp std_msgs sp_msgs)
endif()
