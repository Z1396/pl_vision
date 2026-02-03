# 构建 TinyMPC 静态库，库名称为 tinympcstatic，STATIC 表示生成静态链接库（.a 文件）
# 静态库特点：编译时将库代码嵌入可执行文件，运行时无需依赖外部库文件，适配嵌入式无动态链接的场景
# 源文件列表：包含 TinyMPC 核心算法实现、API 封装、代码生成、rho 超参数基准测试相关源码
add_library(tinympcstatic STATIC
    admm.cpp          # ADMM 算法核心实现：TinyMPC 采用交替方向乘子法求解 MPC 优化问题
    tiny_api.cpp      # 顶层 API 实现：封装底层算法，对外提供简洁的 MPC 调用接口
    codegen.cpp       # 代码生成核心实现：支持将 MPC 求解逻辑生成为嵌入式可直接调用的轻量化代码
    rho_benchmark.cpp # ADMM 算法 rho 超参数基准测试：自动调优 rho 参数，提升 MPC 求解速度与稳定性
)

# 设置目标属性：为 tinympcstatic 静态库启用位置无关代码（Position-Independent Code, PIC）
# 关键作用：1. 允许静态库被动态库（.so）链接时不出现地址重定位错误；2. 提升嵌入式平台上的代码可移植性
# 嵌入式/跨平台开发中必配，避免不同架构/编译器下的地址访问问题
set_property(TARGET tinympcstatic PROPERTY POSITION_INDEPENDENT_CODE ON)

# 【注释掉的代码】链接 Eigen 线性代数库（PUBLIC 表示库的依赖会传递给链接本库的目标）
# 未启用原因：后续通过直接指定 Eigen 头文件路径的方式集成，无需显式链接（Eigen 是纯头文件库，无编译后的库文件）
# target_link_libraries(tinympcstatic PUBLIC Eigen)

# 添加头文件搜索路径（PUBLIC 表示路径会传递给链接本库的目标，如你的云台项目）
# 1. 添加上级目录：让调用者能通过 #include "tinympc/xxx.hpp" 引入头文件（符合库的目录结构规范）
target_include_directories(tinympcstatic PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
# 2. 显式添加 Eigen 头文件路径：直接指定 Eigen 库的 include 目录，解决头文件查找问题
#    Eigen 是 TinyMPC 的核心依赖（矩阵运算、线性求解），纯头文件实现无需链接，仅需指定头文件路径
target_include_directories(tinympcstatic PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../../include/Eigen)

# 条件编译：仅当顶层 CMakeLists.txt 中定义了 USING_CODEGEN 变量时，执行下方嵌入式代码生成逻辑
# USING_CODEGEN 是跨目录CMake变量，用于全局控制是否启用「嵌入式代码生成」功能
if(USING_CODEGEN) # Defined in top-level CMakeLists.txt

# 定义嵌入式代码生成所需的核心文件列表：包含算法实现、API 头文件、类型定义、常量定义
# 这些文件是生成嵌入式轻量化 MPC 求解代码的基础，仅保留核心逻辑，剔除桌面端调试/测试代码
list( APPEND EMBEDDED_FILES
      "${CMAKE_CURRENT_SOURCE_DIR}/admm.cpp"          # ADMM 核心求解算法（嵌入式核心依赖）
      "${CMAKE_CURRENT_SOURCE_DIR}/admm.hpp"          # ADMM 算法头文件（函数/类声明）
      "${CMAKE_CURRENT_SOURCE_DIR}/tiny_api.cpp"      # 顶层 API 实现（嵌入式对外调用接口）
      "${CMAKE_CURRENT_SOURCE_DIR}/tiny_api.hpp"      # 顶层 API 头文件（核心类/函数声明）
      "${CMAKE_CURRENT_SOURCE_DIR}/types.hpp"         # 核心类型定义：MPC 输入/输出/配置结构体
      "${CMAKE_CURRENT_SOURCE_DIR}/tiny_api_constants.hpp" ) # 全局常量定义：求解器默认参数、约束阈值等

# 遍历嵌入式核心文件列表，逐个将文件复制到嵌入式构建目录
# 目的：将核心源码集中到指定目录，方便后续针对嵌入式平台单独编译、裁剪、优化
foreach( f ${EMBEDDED_FILES} )
    # 获取文件的纯名称（剔除路径，仅保留 xxx.cpp/xxx.hpp）
    get_filename_component( fname ${f} NAME )

    # 定义目标文件路径：嵌入式构建目录 + 文件名，所有核心文件统一输出到该目录
    set( dest_file "${EMBEDDED_BUILD_TINYMPC_DIR}/${fname}" )
    # 将目标文件路径加入列表，用于后续作为自定义目标的依赖
    list( APPEND EMBEDDED_BUILD_TINYMPC_FILES "${dest_file}" )

    # 添加自定义命令：生成目标文件（复制操作）
    # 作用：在 CMake 构建阶段，自动将源文件复制到嵌入式构建目录，无需手动拷贝
    add_custom_command(OUTPUT ${dest_file}          # 自定义命令的输出文件
                        COMMAND ${CMAKE_COMMAND} -E copy "${f}" "${dest_file}" # 执行复制命令（CMake 跨平台复制，替代cp）
                        DEPENDS ${f}                # 依赖：源文件发生修改时，自动重新执行复制
                        COMMENT "Copying ${fname}") # 构建时的终端提示信息，方便查看进度
endforeach()

# 添加自定义目标：copy_codegen_tinympc_files，依赖所有复制后的嵌入式核心文件
# 作用：将分散的复制命令封装为一个目标，方便后续通过该目标触发所有文件的复制操作
add_custom_target( copy_codegen_tinympc_files DEPENDS ${EMBEDDED_BUILD_TINYMPC_FILES} )

# 设置依赖关系：让顶层的 copy_codegen_files 目标依赖当前的 copy_codegen_tinympc_files 目标
# 作用：实现「一键触发所有模块的嵌入式代码复制」，当编译 copy_codegen_files 时，自动先执行本库的文件复制
add_dependencies( copy_codegen_files copy_codegen_tinympc_files )

endif(USING_CODEGEN) # 结束 USING_CODEGEN 条件编译块
