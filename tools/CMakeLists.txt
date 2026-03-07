cmake_minimum_required(VERSION 3.16)

add_library(tools OBJECT 
    exiter.cpp
    extended_kalman_filter.cpp
    ransac_sine_fitter.cpp
    img_tools.cpp
    math_tools.cpp
    plotter.cpp
    trajectory.cpp
    recorder.cpp
    logger.cpp
    pid.cpp
    crc.cpp
    websocket_server.cpp
)

#ssl 和 crypto 库：
#这是 OpenSSL 库的两大核心组件：
#crypto：提供加密 / 解密核心算法（如 AES、RSA、SHA 等）；
#ssl：基于 crypto 封装，提供 TLS/SSL 协议相关功能（如 HTTPS 通信、证书验证等）。
#链接这两个库后，你的 tools 目标就能调用 OpenSSL 的 API 实现加密、SSL 通信等功能。
target_link_libraries(tools PRIVATE ssl crypto)