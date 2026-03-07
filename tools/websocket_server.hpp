/**
 * @file websocket_server.hpp
 * @brief WebSocket服务器头文件
 * 
 * 本文件定义了WebSocket服务器类的接口，用于机器人视觉系统的实时数据广播。
 * 支持的功能：
 * - 多客户端WebSocket连接管理
 * - 实时视频流广播（JPEG格式，Base64编码）
 * - JSON数据广播（机器人状态、目标信息等）
 * - 自动连接监控和断开处理
 * 
 * WebSocket协议实现参考：RFC 6455
 */

#pragma once

// 标准库头文件
#include <string>            // std::string：字符串处理
#include <functional>        // std::function：函数对象（预留扩展）
#include <thread>            // std::thread：多线程支持
#include <mutex>             // std::mutex：线程同步
#include <condition_variable> // std::condition_variable：条件变量
#include <set>               // std::set：客户端集合管理
#include <map>               // std::map：客户端握手状态映射

// 第三方库：nlohmann/json，用于JSON数据的序列化和反序列化
#include <nlohmann/json.hpp>

// 类型别名：简化json类型的书写
using json = nlohmann::json;

namespace tools {

/**
 * @class WebSocketServer
 * @brief WebSocket服务器类
 * 
 * 基于原始Socket实现的WebSocket服务器，支持多客户端连接和实时数据广播。
 * 线程安全设计，使用互斥锁保护共享数据。
 * 
 * 使用示例：
 * @code
 *   WebSocketServer server(8080);  // 创建服务器，监听8080端口
 *   server.start();                 // 启动服务器
 *   server.broadcastVideo(jpeg);    // 广播视频帧
 *   server.broadcastData(json_obj); // 广播JSON数据
 *   server.stop();                  // 停止服务器
 * @endcode
 */
class WebSocketServer {
public:
    /**
     * @brief 构造函数
     * @param port 服务器监听端口（例如：8080）
     * 
     * 初始化服务器，但不启动监听。需要调用start()方法开始服务。
     */
    WebSocketServer(int port);
    
    /**
     * @brief 析构函数
     * 
     * 自动调用stop()方法，确保服务器停止并释放所有资源。
     * 关闭所有客户端连接，停止后台线程。
     */
    ~WebSocketServer();

    /**
     * @brief 启动WebSocket服务器
     * 
     * 启动流程：
     * 1. 创建TCP socket
     * 2. 绑定到指定端口
     * 3. 开始监听连接
     * 4. 启动接受连接的独立线程
     * 
     * 注意：该方法是非阻塞的，启动后会立即返回。
     * 实际的连接处理在后台线程中进行。
     */
    void start();
    
    /**
     * @brief 停止WebSocket服务器
     * 
     * 停止流程：
     * 1. 设置停止标志，通知所有线程退出
     * 2. 等待后台线程结束
     * 3. 关闭所有客户端连接
     * 4. 关闭服务器socket
     * 
     * 注意：该方法是阻塞的，会等待所有资源清理完成。
     */
    void stop();
    
    /**
     * @brief 广播视频帧到所有连接的客户端
     * @param jpeg_data JPEG格式的图像数据（二进制字符串）
     * 
     * 处理流程：
     * 1. 将JPEG数据进行Base64编码（JSON只支持文本）
     * 2. 构建包含类型和数据的JSON消息
     * 3. 编码为WebSocket帧
     * 4. 发送到所有已完成握手的客户端
     * 
     * 消息格式：{"type": "video", "data": "base64encoded..."}
     * 
     * 注意：该方法是线程安全的，可以在任意线程中调用。
     */
    void broadcastVideo(const std::string& jpeg_data);
    
    /**
     * @brief 广播JSON数据到所有连接的客户端
     * @param data 要发送的JSON数据（机器人状态、目标信息等）
     * 
     * 处理流程与broadcastVideo类似，但直接接收JSON对象。
     * 用于发送结构化的调试数据（FPS、姿态、目标位置等）。
     * 
     * 消息格式：{"type": "data", "data": {...}}
     * 
     * 注意：该方法是线程安全的，可以在任意线程中调用。
     */
    void broadcastData(const json& data);

private:
    /**
     * @brief 接受客户端连接的线程函数
     * 
     * 在独立线程中运行，循环调用accept()接受新连接。
     * 每接受一个新连接，就创建一个独立线程处理该连接。
     */
    void acceptConnections();
    
    /**
     * @brief 处理单个客户端连接的线程函数
     * @param client_fd 客户端socket文件描述符
     * 
     * 处理流程：
     * 1. 接收HTTP升级请求（WebSocket握手）
     * 2. 解析Sec-WebSocket-Key
     * 3. 计算响应Key并发送握手响应
     * 4. 标记握手完成
     * 5. 启动接收线程保持连接
     */
    void handleConnection(int client_fd);
    
    /**
     * @brief 监控客户端连接的线程函数
     * 
     * 定期检查所有已连接客户端的状态，
     * 使用getsockopt(SO_ERROR)检测socket错误状态，
     * 发现断开的连接时，从管理集合中移除。
     * 
     * 检查周期：100ms
     */
    void monitorConnections();
    
    /**
     * @brief 异步广播线程函数
     * 
     * 从消息队列中取出消息并广播到所有客户端。
     * 使用条件变量等待新消息，避免忙等待。
     * 将耗时的编码和网络发送操作从主线程分离。
     */
    void broadcastLoop();
    
    // ==================== 成员变量 ====================
    
    int port_;                              ///< 服务器监听端口
    int server_fd_;                         ///< 服务器socket文件描述符（-1表示未创建）
    std::set<int> clients_;                 ///< 已连接客户端的socket文件描述符集合
    std::map<int, bool> handshake_completed_; ///< 客户端握手完成状态映射（fd -> 是否完成握手）
    std::mutex clients_mutex_;              ///< 保护clients_和handshake_completed_的互斥锁
    std::thread accept_thread_;             ///< 接受连接的独立线程
    std::thread monitor_thread_;            ///< 连接监控的独立线程
    std::thread broadcast_thread_;          ///< 异步广播线程
    bool running_;                          ///< 服务器运行状态标志
    
    // ==================== 异步广播（双缓冲机制） ====================
    std::string latest_video_jpeg_;    ///< 最新的 JPEG 数据（未编码）
    std::string latest_data_json_;     ///< 最新的数据 JSON（未编码）
    std::mutex broadcast_mutex_;       ///< 保护广播数据
    std::condition_variable frame_cv_; ///< 新帧通知
    bool has_new_video_ = false;       ///< 是否有新视频帧
    bool has_new_data_ = false;        ///< 是否有新数据帧
    
    // ==================== 静态工具函数 ====================
    
    /**
     * @brief 创建WebSocket握手响应
     * @param key 客户端发送的Sec-WebSocket-Key
     * @return HTTP 101响应字符串
     * 
     * 根据RFC 6455规范：
     * 1. 将客户端Key与魔串拼接
     * 2. 计算SHA1哈希
     * 3. Base64编码
     * 4. 构建HTTP 101响应
     */
    static std::string createHandshakeResponse(const std::string& key);
    
    /**
     * @brief 编码WebSocket数据帧（文本帧）
     * @param payload 要发送的原始数据
     * @return 编码后的WebSocket帧
     * 
     * 根据RFC 6455编码WebSocket帧：
     * - FIN=1（最后一帧）
     * - opcode=0x1（文本帧）
     * - MASK=0（服务器发送不需要掩码）
     * - 支持3种payload长度编码：7位、16位、64位
     */
    static std::string encodeFrame(const std::string& payload);
    
    /**
     * @brief Base64编码实现
     * @param data 原始二进制数据
     * @param len 数据长度
     * @return Base64编码后的字符串
     * 
     * 将二进制数据编码为Base64字符串，用于：
     * 1. WebSocket握手中的SHA1哈希编码
     * 2. 视频帧的JSON传输编码
     */
    static std::string base64Encode(const unsigned char* data, size_t len);
    
    /**
     * @brief SHA1哈希计算
     * @param data 输入数据
     * @param len 数据长度
     * @param hash 输出哈希值（20字节）
     * 
     * 使用OpenSSL的SHA1函数计算哈希，
     * 用于WebSocket握手响应的生成。
     */
    static void sha1(const unsigned char* data, size_t len, unsigned char* hash);
};

} // namespace tools
