/**
 * @file websocket_server.cpp
 * @brief WebSocket服务器实现文件
 * 
 * 本文件实现了基于原始Socket的WebSocket服务器，用于机器人视觉系统的实时数据广播。
 * 支持的功能：
 * - 多客户端WebSocket连接管理
 * - 实时视频流广播（JPEG格式，Base64编码）
 * - JSON数据广播（机器人状态、目标信息等）
 * - 自动连接监控和断开处理
 * 
 * WebSocket协议实现参考：RFC 6455
 */

#include "websocket_server.hpp"

// 系统网络库头文件
#include <sys/socket.h>      // Socket API：socket(), bind(), listen(), accept(), send(), recv()
#include <netinet/in.h>      // 网络地址结构：sockaddr_in, htons(), INADDR_ANY
#include <netinet/tcp.h>     // TCP选项：TCP_NODELAY
#include <arpa/inet.h>       // 网络字节序转换
#include <unistd.h>          // Unix标准函数：close(), usleep()
#include <fcntl.h>           // 文件控制：fcntl()用于设置非阻塞模式
#include <string.h>          // 字符串操作：memset()
#include <cstring>           // C++字符串操作
#include <signal.h>          // 信号处理：signal()
#include <errno.h>           // 错误码：errno

// OpenSSL库：用于SHA1哈希计算（WebSocket握手需要）
#include <openssl/sha.h>

// 项目日志工具
#include <tools/logger.hpp>

namespace tools {

/**
 * @brief WebSocket协议魔数字符串
 * 
 * 根据RFC 6455，WebSocket握手需要将客户端发送的Key与这个魔串拼接，
 * 然后计算SHA1哈希，最后进行Base64编码，作为响应的Accept-Key
 */
const std::string MAGIC_STRING = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

/**
 * @brief 构造函数
 * @param port 服务器监听端口（默认8080）
 * 
 * 初始化成员变量：
 * - port_: 监听端口
 * - server_fd_: 服务器socket文件描述符（初始-1表示未创建）
 * - running_: 服务器运行状态标志
 */
WebSocketServer::WebSocketServer(int port) 
    : port_(port), server_fd_(-1), running_(false) {
}

/**
 * @brief 析构函数
 * 
 * 确保服务器停止，释放所有资源。
 * 调用stop()方法关闭socket、停止线程、清理客户端连接。
 */
WebSocketServer::~WebSocketServer() {
    stop();
}

/**
 * @brief 启动WebSocket服务器
 * 
 * 启动流程：
 * 1. 忽略SIGPIPE信号（防止向已关闭连接写数据导致程序崩溃）
 * 2. 创建TCP socket
 * 3. 设置端口复用（SO_REUSEADDR），允许快速重启
 * 4. 绑定到指定端口
 * 5. 开始监听连接（最大等待队列长度10）
 * 6. 启动接受连接的独立线程
 */
void WebSocketServer::start() 
{
    // 忽略SIGPIPE信号：当向已关闭的socket写数据时，不发送SIGPIPE信号，而是返回EPIPE错误
    signal(SIGPIPE, SIG_IGN);
    
    // 创建TCP socket：AF_INET=IPv4, SOCK_STREAM=TCP协议
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ == -1) {
        tools::logger()->error("Failed to create socket");
        return;
    }

    // 设置端口复用选项：允许socket绑定到处于TIME_WAIT状态的端口
    // 这样服务器可以快速重启而不会遇到"Address already in use"错误
    /*三、SO_REUSEADDR 解决的核心问题
    1. 端口占用（Address already in use）
    问题场景：服务器程序退出后，端口可能因处于 TIME_WAIT 状态（TCP 四次挥手后的状态，默认约 2 分钟）而无法立即重启，报错 bind(): Address already in use。
    解决原理：SO_REUSEADDR 允许内核复用处于 TIME_WAIT 状态的端口，让服务器能快速重启。
    2. 多进程 / 多线程绑定同一端口
    场景：多进程（如 fork 子进程）处理客户端连接时，多个进程绑定同一个 IP + 端口，避免端口冲突。
    注意：需配合 bind() 调用，且所有进程都要设置该选项。*/
    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // 配置服务器地址结构
    struct sockaddr_in address;
    memset(&address, 0, sizeof(address));           // 清零结构体
    address.sin_family = AF_INET;                   // IPv4地址族
    address.sin_addr.s_addr = INADDR_ANY;           // 监听所有网络接口（0.0.0.0）
    address.sin_port = htons(port_);                // 端口号转换为网络字节序（大端）

    // 绑定socket到指定地址和端口
    if (bind(server_fd_, (struct sockaddr*)&address, sizeof(address)) < 0) 
    {
        tools::logger()->error("Failed to bind to port {}", port_);
        close(server_fd_);
        return;
    }

    // 开始监听传入连接，backlog=10表示最大等待连接队列长度
    if (listen(server_fd_, 10) < 0) 
    {
        tools::logger()->error("Failed to listen");
        close(server_fd_);
        return;
    }

    // 设置运行标志，启动接受连接的独立线程
    running_ = true;
    accept_thread_ = std::thread(&WebSocketServer::acceptConnections, this);
    broadcast_thread_ = std::thread(&WebSocketServer::broadcastLoop, this);
    tools::logger()->info("WebSocket server started on port {}", port_);
}

/**
 * @brief 停止WebSocket服务器
 * 
 * 停止流程：
 * 1. 设置running_标志为false，通知所有线程退出
 * 2. 等待接受线程结束
 * 3. 等待监控线程结束
 * 4. 关闭所有客户端连接
 * 5. 关闭服务器socket
 */
void WebSocketServer::stop() {
    running_ = false;
    
    frame_cv_.notify_all();
    
    if (accept_thread_.joinable()) {
        accept_thread_.join();
    }
    
    if (monitor_thread_.joinable()) {
        monitor_thread_.join();
    }
    
    if (broadcast_thread_.joinable()) {
        broadcast_thread_.join();
    }
    
    std::lock_guard<std::mutex> lock(clients_mutex_);
    for (int client_fd : clients_) {
        close(client_fd);
    }
    clients_.clear();
    handshake_completed_.clear();
    
    if (server_fd_ != -1) {
        close(server_fd_);
        server_fd_ = -1;
    }
    
    tools::logger()->info("WebSocket server stopped");
}

/**
 * @brief 接受客户端连接的线程函数
 * 
 * 在主循环中：
 * 1. 调用accept()阻塞等待新连接（实际设置了非阻塞，会轮询）
 * 2. 设置新连接为非阻塞模式
 * 3. 将客户端加入管理集合
 * 4. 创建独立线程处理该连接
 * 
 * 每个客户端连接都有一个独立的处理线程
 */
void WebSocketServer::acceptConnections() {
    while (running_) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        
        // 接受新连接
        int client_fd = accept(server_fd_, (struct sockaddr*)&client_addr, &client_len);
        if (client_fd < 0) {
            if (running_) {
                tools::logger()->error("Failed to accept connection");
            }
            continue;
        }

        // 设置客户端socket为非阻塞模式
        // 这样recv()不会阻塞，而是立即返回（如果没有数据则返回EAGAIN）
        fcntl(client_fd, F_SETFL, O_NONBLOCK);
        
        // 禁用Nagle算法，降低延迟
        int flag = 1;
        setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
        
        // 设置发送缓冲区大小
        int sndbuf_size = 256 * 1024;  // 256KB
        setsockopt(client_fd, SOL_SOCKET, SO_SNDBUF, &sndbuf_size, sizeof(sndbuf_size));

        // 加锁将新客户端加入管理集合
        std::lock_guard<std::mutex> lock(clients_mutex_);
        clients_.insert(client_fd);
        
        tools::logger()->info("Client connected: {}", client_fd);
        
        // 创建独立线程处理该客户端的连接
        // 使用detach()让线程独立运行，不需要等待它结束
        std::thread([this, client_fd]() {
            handleConnection(client_fd);
        }).detach();
    }
}

/**
 * @brief 处理单个客户端连接的线程函数
 * @param client_fd 客户端socket文件描述符
 * 
 * 处理流程：
 * 1. 接收HTTP升级请求（WebSocket握手）
 * 2. 解析Sec-WebSocket-Key
 * 3. 计算响应Key并发送握手响应
 * 4. 标记握手完成
 * 5. 启动心跳/接收线程保持连接
 * 
 * WebSocket握手协议（RFC 6455）：
 * 客户端发送：GET / HTTP/1.1
 *            Host: server.example.com
 *            Upgrade: websocket
 *            Connection: Upgrade
 *            Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==
 *            Sec-WebSocket-Version: 13
 * 
 * 服务器响应：HTTP/1.1 101 Switching Protocols
 *            Upgrade: websocket
 *            Connection: Upgrade
 *            Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo=
 */
void WebSocketServer::handleConnection(int client_fd) {
    char buffer[4096];           // 接收缓冲区
    std::string handshake;       // 累积接收的握手数据
    
    while (running_) {
        // 接收数据，非阻塞模式
        ssize_t bytes_read = recv(client_fd, buffer, sizeof(buffer) - 1, 0);
        if (bytes_read <= 0) {
            // 没有数据可读（非阻塞模式正常情况）
            if (bytes_read < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                usleep(10000);   // 等待10ms再试
                continue;
            }
            // 连接关闭或出错
            tools::logger()->info("recv returned: {}, errno: {}", bytes_read, errno);
            break;
        }
        
        buffer[bytes_read] = '\0';
        handshake += buffer;
        
        tools::logger()->info("Received handshake data: {} bytes", bytes_read);
        
        // 检查是否收到完整的HTTP请求头（以\r\n\r\n结尾）
        if (handshake.find("\r\n\r\n") != std::string::npos) {
            // 查找Sec-WebSocket-Key头
            size_t key_pos = handshake.find("Sec-WebSocket-Key:");
            if (key_pos != std::string::npos) {
                // 提取Key值
                size_t key_start = handshake.find(":", key_pos) + 1;
                size_t key_end = handshake.find("\r\n", key_start);
                std::string key = handshake.substr(key_start, key_end - key_start);
                
                // 去除首尾空格
                while (!key.empty() && key[0] == ' ') key.erase(0, 1);
                while (!key.empty() && key.back() == ' ') key.pop_back();
                
                // 创建握手响应
                std::string response = createHandshakeResponse(key);
                send(client_fd, response.c_str(), response.length(), 0);
                
                tools::logger()->info("WebSocket handshake completed for client {}", client_fd);
                
                // 标记握手完成
                std::lock_guard<std::mutex> lock(clients_mutex_);
                handshake_completed_[client_fd] = true;
                
                // 启动接收线程，保持连接并检测断开
                std::thread([this, client_fd]() {
                    while (running_) {
                        char recv_buffer[4096];
                        ssize_t recv_bytes = recv(client_fd, recv_buffer, sizeof(recv_buffer), 0);
                        if (recv_bytes < 0) {
                            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                                usleep(10000);
                                continue;
                            }
                            break;
                        } else if (recv_bytes == 0) {
                            // 客户端关闭连接
                            break;
                        }
                    }
                    
                    // 连接断开，清理资源
                    std::lock_guard<std::mutex> lock(clients_mutex_);
                    clients_.erase(client_fd);
                    handshake_completed_.erase(client_fd);
                    close(client_fd);
                    tools::logger()->info("Client disconnected: {}", client_fd);
                }).detach();
                
                return;  // 握手完成，退出处理函数
            } else {
                tools::logger()->error("Sec-WebSocket-Key not found in handshake");
            }
        }
    }
    
    // 循环退出（服务器停止或出错），清理资源
    std::lock_guard<std::mutex> lock(clients_mutex_);
    clients_.erase(client_fd);
    handshake_completed_.erase(client_fd);
    close(client_fd);
    
    tools::logger()->info("Client disconnected: {}", client_fd);
}

/**
 * @brief 监控客户端连接的线程函数
 * 
 * 定期检查所有已连接客户端的状态：
 * 1. 使用getsockopt(SO_ERROR)检测socket错误状态
 * 2. 发现断开的连接时，从管理集合中移除
 * 
 * 检查周期：100ms
 */
void WebSocketServer::monitorConnections() {
    while (running_) {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        
        std::set<int> disconnected;
        for (int client_fd : clients_) {
            if (handshake_completed_[client_fd]) {
                // 获取socket错误状态
                int error = 0;
                socklen_t len = sizeof(error);
                int retval = getsockopt(client_fd, SOL_SOCKET, SO_ERROR, &error, &len);
                
                // 如果有错误，标记为断开
                if (retval != 0 || error != 0) {
                    tools::logger()->info("Client {} disconnected (getsockopt error: {})", client_fd, error);
                    disconnected.insert(client_fd);
                }
            }
        }
        
        // 清理断开的连接
        for (int client_fd : disconnected) {
            clients_.erase(client_fd);
            handshake_completed_.erase(client_fd);
            close(client_fd);
            tools::logger()->info("Client disconnected: {}", client_fd);
        }
        
        usleep(100000);  // 100ms检查一次
    }
}

/**
 * @brief 创建WebSocket握手响应
 * @param key 客户端发送的Sec-WebSocket-Key
 * @return HTTP响应字符串
 * 
 * 根据RFC 6455规范：
 * 1. 将客户端Key与魔串拼接
 * 2. 计算SHA1哈希
 * 3. Base64编码
 * 4. 构建HTTP 101响应
 */
std::string WebSocketServer::createHandshakeResponse(const std::string& key) {
    // 拼接Key和魔串
    std::string accept_key = key + MAGIC_STRING;
    
    // 计算SHA1哈希（20字节）
    unsigned char hash[20];
    sha1(reinterpret_cast<const unsigned char*>(accept_key.c_str()), accept_key.length(), hash);
    
    // Base64编码
    std::string encoded_key = base64Encode(hash, 20);
    
    // 构建HTTP响应
    std::string response = 
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Accept: " + encoded_key + "\r\n"
        "\r\n";
    
    return response;
}

/**
 * @brief 编码WebSocket数据帧（文本帧）
 * @param payload 要发送的原始数据
 * @return 编码后的WebSocket帧
 * 
 * WebSocket帧格式（RFC 6455）：
 * 
 *  0                   1                   2                   3
 *  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 * +-+-+-+-+-------+-+-------------+-------------------------------+
 * |F|R|R|R| opcode|M| Payload len |    Extended payload length    |
 * |I|S|S|S|  (4)  |A|     (7)     |             (16/64)           |
 * |N|V|V|V|       |S|             |   (if payload len==126/127)   |
 * | |1|2|3|       |K|             |                               |
 * +-+-+-+-+-------+-+-------------+ - - - - - - - - - - - - - - - +
 * |     Extended payload length continued, if payload len == 127  |
 * + - - - - - - - - - - - - - - - +-------------------------------+
 * |                               |Masking-key, if MASK set to 1  |
 * +-------------------------------+-------------------------------+
 * | Masking-key (continued)       |          Payload Data         |
 * +-------------------------------- - - - - - - - - - - - - - - - +
 * :                     Payload Data continued ...                :
 * + - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - +
 * |                     Payload Data continued ...                |
 * +---------------------------------------------------------------+
 * 
 * 本实现：
 * - FIN=1（最后一帧）
 * - opcode=0x1（文本帧）
 * - MASK=0（服务器发送不需要掩码）
 * - 支持3种payload长度编码：7位、16位、64位
 */
std::string WebSocketServer::encodeFrame(const std::string& payload) {
    std::string frame;
    
    // 第一个字节：FIN=1, RSV=000, opcode=0001（文本帧）
    // 0x81 = 1000 0001
    frame.push_back(0x81);
    
    // 第二个字节：MASK=0, 长度
    if (payload.length() <= 125) {
        // 短帧：直接编码长度（7位）
        frame.push_back(static_cast<char>(payload.length()));
    } else if (payload.length() <= 65535) {
        // 中帧：长度=126，后跟16位长度
        frame.push_back(126);
        frame.push_back(static_cast<char>((payload.length() >> 8) & 0xFF));  // 高8位
        frame.push_back(static_cast<char>(payload.length() & 0xFF));         // 低8位
    } else {
        // 长帧：长度=127，后跟64位长度
        frame.push_back(127);
        for (int i = 7; i >= 0; i--) {
            frame.push_back(static_cast<char>((payload.length() >> (i * 8)) & 0xFF));
        }
    }
    
    // 添加payload数据
    frame += payload;
    return frame;
}

/**
 * @brief Base64编码实现
 * @param data 原始二进制数据
 * @param len 数据长度
 * @return Base64编码后的字符串
 * 
 * Base64编码原理：
 * - 每3个字节（24位）分为4个6位组
 * - 每个6位组映射到Base64字符表（0-63）
 * - 不足3字节时补'='
 */
std::string WebSocketServer::base64Encode(const unsigned char* data, size_t len) {
    // Base64字符表
    const char* base64_chars = 
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789+/";
    
    std::string result;
    int i = 0;
    size_t j = 0;
    unsigned char char_array_3[3];  // 存储3个输入字节
    unsigned char char_array_4[4];  // 存储4个输出索引
    
    // 处理完整的3字节组
    while (len--) {
        char_array_3[i++] = *(data++);
        if (i == 3) {
            // 24位分成4个6位
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;                                    // 第1字节高6位
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4); // 第1字节低2位 + 第2字节高4位
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6); // 第2字节低4位 + 第3字节高2位
            char_array_4[3] = char_array_3[2] & 0x3f;                                           // 第3字节低6位
            
            // 映射到Base64字符
            for (i = 0; i < 4; i++) {
                result += base64_chars[char_array_4[i]];
            }
            
            i = 0;
        }
    }
    
    // 处理剩余字节（1或2个）
    if (i) {
        // 用0填充剩余位置
        for (j = i; j < 3; j++) {
            char_array_3[j] = '\0';
        }
        
        // 同样的编码逻辑
        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;
        
        // 输出已处理的部分
        for (j = 0; j < i + 1; j++) {
            result += base64_chars[char_array_4[j]];
        }
        
        // 补'='
        while (i++ < 3) {
            result += '=';
        }
    }
    
    return result;
}

/**
 * @brief SHA1哈希计算
 * @param data 输入数据
 * @param len 数据长度
 * @param hash 输出哈希值（20字节）
 * 
 * 使用OpenSSL的SHA1函数计算哈希
 */
void WebSocketServer::sha1(const unsigned char* data, size_t len, unsigned char* hash) {
    SHA1(data, len, hash);
}

/**
 * @brief 异步广播线程函数
 * 
 * 从消息队列中取出消息并广播到所有客户端。
 * 使用条件变量等待新消息，避免忙等待。
 */
void WebSocketServer::broadcastLoop() 
{
    while (running_) 
    {
        std::string jpeg_data;
        std::string data_json;
        bool send_video = false;
        bool send_data = false;
        
        {
            std::unique_lock<std::mutex> lock(broadcast_mutex_);
            frame_cv_.wait_for(lock, std::chrono::milliseconds(5), [this] {
                return has_new_video_ || has_new_data_ || !running_;
            });
            
            if (!running_) break;
            
            if (has_new_video_) {
                jpeg_data = latest_video_jpeg_;
                has_new_video_ = false;
                send_video = true;
            }
            
            if (has_new_data_) {
                data_json = std::move(latest_data_json_);
                has_new_data_ = false;
                send_data = true;
            }
        }
        
        if (!send_video && !send_data) continue;
        
        std::string video_frame, data_frame;
        
        if (send_video) {
            json data;
            data["type"] = "video";
            data["data"] = base64Encode(reinterpret_cast<const unsigned char*>(jpeg_data.c_str()), jpeg_data.length());
            video_frame = encodeFrame(data.dump());
        }
        
        if (send_data) {
            json message;
            message["type"] = "data";
            message["data"] = json::parse(data_json);
            data_frame = encodeFrame(message.dump());
        }
        
        std::vector<int> disconnected;
        {
            std::lock_guard<std::mutex> lock(clients_mutex_);
            
            if (send_video && !video_frame.empty()) {
                for (int client_fd : clients_) {
                    if (!handshake_completed_[client_fd]) continue;
                    
                    ssize_t sent = send(client_fd, video_frame.c_str(), video_frame.length(), 
                                       MSG_DONTWAIT | MSG_NOSIGNAL);
                    if (sent < 0) {
                        if (errno != EAGAIN && errno != EWOULDBLOCK) {
                            disconnected.push_back(client_fd);
                        }
                    }
                }
            }
            
            if (send_data && !data_frame.empty()) {
                for (int client_fd : clients_) {
                    if (!handshake_completed_[client_fd]) continue;
                    
                    ssize_t sent = send(client_fd, data_frame.c_str(), data_frame.length(), 
                                       MSG_DONTWAIT | MSG_NOSIGNAL);
                    if (sent < 0) {
                        if (errno != EAGAIN && errno != EWOULDBLOCK) {
                            disconnected.push_back(client_fd);
                        }
                    }
                }
            }
        }
        
        if (!disconnected.empty()) {
            std::lock_guard<std::mutex> lock(clients_mutex_);
            for (int client_fd : disconnected) {
                clients_.erase(client_fd);
                handshake_completed_.erase(client_fd);
                close(client_fd);
            }
        }
    }
}

void WebSocketServer::broadcastVideo(const std::string& jpeg_data) {
    std::lock_guard<std::mutex> lock(broadcast_mutex_);
    latest_video_jpeg_ = jpeg_data;
    has_new_video_ = true;
    frame_cv_.notify_one();
}

void WebSocketServer::broadcastData(const json& data) {
    std::lock_guard<std::mutex> lock(broadcast_mutex_);
    latest_data_json_ = data.dump();
    has_new_data_ = true;
    frame_cv_.notify_one();
}

} // namespace tools
