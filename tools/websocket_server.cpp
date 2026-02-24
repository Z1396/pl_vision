#include "websocket_server.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <cstring>
#include <signal.h>
#include <errno.h>
#include <openssl/sha.h>
#include <tools/logger.hpp>

namespace tools {

const std::string MAGIC_STRING = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

WebSocketServer::WebSocketServer(int port) 
    : port_(port), server_fd_(-1), running_(false) {
}

WebSocketServer::~WebSocketServer() {
    stop();
}

void WebSocketServer::start() {
    signal(SIGPIPE, SIG_IGN);
    
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ == -1) {
        tools::logger()->error("Failed to create socket");
        return;
    }

    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in address;
    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port_);

    if (bind(server_fd_, (struct sockaddr*)&address, sizeof(address)) < 0) {
        tools::logger()->error("Failed to bind to port {}", port_);
        close(server_fd_);
        return;
    }

    if (listen(server_fd_, 10) < 0) {
        tools::logger()->error("Failed to listen");
        close(server_fd_);
        return;
    }

    running_ = true;
    accept_thread_ = std::thread(&WebSocketServer::acceptConnections, this);
    tools::logger()->info("WebSocket server started on port {}", port_);
}

void WebSocketServer::stop() {
    running_ = false;
    
    if (accept_thread_.joinable()) {
        accept_thread_.join();
    }
    
    if (monitor_thread_.joinable()) {
        monitor_thread_.join();
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

void WebSocketServer::acceptConnections() {
    while (running_) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        
        int client_fd = accept(server_fd_, (struct sockaddr*)&client_addr, &client_len);
        if (client_fd < 0) {
            if (running_) {
                tools::logger()->error("Failed to accept connection");
            }
            continue;
        }

        fcntl(client_fd, F_SETFL, O_NONBLOCK);

        std::lock_guard<std::mutex> lock(clients_mutex_);
        clients_.insert(client_fd);
        
        tools::logger()->info("Client connected: {}", client_fd);
        
        std::thread([this, client_fd]() {
            handleConnection(client_fd);
        }).detach();
    }
}

void WebSocketServer::handleConnection(int client_fd) {
    char buffer[4096];
    std::string handshake;
    
    while (running_) {
        ssize_t bytes_read = recv(client_fd, buffer, sizeof(buffer) - 1, 0);
        if (bytes_read <= 0) {
            if (bytes_read < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                usleep(10000);
                continue;
            }
            tools::logger()->info("recv returned: {}, errno: {}", bytes_read, errno);
            break;
        }
        
        buffer[bytes_read] = '\0';
        handshake += buffer;
        
        tools::logger()->info("Received handshake data: {} bytes", bytes_read);
        
        if (handshake.find("\r\n\r\n") != std::string::npos) {
            size_t key_pos = handshake.find("Sec-WebSocket-Key:");
            if (key_pos != std::string::npos) {
                size_t key_start = handshake.find(":", key_pos) + 1;
                size_t key_end = handshake.find("\r\n", key_start);
                std::string key = handshake.substr(key_start, key_end - key_start);
                
                while (!key.empty() && key[0] == ' ') key.erase(0, 1);
                while (!key.empty() && key.back() == ' ') key.pop_back();
                
                std::string response = createHandshakeResponse(key);
                send(client_fd, response.c_str(), response.length(), 0);
                
                tools::logger()->info("WebSocket handshake completed for client {}", client_fd);
                
                std::lock_guard<std::mutex> lock(clients_mutex_);
                handshake_completed_[client_fd] = true;
                
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
                            break;
                        }
                    }
                    
                    std::lock_guard<std::mutex> lock(clients_mutex_);
                    clients_.erase(client_fd);
                    handshake_completed_.erase(client_fd);
                    close(client_fd);
                    tools::logger()->info("Client disconnected: {}", client_fd);
                }).detach();
                
                return;
            } else {
                tools::logger()->error("Sec-WebSocket-Key not found in handshake");
            }
        }
    }
    
    std::lock_guard<std::mutex> lock(clients_mutex_);
    clients_.erase(client_fd);
    handshake_completed_.erase(client_fd);
    close(client_fd);
    
    tools::logger()->info("Client disconnected: {}", client_fd);
}

void WebSocketServer::monitorConnections() {
    while (running_) {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        
        std::set<int> disconnected;
        for (int client_fd : clients_) {
            if (handshake_completed_[client_fd]) {
                int error = 0;
                socklen_t len = sizeof(error);
                int retval = getsockopt(client_fd, SOL_SOCKET, SO_ERROR, &error, &len);
                
                if (retval != 0 || error != 0) {
                    tools::logger()->info("Client {} disconnected (getsockopt error: {})", client_fd, error);
                    disconnected.insert(client_fd);
                }
            }
        }
        
        for (int client_fd : disconnected) {
            clients_.erase(client_fd);
            handshake_completed_.erase(client_fd);
            close(client_fd);
            tools::logger()->info("Client disconnected: {}", client_fd);
        }
        
        usleep(100000);
    }
}

std::string WebSocketServer::createHandshakeResponse(const std::string& key) {
    std::string accept_key = key + MAGIC_STRING;
    
    unsigned char hash[20];
    sha1(reinterpret_cast<const unsigned char*>(accept_key.c_str()), accept_key.length(), hash);
    
    std::string encoded_key = base64Encode(hash, 20);
    
    std::string response = 
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Accept: " + encoded_key + "\r\n"
        "\r\n";
    
    return response;
}

std::string WebSocketServer::encodeFrame(const std::string& payload) {
    std::string frame;
    
    frame.push_back(0x81);
    
    if (payload.length() <= 125) {
        frame.push_back(static_cast<char>(payload.length()));
    } else if (payload.length() <= 65535) {
        frame.push_back(126);
        frame.push_back(static_cast<char>((payload.length() >> 8) & 0xFF));
        frame.push_back(static_cast<char>(payload.length() & 0xFF));
    } else {
        frame.push_back(127);
        for (int i = 7; i >= 0; i--) {
            frame.push_back(static_cast<char>((payload.length() >> (i * 8)) & 0xFF));
        }
    }
    
    frame += payload;
    return frame;
}

std::string WebSocketServer::base64Encode(const unsigned char* data, size_t len) {
    const char* base64_chars = 
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789+/";
    
    std::string result;
    int i = 0;
    size_t j = 0;
    unsigned char char_array_3[3];
    unsigned char char_array_4[4];
    
    while (len--) {
        char_array_3[i++] = *(data++);
        if (i == 3) {
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;
            
            for (i = 0; i < 4; i++) {
                result += base64_chars[char_array_4[i]];
            }
            
            i = 0;
        }
    }
    
    if (i) {
        for (j = i; j < 3; j++) {
            char_array_3[j] = '\0';
        }
        
        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;
        
        for (j = 0; j < i + 1; j++) {
            result += base64_chars[char_array_4[j]];
        }
        
        while (i++ < 3) {
            result += '=';
        }
    }
    
    return result;
}

void WebSocketServer::sha1(const unsigned char* data, size_t len, unsigned char* hash) {
    SHA1(data, len, hash);
}

void WebSocketServer::broadcastVideo(const std::string& jpeg_data) {
    json data;
    data["type"] = "video";
    data["data"] = base64Encode(reinterpret_cast<const unsigned char*>(jpeg_data.c_str()), jpeg_data.length());
    
    std::string payload = data.dump();
    std::string frame = encodeFrame(payload);
    
    std::lock_guard<std::mutex> lock(clients_mutex_);
    tools::logger()->info("Broadcasting video to {} clients, payload size: {} bytes, frame size: {} bytes", 
                        clients_.size(), payload.length(), frame.length());
    
    std::vector<int> disconnected;
    for (int client_fd : clients_) {
        size_t total_sent = 0;
        while (total_sent < frame.length()) {
            ssize_t sent = send(client_fd, frame.c_str() + total_sent, frame.length() - total_sent, 0);
            if (sent < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    usleep(1000);
                    continue;
                }
                tools::logger()->error("Failed to send video to client {}: errno {}", client_fd, errno);
                disconnected.push_back(client_fd);
                break;
            }
            total_sent += sent;
        }
    }
    
    for (int client_fd : disconnected) {
        clients_.erase(client_fd);
        handshake_completed_.erase(client_fd);
        close(client_fd);
        tools::logger()->info("Client disconnected: {}", client_fd);
    }
}

void WebSocketServer::broadcastData(const json& data) {
    json message;
    message["type"] = "data";
    message["data"] = data;
    
    std::string payload = message.dump();
    std::string frame = encodeFrame(payload);
    
    std::lock_guard<std::mutex> lock(clients_mutex_);
    tools::logger()->info("Broadcasting data to {} clients, payload size: {} bytes, frame size: {} bytes", 
                        clients_.size(), payload.length(), frame.length());
    
    std::vector<int> disconnected;
    for (int client_fd : clients_) {
        size_t total_sent = 0;
        while (total_sent < frame.length()) {
            ssize_t sent = send(client_fd, frame.c_str() + total_sent, frame.length() - total_sent, 0);
            if (sent < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    usleep(1000);
                    continue;
                }
                tools::logger()->error("Failed to send data to client {}: errno {}", client_fd, errno);
                disconnected.push_back(client_fd);
                break;
            }
            total_sent += sent;
        }
    }
    
    for (int client_fd : disconnected) {
        clients_.erase(client_fd);
        handshake_completed_.erase(client_fd);
        close(client_fd);
        tools::logger()->info("Client disconnected: {}", client_fd);
    }
}

}