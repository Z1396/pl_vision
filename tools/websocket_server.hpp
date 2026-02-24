#pragma once

#include <string>
#include <functional>
#include <thread>
#include <mutex>
#include <set>
#include <map>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace tools {

class WebSocketServer {
public:
    WebSocketServer(int port);
    ~WebSocketServer();

    void start();
    void stop();
    
    void broadcastVideo(const std::string& jpeg_data);
    void broadcastData(const json& data);

private:
    void acceptConnections();
    void handleConnection(int client_fd);
    void monitorConnections();
    
    int port_;
    int server_fd_;
    std::set<int> clients_;
    std::map<int, bool> handshake_completed_;
    std::mutex clients_mutex_;
    std::thread accept_thread_;
    std::thread monitor_thread_;
    bool running_;
    
    static std::string createHandshakeResponse(const std::string& key);
    static std::string encodeFrame(const std::string& payload);
    static std::string base64Encode(const unsigned char* data, size_t len);
    static void sha1(const unsigned char* data, size_t len, unsigned char* hash);
};

}