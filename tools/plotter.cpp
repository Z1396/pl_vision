#include "plotter.hpp"

#include <arpa/inet.h>   // htons, inet_addr
#include <sys/socket.h>  // socket, sendto
#include <unistd.h>      // close

namespace tools
{
Plotter::Plotter(std::string host, uint16_t port)
{
  socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);

  destination_.sin_family = AF_INET;
  destination_.sin_port = ::htons(port);
  destination_.sin_addr.s_addr = ::inet_addr(host.c_str());
}

Plotter::~Plotter() { ::close(socket_); }


//核心作用是：将 JSON 格式的数据通过 UDP 协议安全地发送到预设的网络目标。
void Plotter::plot(const nlohmann::json & json)  /*json.dump()：nlohmann::json库提供的方法，
                                                将 JSON 对象序列化为 JSON 格式的字符串（例如{"name":"value"}）*/
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto data = json.dump();/*它的作用与 json::parse 相反：parse 是将 JSON 字符串解析为可操作的 JSON 对象，
                          而 dump 是将内存中的 JSON 对象转换为字符串（便于存储到文件、网络传输或打印输出）。*/
  ::sendto(
    socket_, data.c_str(), data.length(), 0, reinterpret_cast<sockaddr *>(&destination_),
    sizeof(destination_));
}

}  // namespace tools