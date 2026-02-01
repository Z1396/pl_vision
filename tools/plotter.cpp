#include "plotter.hpp"

#include <arpa/inet.h>   // htons, inet_addr 网络字节序转换、IP 地址字符串转数值
#include <sys/socket.h>  // socket, sendto   创建套接字、UDP 数据报发送函数
#include <unistd.h>      // close            关闭文件描述符（socket 句柄本质是文件描述符）

namespace tools
{
Plotter::Plotter(std::string host, uint16_t port)
{
  /*::socket：加全局作用域符::，表示调用系统底层的 socket 函数（而非类内 / 命名空间内的同名函数），避免作用域冲突，是网络编程的规范写法；
  参数 1：AF_INET：地址族，固定为 IPv4 协议（与sockaddr_in匹配）；
  参数 2：SOCK_DGRAM：套接字类型，指定为UDP 协议（无连接、面向数据报、不可靠），区别于 TCP 的SOCK_STREAM（面向连接、可靠）；
  参数 3：0：协议编号，UDP 协议仅有一种实现，传 0 由系统自动匹配；
  返回值：成功返回非负的 socket 句柄（文件描述符），失败返回-1（代码中未做错误检查，实际生产环境需补充）；
  赋值给socket_：将创建的 UDP 套接字句柄保存为类成员，供后续sendto、析构函数使用。*/
  socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);

  // 1. 指定地址族为IPv4，与socket创建的AF_INET一致
  destination_.sin_family = AF_INET;
  // 2. 端口号转换为网络字节序，赋值给sin_port
  destination_.sin_port = ::htons(port);
  // 3. IP地址字符串转换为32位网络字节序数值，赋值给sin_addr.s_addr
  destination_.sin_addr.s_addr = ::inet_addr(host.c_str());

  /*关键函数解析（均加::调用系统函数）：
  htons(uint16_t port)：Host to Network Short的缩写，将主机字节序的 16 位端口号转换为网络字节序（大端序）。网络通信中所有数据必须使用网络字节序，否则不同字节序的主机间会出现数据错乱；
  inet_addr(const char* ip)：将点分十进制的 IP 字符串（如"192.168.1.100"、"127.0.0.1"）转换为32 位无符号整数的网络字节序 IP 地址，直接赋值给sockaddr_in的嵌套成员sin_addr.s_addr；
  host.c_str()：将 C++ 的std::string类型 IP 转换为 C 风格字符串，匹配inet_addr的参数要求。*/

}

/*核心作用：释放 UDP 套接字资源，遵循RAII 设计原则（资源在构造函数申请，析构函数自动释放），避免手动调用关闭函数导致的资源泄漏；
::close：加全局作用域符，调用系统底层的close函数，关闭 socket 句柄（socket 本质是 Linux/Unix 的文件描述符，与普通文件、管道的关闭方式一致）；
自动执行：当Plotter对象生命周期结束（如超出作用域、被 delete）时，编译器自动调用析构函数，无需手动执行，保证资源必被释放。*/
Plotter::~Plotter() { ::close(socket_); }


/*核心作用
接收nlohmann::json类型的 JSON 对象，将其序列化为标准 JSON 格式字符串，通过已初始化的 UDP 套接字，线程安全地发送到构造函数中配置的目标服务端（IP + 端口）。
整体执行流程
plaintext

接收JSON对象 → 加互斥锁保证线程安全 → JSON对象序列化为字符串 → 调用系统sendto发送UDP数据报 → 自动解锁*/
void Plotter::plot(const nlohmann::json & json)  /*json.dump()：nlohmann::json库提供的方法，
                                                将 JSON 对象序列化为 JSON 格式的字符串（例如{"name":"value"}）*/
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto data = json.dump();/*它的作用与 json::parse 相反：parse 是将 JSON 字符串解析为可操作的 JSON 对象，
                          而 dump 是将内存中的 JSON 对象转换为字符串（便于存储到文件、网络传输或打印输出）。*/

  /*sendto函数说明：
  UDP 是无连接协议，因此无需像 TCP 那样调用connect建立连接，
  每次发送数据时通过sendto直接指定目标服务端的地址和端口，这是 UDP 与 TCP 的核心区别之一。*/
  ::sendto(
    socket_, data.c_str(), data.length(), 0, reinterpret_cast<sockaddr *>(&destination_),sizeof(destination_));
  /*5	reinterpret_cast<sockaddr *>(&destination_)	struct sockaddr*	目标服务端的通用套接字地址指针（核心转换细节）：
  ・原始destination_是struct sockaddr_in类型（IPv4 专用地址结构，已配置 IP + 端口）；
  • sendto要求统一传入通用地址结构struct sockaddr*（保证 API 对 IPv4/IPv6 的通用性）；
  ・使用reinterpret_cast强制转换：因两个结构无继承关系，仅内存布局兼容，该转换是 C++ 中底层内存地址的强制转换，是该场景的唯一合法转换方式（static_cast会编译报错）
  
  6	sizeof(destination_)	socklen_t	目标地址结构的字节大小：
  ・告诉系统地址结构的长度，使其能正确解析struct sockaddr*中的 IPv4 地址信息；
  ・因destination_是sockaddr_in，该值固定为sizeof(sockaddr_in)（通常 16 字节）*/
}

}  // namespace tools