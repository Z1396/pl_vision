#ifndef IO__SOCKETCAN_HPP
#define IO__SOCKETCAN_HPP

#include <linux/can.h>
#include <net/if.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <thread>

#include "tools/logger.hpp"

using namespace std::chrono_literals;

constexpr int MAX_EVENTS = 10;

namespace io
{
class SocketCAN
{
public:
/**
 * @brief SocketCAN 类的构造函数
 * @param interface CAN 接口名称（如 "can0"）
 * @param rx_handler 接收 CAN 帧时的回调处理函数
 */
SocketCAN(const std::string & interface, std::function<void(const can_frame & frame)> rx_handler)
  : interface_(interface),        // 初始化CAN接口名称成员变量
    socket_fd_(-1),               // 初始化socket文件描述符为-1（表示未打开）
    epoll_fd_(-1),                // 初始化epoll文件描述符为-1（表示未初始化）
    rx_handler_(rx_handler),      // 初始化接收回调处理函数
    quit_(false),                 // 初始化退出标志为false
    ok_(false)                    // 初始化状态标志为false（表示未就绪）
{
    // 尝试打开并初始化CAN socket
    try_open();

    // 创建守护线程，用于监控和恢复CAN连接
    //this 指向当前类对象本身，而不是线程对象。线程 daemon_thread_ 只是这个类的一个成员，通过 this 可以访问该类的所有其他成员。
    daemon_thread_ = std::thread{[this]   
    {
        // 循环运行，直到收到退出信号
        while (!quit_) 
        {
            // 每100毫秒检查一次状态
            std::this_thread::sleep_for(100ms);

            // 如果当前状态正常，则继续等待下一次检查
            if (ok_) continue;

            // 如果读线程正在运行，则等待其结束
            if (read_thread_.joinable()) 
                read_thread_.join();

            // 关闭当前的socket资源
            close();
            
            // 尝试重新打开并初始化CAN连接
            try_open();
        }
    }};
}

  ~SocketCAN()
  {
    quit_ = true;
    if (daemon_thread_.joinable()) daemon_thread_.join();
    if (read_thread_.joinable()) read_thread_.join();
    close();
    tools::logger()->info("SocketCAN destructed.");
  }

  void write(can_frame * frame) const
  {
    if (::write(socket_fd_, frame, sizeof(can_frame)) == -1)
      throw std::runtime_error("Unable to write!");
  }

private:
  std::string interface_;
  int socket_fd_;
  int epoll_fd_;
  bool quit_;
  bool ok_;
  std::thread read_thread_;
  std::thread daemon_thread_;
  can_frame frame_;
  epoll_event events_[MAX_EVENTS];
  std::function<void(const can_frame & frame)> rx_handler_;

void open()
{
    // 1. 创建CAN原始套接字
    // PF_CAN: CAN协议族，SOCK_RAW: 原始套接字，CAN_RAW: CAN原始模式
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) 
        throw std::runtime_error("Error opening socket!");  // 创建失败时抛出异常

    // 2. 获取CAN接口的索引（如"can0"对应的系统内部编号）
    ifreq ifr;  // 用于存储接口信息的结构体

    // 将接口名称复制到ifr结构体（如"can0"），确保不超过最大长度
    std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);

    // 通过ioctl获取接口索引，SIOCGIFINDEX是获取接口索引的控制命令
    /*参数 1：socket_fd_：关联的 CAN 套接字描述符（告诉内核操作哪个套接字对应的设备）。
    参数 2：SIOCGIFINDEX：Socket I/O Control Get Interface Index（获取接口索引的控制命令），是内核定义的宏（值为 0x8933）。
    参数 3：&ifr：输入输出参数 —— 输入时ifr.ifr_name是接口名，输出时ifr.ifr_ifindex被内核填充为接口索引（如 can0 的索引可能是 3）。*/
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0)
        throw std::runtime_error("Error getting interface index!");

    // 3. 将套接字绑定到指定的CAN接口
    sockaddr_can addr;  // CAN地址结构体

    /*memset()内存初始化函数，将addr结构体的所有字节设为 0，避免未初始化的垃圾值影响后续操作
    （如can_family若未清零，可能被误设为其他地址族）。*/
    std::memset(&addr, 0, sizeof(sockaddr_can));  

    /*CAN 总线专用的地址结构体（类似 TCP/IP 的sockaddr_in），用于存储 CAN 接口的地址信息：
    can_family：地址族，必须设为AF_CAN（与PF_CAN等价，多数系统中AF_CAN == PF_CAN）。
    can_ifindex：CAN 接口索引（即步骤 2 中获取的ifr.ifr_ifindex），是绑定的核心标识。*/
    addr.can_family = AF_CAN;  
    addr.can_ifindex = ifr.ifr_ifindex;  // 绑定到前面获取的接口索引

    /*bind()
    套接字绑定系统调用，将套接字与指定地址（此处为 CAN 接口）关联。
    参数 1：socket_fd_：待绑定的 CAN 套接字描述符。
    参数 2：(sockaddr *)&addr：强制转换为通用地址结构体sockaddr（bind是通用函数，需适配不同协议族的地址结构体）。
    参数 3：sizeof(sockaddr_can)：sockaddr_can结构体的长度（告诉内核地址信息的字节数）。
    错误处理：若绑定失败（如接口已被占用、权限不足），需先调用::close(socket_fd_)释放已创建的套接字（避免资源泄漏），再抛出异常。*/
    if (bind(socket_fd_, (sockaddr *)&addr, sizeof(sockaddr_can)) < 0) 
    {
        ::close(socket_fd_);  // 绑定失败时关闭套接字
        throw std::runtime_error("Error binding socket to interface!");
    }

    // 4. 初始化epoll（I/O多路复用机制）用于监听套接字事件
    /*epoll_create1()
    创建一个 epoll 实例（内核维护的事件监控表），返回值epoll_fd_是 “epoll 描述符”，后续通过它操作 epoll 实例。
    参数：0：表示默认行为（若设为EPOLL_CLOEXEC，则进程 fork 后子进程自动关闭该 epoll 描述符，避免资源泄漏）。
    错误处理：返回 - 1 表示创建失败（如系统资源不足），抛出异常。*/
    epoll_fd_ = epoll_create1(0);

    if (epoll_fd_ == -1) 
        throw std::runtime_error("Error creating epoll file descriptor!");

    /*结构体：epoll_event
    用于描述 epoll 监控的 “事件类型” 和 “关联资源”：
    events：监控的事件类型，此处EPOLLIN表示 “有数据可读”（即 CAN 总线收到数据时触发该事件）。
    data：联合体成员，此处用fd存储关联的文件描述符（socket_fd_），方便事件触发时识别是哪个套接字有数据。*/
    epoll_event ev;
    ev.events = EPOLLIN;  // 关注"有数据可读"事件
    ev.data.fd = socket_fd_;  // 关联CAN套接字的文件描述符

    /*epoll_ctl()
    epoll 实例的控制函数，用于添加 / 修改 / 删除监控的文件描述符及事件。
    参数 1：epoll_fd_：待操作的 epoll 实例描述符。
    参数 2：EPOLL_CTL_ADD：操作类型 ——“添加” 一个文件描述符到 epoll 监控表。
    参数 3：ev.data.fd：待监控的文件描述符（此处为 CAN 套接字socket_fd_）。
    参数 4：&ev：epoll_event结构体，指定监控的事件类型和关联数据。
    错误处理：返回非 0 表示添加失败（如文件描述符无效），抛出异常。*/
    if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, ev.data.fd, &ev))
        throw std::runtime_error("Error adding socket to epoll file descriptor!");

    /*创建一个独立的接收线程，异步监听 CAN 总线数据（避免阻塞主线程），核心是循环调用read()函数读取数据，并处理异常。*/
    read_thread_ = std::thread([this]() 
    {
        ok_ = true;  // 标记CAN接口状态为正常
        while (!quit_) 
        {  // 循环直到收到退出信号
            std::this_thread::sleep_for(10us);  // 短暂休眠，降低CPU占用

            try 
            {
                read();  // 调用read()函数读取CAN消息
            } catch (const std::exception & e) 
            {
                // 读取失败时记录警告，标记状态异常并退出循环
                tools::logger()->warn("SocketCAN::read() failed: {}", e.what());
                ok_ = false;
                break;
            }
        }
    });

    // 6. 记录日志，提示CAN接口已成功打开
    tools::logger()->info("SocketCAN opened.");
}


// 尝试打开SocketCAN连接
void try_open()
{
    try  // 调用实际的打开函数
    {
        open();
    } 
    // 捕获所有标准异常
    catch (const std::exception & e) 
    {
        // 记录警告日志，包含异常信息
        tools::logger()->warn("SocketCAN::open() failed: {}", e.what());
    }
}

// 读取SocketCAN数据帧
void read()
{
    /* epoll_wait()
    用于等待 epoll 实例中监控的文件描述符发生事件（此处是 CAN 套接字的 “可读事件”），是 epoll 机制的核心函数。
    参数详解：
    epoll_fd_：epoll 实例的描述符（在open()函数中由epoll_create1()创建）。
    events_：指向epoll_event结构体数组的指针，用于存储发生的事件信息（输出参数）。
    MAX_EVENTS：events_数组的最大长度（即一次最多处理的事件数，需提前定义，如#define MAX_EVENTS 10）。
    2：超时时间（单位：毫秒）。若设为-1表示永久阻塞，0表示立即返回，此处2表示最多等待 2 毫秒后返回（避免长时间阻塞线程）。
    返回值：num_events表示实际发生的事件数（≥0）；若返回-1，表示发生错误（如epoll_fd_无效）。*/
    int num_events = epoll_wait(epoll_fd_, events_, MAX_EVENTS, 2);
    
    // 如果epoll_wait返回-1，表示发生错误，抛出运行时异常
    if (num_events == -1) throw std::runtime_error("Error waiting for events!");

    // 遍历所有发生的事件
    for (int i = 0; i < num_events; i++) 
    {
        /*函数：recv()
        用于从套接字接收数据，此处从 CAN 套接字读取原始 CAN 帧。
        参数详解：
        socket_fd_：CAN 套接字描述符（在open()函数中创建）。
        &frame_：指向can_frame结构体的指针，用于存储接收的 CAN 帧数据（输出参数）。
        can_frame是 CAN 帧的标准结构体（定义于<linux/can.h>），包含帧 ID、数据长度、数据域等：
        sizeof(can_frame)：接收缓冲区的大小（即can_frame结构体的字节数，确保能容纳一帧完整数据）。
        MSG_DONTWAIT：接收标志，指定非阻塞模式。若套接字中无数据，recv()会立即返回-1，而非阻塞等待（避免线程卡在此处）。
        返回值：num_bytes表示实际接收的字节数（成功时应为sizeof(can_frame)，即 CAN 帧的完整大小）；若返回-1，表示接收失败（如无数据、套接字关闭）。*/
        ssize_t num_bytes = recv(socket_fd_, &frame_, sizeof(can_frame), MSG_DONTWAIT);
        
        // 如果recv返回-1，表示读取失败，抛出运行时异常
        if (num_bytes == -1) throw std::runtime_error("Error reading from SocketCAN!");

        // 调用接收处理函数，将接收到的帧传递给回调函数
        rx_handler_(frame_);
    }
}

// 关闭SocketCAN连接及相关资源
void close()
{
    // 如果套接字描述符为-1，表示未打开，直接返回
    if (socket_fd_ == -1) return;
    
    // 从epoll中移除该套接字的事件监听
    epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, socket_fd_, NULL);
    
    // 关闭epoll文件描述符
    ::close(epoll_fd_);
    
    // 关闭套接字描述符
    ::close(socket_fd_);
}
};

}  // namespace io

#endif  // IO__SOCKETCAN_HPP