// 头文件保护宏：防止多次包含导致的类/函数/宏重复定义编译错误
// 命名规则：模块名__文件名_HPP（大写，双层下划线分隔，区分命名空间与文件名）
#ifndef IO__SOCKETCAN_HPP
#define IO__SOCKETCAN_HPP

// 引入Linux SocketCAN核心头文件：定义CAN帧结构体、协议族、原始模式等核心常量
#include <linux/can.h>
// 新增：CAN RAW 套接字选项头文件（SOL_CAN_RAW、CAN_RAW_FD_FRAMES 定义在此）
#include <linux/can/raw.h>
// 引入网络接口头文件：定义ifreq结构体，用于获取CAN网络接口索引
#include <net/if.h>
// 引入epoll头文件：Linux IO多路复用核心头文件，实现高效的套接字事件监听
#include <sys/epoll.h>
// 引入ioctl头文件：设备控制接口，用于获取CAN接口的系统索引
#include <sys/ioctl.h>
// 引入unix系统调用头文件：定义close/write/recv等底层系统调用
#include <unistd.h>

// 引入C++标准库头文件
#include <chrono>   // 高精度时间库：用于线程休眠，控制守护线程检查频率
#include <cstring>  // 字符串/内存操作库：memset/strncpy等，处理C风格字符串和内存初始化
#include <functional> // 函数包装器：存储CAN帧接收回调函数，实现业务解耦
#include <stdexcept>  // 异常处理库：抛出runtime_error异常，处理底层系统调用错误
#include <thread>     // 多线程库：创建接收线程和守护线程，实现异步通信与故障恢复

// 引入项目工具模块日志类：实现日志打印（INFO/WARN/ERROR），便于问题排查和调试
#include "tools/logger.hpp"

// 引入C++14时间字面量：支持100ms、10us等直观的时间表示，替代std::chrono::milliseconds(100)
using namespace std::chrono_literals;

// 编译期常量：epoll一次最多处理的事件数，限制事件数组大小，避免内存浪费
constexpr int MAX_EVENTS = 10;
// 修复：重命名宏，避免与系统CANFD_MAX_DLEN冲突
constexpr size_t SOCKETCAN_FD_MAX_DLEN = 64;
// 修复：基于重命名的常量定义帧大小
constexpr size_t SOCKETCAN_FD_FRAME_SIZE = sizeof(canfd_frame);

// IO模块命名空间：隔离CAN总线通信相关类/函数，避免与项目其他模块命名冲突
namespace io
{
/**
 * @brief Linux SocketCAN总线通信封装类（支持CAN 2.0 / CAN FD）
 * @details 基于Linux原生SocketCAN实现CAN总线的**异步收发、故障自动恢复、多线程安全**通信，核心特性：
 *          1. 封装SocketCAN底层系统调用（socket/bind/epoll/recv/write），屏蔽底层细节；
 *          2. 采用**IO多路复用（epoll）** + 独立接收线程，实现高效非阻塞数据接收，不阻塞主线程；
 *          3. 设计**守护线程**，实时监控CAN连接状态，异常时自动尝试重连，提升鲁棒性；
 *          4. 采用**回调机制**处理接收数据，与上层业务解耦，支持自定义数据处理逻辑；
 *          5. 完善的异常处理和资源管理，析构时自动释放所有资源，避免内存/文件描述符泄漏。
 * @note 仅支持Linux系统（SocketCAN是Linux内核原生CAN总线实现，无跨平台性）
 * @note 需系统开启CAN总线支持，且已配置CAN接口（如can0，设置波特率等）
 * @note 编译时无需额外库，链接时需保证内核版本≥2.6.25（SocketCAN最低支持版本），CAN FD需内核≥3.6
 */
class SocketCAN
{
public:

    using CANFdRxHandler = std::function<void(const canfd_frame & frame)>;
    using CAN20RxHandler = std::function<void(const can_frame & frame)>;
    /**
     * @brief 构造函数：初始化SocketCAN通信（兼容CAN 2.0 / CAN FD），创建接收线程和守护线程
     * @param interface CAN总线接口名称（如"can0"、"can1"），需提前在系统中配置
     * @param rx_handler CAN帧接收回调函数（兼容CAN 2.0帧），接收成功后自动调用
     * @param enable_canfd 是否启用CAN FD模式（默认false，兼容原有CAN 2.0代码）
     * @note 初始化流程：初始化成员变量 → 尝试打开并初始化CAN连接 → 创建守护线程 → 守护线程中启动接收线程
     * @note 回调函数运行在**接收线程**中，需保证回调函数执行效率，避免耗时操作（如大量IO、睡眠）
     * @note 守护线程为后台线程，析构时会自动join，避免程序退出时的资源泄漏
     */
    // 修复：恢复CAN 2.0回调参数（can_frame），保证与CBoard的兼容性
    SocketCAN(const std::string & interface, CAN20RxHandler rx_handler,
              bool enable_canfd = true)
        : interface_(interface),        // 初始化CAN接口名称
          socket_fd_(-1),               // 初始化CAN套接字文件描述符为-1（-1表示未打开/无效）
          epoll_fd_(-1),                // 初始化epoll实例描述符为-1（-1表示未初始化）
          quit_(false),                 // 初始化退出标志为false（线程正常运行）
          ok_(false),                   // 初始化连接状态标志为false（未就绪，需尝试连接）
          enable_canfd_(enable_canfd),   // 初始化CAN FD模式开关
          can20_rx_handler_(std::move(rx_handler)),
          canfd_rx_handler_(nullptr)
    {
        // 首次尝试打开并初始化CAN连接（失败仅打印日志，不终止程序，由守护线程后续重连）
        try_open();
        start_daemon_thread();
    }

    SocketCAN(const std::string & interface, CANFdRxHandler rx_handler,
            bool enable_canfd = true)
    : interface_(interface),        // 初始化CAN接口名称
        socket_fd_(-1),               // 初始化CAN套接字文件描述符为-1（-1表示未打开/无效）
        epoll_fd_(-1),                // 初始化epoll实例描述符为-1（-1表示未初始化）
        quit_(false),                 // 初始化退出标志为false（线程正常运行）
        ok_(false),                   // 初始化连接状态标志为false（未就绪，需尝试连接）
        enable_canfd_(enable_canfd),   // 初始化CAN FD模式开关
        can20_rx_handler_(nullptr),
        canfd_rx_handler_(std::move(rx_handler))
    {
        // 首次尝试打开并初始化CAN连接（失败仅打印日志，不终止程序，由守护线程后续重连）
        try_open();
        start_daemon_thread();
    }

    /**
     * @brief 析构函数：释放所有资源，安全退出所有线程
     * @details 析构流程：设置退出标志 → 等待守护线程和接收线程结束 → 关闭CAN连接 → 打印析构日志
     * @note 严格保证**线程安全退出**，避免析构时线程仍在访问类成员导致的未定义行为
     * @note 显式调用close()释放文件描述符，避免系统资源泄漏
     */
    ~SocketCAN()
    {
        // 设置全局退出标志，通知所有线程退出循环
        quit_ = true;
        // 等待守护线程结束（若线程可连接，即处于运行状态）
        if (daemon_thread_.joinable()) daemon_thread_.join();
        // 等待接收线程结束（若线程可连接）
        if (read_thread_.joinable()) read_thread_.join();
        // 关闭CAN连接，释放套接字、epoll等文件描述符
        close();
        // 打印析构日志，便于调试和资源监控
        tools::logger()->info("SocketCAN destructed.");
    }

    /**
     * @brief 发送CAN 2.0帧到CAN总线（兼容原有接口）
     * @param frame 指向待发送CAN 2.0帧的指针
     * @note const成员函数：仅发送数据，不修改类的任何成员变量，保证const正确性
     * @note 底层调用Linux原生write()系统调用，直接将CAN帧写入套接字
     * @note 发送失败时抛出std::runtime_error异常，由上层业务捕获处理
     * @warning 调用前需确保ok_=true（连接正常），否则会抛出异常
     */
    void write(can_frame * frame) const
    {
        if (!ok_) 
        {
            throw std::runtime_error("SocketCAN not connected!");
        }

        // 若启用CAN FD模式，自动转为CAN FD帧发送
        if (enable_canfd_) 
        {
            canfd_frame fd_frame{};
            fd_frame.can_id = frame->can_id;
            fd_frame.len = frame->can_dlc;
            fd_frame.flags = 0; // 标记为CAN 2.0帧
            std::memcpy(fd_frame.data, frame->data, frame->can_dlc);
            
            if (::write(socket_fd_, &fd_frame, SOCKETCAN_FD_FRAME_SIZE) == -1) 
            {
                throw std::runtime_error("Unable to write CAN 2.0 frame via CAN FD!");
            }
        } 
        else 
        {
            // 原生CAN 2.0发送
            if (::write(socket_fd_, frame, sizeof(can_frame)) == -1) 
            {
                throw std::runtime_error("Unable to write CAN 2.0 frame!");
            }
        }
    }

    /**
     * @brief 发送CAN FD帧到CAN总线（扩展接口）
     * @param frame 指向待发送CAN FD帧的指针
     * @note 仅在enable_canfd=true时可用
     */
    void write(canfd_frame * frame) const
    {
        if (!ok_) 
        {
            throw std::runtime_error("SocketCAN not connected!");
        }
        if (!enable_canfd_) 
        {
            throw std::runtime_error("CAN FD mode not enabled!");
        }
        // 校验数据长度
        if (frame->len > SOCKETCAN_FD_MAX_DLEN) 
        {
            throw std::runtime_error("Invalid CAN FD frame data length!");
        }
        if (::write(socket_fd_, frame, SOCKETCAN_FD_FRAME_SIZE) == -1) 
        {
            throw std::runtime_error("Unable to write CAN FD frame!");
        }
    }

private:
    // 私有成员变量：封装所有底层资源和状态，禁止上层业务直接访问，保证封装性
    std::string interface_;                  // CAN总线接口名称（如"can0"）
    int socket_fd_;                          // CAN套接字文件描述符：标识打开的CAN套接字，-1表示无效
    int epoll_fd_;                           // epoll实例文件描述符：标识epoll事件监控实例，-1表示无效
    bool quit_;                              // 全局退出标志：通知所有线程退出循环，实现优雅退出
    bool ok_;                                // CAN连接状态标志：true=连接正常/就绪，false=连接异常/未就绪
    bool enable_canfd_;                      // CAN FD模式开关：true=启用，false=禁用（默认）
    std::thread read_thread_;                // 接收线程：独立线程处理CAN帧接收，避免阻塞主线程
    std::thread daemon_thread_;              // 守护线程：监控连接状态，异常时自动重连
    canfd_frame fd_frame_;                   // CAN FD帧缓冲区（兼容CAN 2.0）
    epoll_event events_[MAX_EVENTS];         // epoll事件数组：存储epoll_wait检测到的就绪事件
    CAN20RxHandler can20_rx_handler_;  // 2.0帧回调（兼容）
    CANFdRxHandler canfd_rx_handler_;  // FD帧回调（完整数据）


    // 提取守护线程逻辑（简化构造函数）
    void start_daemon_thread() 
    {
        daemon_thread_ = std::thread{[this]
        {
            while (!quit_)
            {
                std::this_thread::sleep_for(100ms);
                if (ok_) continue;

                if (read_thread_.joinable())
                    read_thread_.join();

                close();
                try_open();
            }
        }};
    }
    /**
     * @brief 实际打开并初始化CAN总线连接的核心方法
     * @details 完成SocketCAN全流程初始化：创建原始套接字 → 获取接口索引 → 绑定套接字 → 初始化epoll → 创建接收线程
     * @note 该方法会抛出std::runtime_error异常，所有系统调用失败均会抛出，由try_open()捕获
     * @note 初始化成功后，ok_会被设为true，标记连接就绪
     */
    void open()
    {
        // 1. 创建CAN原始套接字
        socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_fd_ < 0)
            throw std::runtime_error("Error opening socket!");

        // 2. 启用CAN FD模式（若配置）
        if (enable_canfd_) 
        {
            int enable = 1;
            // SOL_CAN_RAW和CAN_RAW_FD_FRAMES已在<linux/can/raw.h>中定义
            if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 
                           &enable, sizeof(enable)) < 0) 
            {
                ::close(socket_fd_);
                throw std::runtime_error("Failed to enable CAN FD mode!");
            }
            tools::logger()->info("CAN FD mode enabled for interface: {}", interface_);
        }

        // 3. 获取CAN接口的系统索引
        ifreq ifr;
        std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
        if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) 
        {
            ::close(socket_fd_);
            throw std::runtime_error("Error getting interface index!");
        }

        // 4. 绑定套接字到指定CAN接口
        sockaddr_can addr;
        std::memset(&addr, 0, sizeof(sockaddr_can));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(socket_fd_, (sockaddr *)&addr, sizeof(sockaddr_can)) < 0) 
        {
            ::close(socket_fd_);
            throw std::runtime_error("Error binding socket to interface!");
        }

        // 5. 初始化epoll实例
        epoll_fd_ = epoll_create1(0);
        if (epoll_fd_ == -1) 
        {
            ::close(socket_fd_);
            throw std::runtime_error("Error creating epoll file descriptor!");
        }

        // 6. 将CAN套接字添加到epoll监控
        epoll_event ev;
        ev.events = EPOLLIN;
        ev.data.fd = socket_fd_;
        if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, ev.data.fd, &ev)) 
        {
            ::close(socket_fd_);
            ::close(epoll_fd_);
            throw std::runtime_error("Error adding socket to epoll!");
        }

        // 7. 创建接收线程
        read_thread_ = std::thread([this]() 
        {
            ok_ = true;
            while (!quit_) 
            {
                std::this_thread::sleep_for(10us);
                try 
                {
                    read();
                } catch (const std::exception & e) 
                {
                    tools::logger()->warn("SocketCAN::read() failed: {}", e.what());
                    ok_ = false;
                    break;
                }
            }
        });

        tools::logger()->info("SocketCAN opened (interface: {}, CAN FD: {})", 
                              interface_, enable_canfd_ ? "enabled" : "disabled");
    }

    /**
     * @brief 安全尝试打开CAN连接的包装方法
     */
    void try_open()
    {
        try 
        {
            open();
        } catch (const std::exception & e) 
        {
            tools::logger()->warn("SocketCAN::open() failed: {}", e.what());
            ok_ = false;
        }
    }

    /**
     * @brief 重构read()方法：自适应触发FD/2.0回调
     * 核心逻辑：优先触发FD回调（传递完整数据），无FD回调则降级触发2.0回调
     */
    void read()
    {
        int num_events = epoll_wait(epoll_fd_, events_, MAX_EVENTS, 2);
        if (num_events == -1)
            throw std::runtime_error("Error waiting for events!");

        for (int i = 0; i < num_events; i++) 
        {
            ssize_t num_bytes = recv(socket_fd_, &fd_frame_, SOCKETCAN_FD_FRAME_SIZE, MSG_DONTWAIT);
            if (num_bytes == -1)
                throw std::runtime_error("Error reading from SocketCAN!");

            // 1. 优先触发FD帧回调（传递完整的CAN FD数据，包括超过8字节的部分）
            if (canfd_rx_handler_) 
            {
                canfd_rx_handler_(fd_frame_);
            }
            // 2. 无FD回调则降级触发2.0帧回调（兼容原有逻辑）
            else if (can20_rx_handler_) 
            {
                can_frame frame{};
                frame.can_id = fd_frame_.can_id;
                frame.can_dlc = std::min((int)fd_frame_.len, 8);
                std::memcpy(frame.data, fd_frame_.data, frame.can_dlc);
                can20_rx_handler_(frame);
            }
            // 3. 无任何回调：打印警告（避免数据丢失）
            else 
            {
                tools::logger()->warn("No CAN rx handler registered, frame dropped!");
            }
        }
    }

    /**
     * @brief 关闭CAN连接并释放所有相关系统资源
     */
    void close()
    {
        if (socket_fd_ == -1)
            return;

        epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, socket_fd_, NULL);
        ::close(epoll_fd_);
        ::close(socket_fd_);
        socket_fd_ = -1;
        epoll_fd_ = -1;
        ok_ = false;
    }
};

}  // namespace io

#endif  // IO__SOCKETCAN_HPP
