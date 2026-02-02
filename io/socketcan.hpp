// 头文件保护宏：防止多次包含导致的类/函数/宏重复定义编译错误
// 命名规则：模块名__文件名_HPP（大写，双层下划线分隔，区分命名空间与文件名）
#ifndef IO__SOCKETCAN_HPP
#define IO__SOCKETCAN_HPP

// 引入Linux SocketCAN核心头文件：定义CAN帧结构体、协议族、原始模式等核心常量
#include <linux/can.h>
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

// IO模块命名空间：隔离CAN总线通信相关类/函数，避免与项目其他模块命名冲突
namespace io
{
/**
 * @brief Linux SocketCAN总线通信封装类
 * @details 基于Linux原生SocketCAN实现CAN总线的**异步收发、故障自动恢复、多线程安全**通信，核心特性：
 *          1. 封装SocketCAN底层系统调用（socket/bind/epoll/recv/write），屏蔽底层细节；
 *          2. 采用**IO多路复用（epoll）** + 独立接收线程，实现高效非阻塞数据接收，不阻塞主线程；
 *          3. 设计**守护线程**，实时监控CAN连接状态，异常时自动尝试重连，提升鲁棒性；
 *          4. 采用**回调机制**处理接收数据，与上层业务解耦，支持自定义数据处理逻辑；
 *          5. 完善的异常处理和资源管理，析构时自动释放所有资源，避免内存/文件描述符泄漏。
 * @note 仅支持Linux系统（SocketCAN是Linux内核原生CAN总线实现，无跨平台性）
 * @note 需系统开启CAN总线支持，且已配置CAN接口（如can0，设置波特率等）
 * @note 编译时无需额外库，链接时需保证内核版本≥2.6.25（SocketCAN最低支持版本）
 */
class SocketCAN
{
public:
    /**
     * @brief 构造函数：初始化SocketCAN通信，创建接收线程和守护线程
     * @param interface CAN总线接口名称（如"can0"、"can1"），需提前在系统中配置
     * @param rx_handler CAN帧接收回调函数，接收成功后自动调用，传递接收到的原始CAN帧
     * @note 初始化流程：初始化成员变量 → 尝试打开并初始化CAN连接 → 创建守护线程 → 守护线程中启动接收线程
     * @note 回调函数运行在**接收线程**中，需保证回调函数执行效率，避免耗时操作（如大量IO、睡眠）
     * @note 守护线程为后台线程，析构时会自动join，避免程序退出时的资源泄漏
     */
    SocketCAN(const std::string & interface, std::function<void(const can_frame & frame)> rx_handler)
        : interface_(interface),        // 初始化CAN接口名称
          socket_fd_(-1),               // 初始化CAN套接字文件描述符为-1（-1表示未打开/无效）
          epoll_fd_(-1),                // 初始化epoll实例描述符为-1（-1表示未初始化）
          rx_handler_(rx_handler),      // 初始化接收回调函数，保存上层业务的处理逻辑
          quit_(false),                 // 初始化退出标志为false（线程正常运行）
          ok_(false)                    // 初始化连接状态标志为false（未就绪，需尝试连接）
    {
        // 首次尝试打开并初始化CAN连接（失败仅打印日志，不终止程序，由守护线程后续重连）
        try_open();

        // 创建守护线程：负责监控CAN连接状态，异常时自动重连
        // 采用lambda表达式作为线程入口，捕获this指针以访问类成员变量和方法
        daemon_thread_ = std::thread{[this]
        {
            // 守护线程主循环：直到收到退出信号（quit_=true）才退出
            while (!quit_)
            {
                // 每100毫秒检查一次连接状态，降低CPU占用率
                std::this_thread::sleep_for(100ms);

                // 连接状态正常，跳过后续重连逻辑，继续下一次检查
                if (ok_) continue;

                // 连接异常：先等待接收线程结束（避免线程竞态和资源访问冲突）
                if (read_thread_.joinable())
                    read_thread_.join();

                // 关闭当前无效的CAN连接和相关资源
                close();

                // 尝试重新打开并初始化CAN连接
                try_open();
            }
        }};
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
     * @brief 发送CAN帧到CAN总线
     * @param frame 指向待发送CAN帧的指针（can_frame为Linux SocketCAN原生结构体）
     * @note const成员函数：仅发送数据，不修改类的任何成员变量，保证const正确性
     * @note 底层调用Linux原生write()系统调用，直接将CAN帧写入套接字
     * @note 发送失败时抛出std::runtime_error异常，由上层业务捕获处理
     * @warning 调用前需确保ok_=true（连接正常），否则会抛出异常
     */
    void write(can_frame * frame) const
    {
        // 调用系统write()函数发送CAN帧，返回值为实际写入的字节数
        // 若返回-1，表示发送失败（如连接断开、接口异常）
        if (::write(socket_fd_, frame, sizeof(can_frame)) == -1)
            throw std::runtime_error("Unable to write!");
    }

private:
    // 私有成员变量：封装所有底层资源和状态，禁止上层业务直接访问，保证封装性
    std::string interface_;                  // CAN总线接口名称（如"can0"）
    int socket_fd_;                          // CAN套接字文件描述符：标识打开的CAN套接字，-1表示无效
    int epoll_fd_;                           // epoll实例文件描述符：标识epoll事件监控实例，-1表示无效
    bool quit_;                              // 全局退出标志：通知所有线程退出循环，实现优雅退出
    bool ok_;                                // CAN连接状态标志：true=连接正常/就绪，false=连接异常/未就绪
    std::thread read_thread_;                // 接收线程：独立线程处理CAN帧接收，避免阻塞主线程
    std::thread daemon_thread_;              // 守护线程：监控连接状态，异常时自动重连
    can_frame frame_;                        // CAN帧缓冲区：存储接收到的原始CAN帧，复用缓冲区避免频繁内存分配
    epoll_event events_[MAX_EVENTS];         // epoll事件数组：存储epoll_wait检测到的就绪事件，最大容量MAX_EVENTS
    std::function<void(const can_frame & frame)> rx_handler_;  // 接收回调函数：上层业务自定义的CAN帧处理逻辑

    /**
     * @brief 实际打开并初始化CAN总线连接的核心方法
     * @details 完成SocketCAN全流程初始化：创建原始套接字 → 获取接口索引 → 绑定套接字 → 初始化epoll → 创建接收线程
     * @note 该方法会抛出std::runtime_error异常，所有系统调用失败均会抛出，由try_open()捕获
     * @note 初始化成功后，ok_会被设为true，标记连接就绪
     */
    void open()
    {
        // 1. 创建CAN原始套接字
        // PF_CAN：CAN协议族（Protocol Family CAN）
        // SOCK_RAW：原始套接字，直接访问底层CAN总线数据，不经过协议栈处理
        // CAN_RAW：CAN原始模式，是SocketCAN的核心工作模式
        socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_fd_ < 0)
            throw std::runtime_error("Error opening socket!");  // 创建失败，抛出异常

        // 2. 获取CAN接口的系统索引（如"can0"对应内核中的数字索引，如3）
        ifreq ifr;  // 网络接口请求结构体：用于与内核交互，获取/设置接口信息
        // 将CAN接口名称复制到ifr结构体，IFNAMSIZ-1保证不越界，自动补'\0'
        std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
        // 调用ioctl系统调用获取接口索引，SIOCGIFINDEX=获取接口索引命令
        // 入参：ifr.ifr_name=接口名；出参：ifr.ifr_ifindex=接口索引
        if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0)
            throw std::runtime_error("Error getting interface index!");

        // 3. 将CAN套接字绑定到指定的CAN接口（核心步骤，关联套接字与物理CAN接口）
        sockaddr_can addr;  // CAN专用地址结构体：替代TCP/IP的sockaddr_in
        std::memset(&addr, 0, sizeof(sockaddr_can));  // 初始化地址结构体，避免垃圾值
        addr.can_family = AF_CAN;  // CAN地址族，必须设为AF_CAN（与PF_CAN等价）
        addr.can_ifindex = ifr.ifr_ifindex;  // 绑定到获取到的CAN接口索引

        // 调用bind系统调用完成绑定，将套接字与CAN接口关联
        // 需将sockaddr_can强制转为通用sockaddr*，适配bind的通用接口
        if (bind(socket_fd_, (sockaddr *)&addr, sizeof(sockaddr_can)) < 0)
        {
            ::close(socket_fd_);  // 绑定失败，先关闭已创建的套接字，避免资源泄漏
            throw std::runtime_error("Error binding socket to interface!");
        }

        // 4. 初始化epoll实例，实现IO多路复用（高效监听套接字可读事件）
        // epoll_create1(0)：创建epoll实例，0表示默认属性，返回epoll文件描述符
        epoll_fd_ = epoll_create1(0);
        if (epoll_fd_ == -1)
            throw std::runtime_error("Error creating epoll file descriptor!");

        // 5. 将CAN套接字添加到epoll监控，关注"可读事件（EPOLLIN）"
        epoll_event ev;  // epoll事件结构体：描述监控的事件类型和关联资源
        ev.events = EPOLLIN;  // 监控套接字的"有数据可读"事件（CAN总线收到数据时触发）
        ev.data.fd = socket_fd_;  // 将CAN套接字描述符关联到该事件，便于后续识别

        // epoll_ctl：epoll控制函数，EPOLL_CTL_ADD=添加监控对象
        if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, ev.data.fd, &ev))
            throw std::runtime_error("Error adding socket to epoll file descriptor!");

        // 6. 创建独立的接收线程，异步处理CAN帧接收，避免阻塞主线程
        //[this] 是 lambda 的捕获列表，专门捕获当前类对象的 this 指针，让 lambda 内部能访问和调用类的非静态成员（变量 / 函数）
        read_thread_ = std::thread([this]()
        {
            ok_ = true;  // 标记CAN连接状态为正常/就绪
            // 接收线程主循环：直到收到退出信号
            while (!quit_)
            {
                std::this_thread::sleep_for(10us);  // 短暂休眠，降低CPU空转率

                try
                {
                    read();  // 调用read()方法，读取并处理CAN帧
                }
                catch (const std::exception & e)
                {
                    // 读取失败（如连接断开），打印警告日志，标记状态异常，退出接收线程
                    tools::logger()->warn("SocketCAN::read() failed: {}", e.what());
                    ok_ = false;
                    break;
                }
            }
        });

        // 7. 初始化成功，打印日志
        tools::logger()->info("SocketCAN opened.");
    }

    /**
     * @brief 安全尝试打开CAN连接的包装方法
     * @details 封装open()方法，捕获所有标准异常，避免因连接失败导致程序终止
     * @note 失败仅打印WARN日志，不抛出异常，由守护线程后续持续尝试重连
     * @note 成功则open()内部会将ok_设为true，标记连接就绪
     */
    void try_open()
    {
        try
        {
            open();  // 调用实际的初始化方法
        }
        catch (const std::exception & e)
        {
            // 捕获所有标准异常，打印异常信息，供调试和问题排查
            tools::logger()->warn("SocketCAN::open() failed: {}", e.what());
        }
    }

    /**
     * @brief 读取CAN总线数据并触发回调的核心方法
     * @details 基于epoll实现非阻塞式数据读取：检测套接字可读事件 → 读取CAN帧 → 调用上层回调函数
     * @note 运行在接收线程中，异常时会抛出std::runtime_error，由接收线程捕获
     * @note 采用非阻塞模式，避免线程因无数据而长时间阻塞
     */
    void read()
    {
        // 调用epoll_wait检测就绪事件，实现IO多路复用
        // 参数：epoll_fd_=epoll实例；events_=事件输出数组；MAX_EVENTS=最大处理事件数；2=超时时间（ms）
        // 返回值：>0=就绪事件数；0=超时；-1=失败
        int num_events = epoll_wait(epoll_fd_, events_, MAX_EVENTS, 2);
        if (num_events == -1)
            throw std::runtime_error("Error waiting for events!");

        // 遍历所有就绪事件，处理每一个可读的CAN帧
        for (int i = 0; i < num_events; i++)
        {
            // 调用recv系统调用读取CAN帧，非阻塞模式（MSG_DONTWAIT）
            // 参数：socket_fd_=CAN套接字；&frame_=数据缓冲区；sizeof(can_frame)=缓冲区大小；MSG_DONTWAIT=非阻塞
            // 返回值：>0=实际读取字节数（正常为sizeof(can_frame)）；-1=失败
            ssize_t num_bytes = recv(socket_fd_, &frame_, sizeof(can_frame), MSG_DONTWAIT);
            if (num_bytes == -1)
                throw std::runtime_error("Error reading from SocketCAN!");

            // 读取成功，调用上层注册的回调函数，传递接收到的CAN帧，实现业务解耦
            rx_handler_(frame_);
            /*六、回调的核心价值（为什么要使用回调，而非直接调用）
            你可能会问：“我直接调用自己的函数不就行了，为什么要绕一圈用回调？”—— 回调的核心价值是解耦，让调用者和被调用者（回调函数）之间无需知道彼此的具体实现，仅通过统一的接口交互，这在框架 / 库开发、异步编程、事件驱动中必不可少：

                框架 / 库开发：库开发者无法预知用户的具体业务逻辑（如按钮点击后要执行什么操作），因此提供回调接口，让用户自己实现逻辑并传入，库在合适时机执行；
                异步编程：异步任务（如网络请求、文件读写）完成后，需要执行后续逻辑，通过回调可以将后续逻辑传递给异步函数，任务完成后自动回调；
                事件驱动：GUI 界面（按钮、菜单）、网络通信、定时器等场景，需要在 “事件触发” 时执行特定逻辑，回调是实现事件驱动的核心方式；
                代码复用：调用者的逻辑（如事件检测、任务调度）可以复用，仅需替换不同的回调函数，即可实现不同的业务效果。

            七、核心总结（一句话记清回调的本质和流程）

                回调的本质：将可调用对象作为参数传递，由接收方在合适时机反向调用，核心是 “传递 + 反向执行”；
                能回调的原因：C++ 支持可调用对象（函数 / 绑定对象 /lambda）作为函数参数，传递后可在其他作用域主动调用；
                具体怎么回调：分 4 步 ——定义回调函数 → 定义注册函数（接收回调） → 注册回调（传递可调用对象） → 触发回调（接收方执行回调）；
                类成员函数回调：需先通过std::bind/lambda 适配（绑定 this），转为std::function可接收的可调用对象，再按普通流程回调；
                回调的价值：实现代码解耦，让调用者和被调用者无需耦合具体实现，是框架开发、异步编程、事件驱动的核心技术。

            简单记：回调就是 “你把函数给我，我帮你在需要的时候执行”，传递的是函数，执行的时机由接收方决定，这就是 “回调” 的全部逻辑。*/
        }
    }

    /**
     * @brief 关闭CAN连接并释放所有相关系统资源
     * @details 释放流程：从epoll移除套接字 → 关闭epoll实例 → 关闭CAN套接字
     * @note 对文件描述符做有效性检查（!= -1），避免重复关闭导致的未定义行为
     * @note 调用系统原生close()（加::作用域），避免与类成员函数close()重名
     */
    void close()
    {
        if (socket_fd_ == -1)  // 套接字未打开，直接返回
            return;

        // 从epoll实例中移除CAN套接字的监控，EPOLL_CTL_DEL=删除监控对象
        epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, socket_fd_, NULL);
        ::close(epoll_fd_);   // 关闭epoll实例文件描述符
        ::close(socket_fd_);  // 关闭CAN套接字文件描述符
        socket_fd_ = -1;      // 重置套接字描述符为无效，避免重复操作
        epoll_fd_ = -1;       // 重置epoll描述符为无效
    }
};

}  // namespace io

#endif  // IO__SOCKETCAN_HPP


/*核心答案是：该类并非模板类，却将所有成员函数（包括实现）直接写在.hpp头文件中，是工业级 C++ 开发中针对「仅在 Linux 平台编译、无跨模块编译需求的专用类」的标准优化设计，完全符合 C++ 编译链接规则，且能带来显著的工程价值。
与模板类「必须将实现写在头文件（编译期实例化要求）」的语法强制要求不同，这个SocketCAN类是普通类，其写法是工程设计选择，而非语法刚需，背后有多重核心原因和明确的适用场景，以下分维度详细解释：
一、先明确核心前提：模板类与普通类的头文件实现要求差异
这是理解该设计的基础，两者写在头文件的底层原因完全不同：
类类型	头文件写实现的原因	性质
模板类（如之前的ThreadSafeQueue）	C++ 模板的编译期实例化特性：编译器需在实例化时看到模板成员函数的完整定义，否则会报「未定义的引用」链接错误	语法强制要求，不这么写会编译失败
普通类（如SocketCAN）	工程设计优化选择，利用 C++「头文件内联编译」规则，兼顾编译效率、代码维护性	可选设计方案，也可拆分到.cpp，但特定场景下头文件实现更优
二、SocketCAN头文件内实现所有逻辑的核心原因
结合该类的使用场景（Linux 专属 SocketCAN 封装） 和工程属性（项目内部 IO 模块专用类，不对外提供库），头文件内实现是最优选择，核心原因有 4 点：
1. 利用内联编译优化，消除函数调用开销，提升通信实时性
C++ 中有一个核心规则：直接在类定义内实现的成员函数，编译器会自动将其视为inline（内联）函数（无需显式加inline关键字）。

    内联函数的编译特性：编译器会尝试将函数体直接嵌入到调用处，消除函数调用的底层开销（如栈帧创建、参数传递、返回地址跳转）；
    对SocketCAN的价值：该类是 CAN 总线通信的底层封装，实时性要求极高（CAN 帧收发延迟需控制在微秒级），消除函数调用开销能直接提升通信效率，避免因频繁调用read()/write()/close()带来的延迟；
    补充：若拆分到.cpp文件，函数为非内联，跨文件调用时必然产生调用开销，与 CAN 总线的实时性需求相悖。

2. 满足单模块编译需求，简化项目构建，避免链接错误
该类是项目内部 IO 模块的专用类，仅在项目内部被CBoard等类调用，不会被编译为独立的动态库 / 静态库对外提供，项目采用「单模块整体编译」模式：

    头文件内实现：所有编译单元（.cpp）包含SocketCAN.hpp后，能直接获取函数完整定义，编译时无需处理跨文件的链接依赖，简化项目 Makefile/CMake 配置，无需为该类单独配置编译规则；
    避免链接问题：若拆分到.cpp，需保证该.cpp被加入编译流程，否则会出现「未定义的引用」链接错误；而头文件实现从根源避免了该问题，尤其适合团队协作开发（减少新人的配置成本）。

3. 实现代码单文件聚合，大幅提升维护性和可移植性
SocketCAN的核心逻辑与 Linux 底层系统调用强绑定（socket/epoll/ioctl），功能内聚、无外部依赖（仅依赖系统头文件和项目日志类）：

    单文件维护：所有定义（类声明、成员变量、函数实现）都在SocketCAN.hpp中，后续修改（如调整 epoll 超时时间、修改重连逻辑）时，仅需编辑一个文件，无需在.hpp和.cpp之间来回切换，大幅提升维护效率；
    便捷移植：若项目需要在其他 Linux 设备上部署，仅需拷贝这一个头文件，无需同步拷贝对应的.cpp，降低移植成本。

4. 适配Linux 平台专属特性，无跨平台 / 多编译单元冲突
该类有明确的平台限制（仅支持 Linux），且无模板参数、无重载多版本，头文件内实现不会产生编译冲突：

    跨平台冲突风险：若类需跨平台（如同时支持 Linux/Windows），头文件实现可能因平台专属代码（如linux/can.h）导致编译失败，此时需拆分到.cpp做平台条件编译；但SocketCAN本身是 Linux 专属，无此问题；
    多编译单元重复定义风险：C++ 中inline函数允许在多个编译单元中存在相同定义（编译器会自动合并），而非内联函数若在多个编译单元中定义会报「多重定义」链接错误；而该类的函数因类内实现被自动标为 inline，完全避免了此问题。

三、该设计的适用场景边界：并非所有普通类都适合头文件内实现
SocketCAN的头文件实现是场景化最优解，但该方案有明确的适用边界，超出边界则需拆分到.cpp文件，核心适用条件：

    平台专属 / 功能内聚：类仅适用于单一平台（如 Linux），功能高度内聚，无跨平台兼容需求；
    项目内部专用：仅在项目内部使用，不编译为独立库对外提供，无跨模块链接需求；
    实时性要求高：作为底层封装类，需要消除函数调用开销，提升执行效率；
    代码量适中：类的实现代码量适中（该类约 300 行实现），不会因头文件过大导致编译时间显著增加。

反例（不适合头文件实现）：

    跨平台通用类（如字符串工具类）：头文件内实现会因平台专属代码导致编译冲突；
    对外提供的库类（如项目的公共工具库）：头文件内实现会暴露底层细节，且库编译时会产生冗余代码；
    大体积业务类（如上千行的算法类）：头文件过大会导致所有包含该头文件的编译单元编译时间大幅增加。

四、补充：若拆分到.cpp会怎样？（对比理解设计选择）
若将SocketCAN的成员函数实现拆分到SocketCAN.cpp，会出现以下问题，进一步印证头文件实现的合理性：

    失去内联优化，函数调用开销增加，降低 CAN 总线通信的实时性；
    项目构建复杂度提升：需在 CMake/Makefile 中添加SocketCAN.cpp的编译规则，否则会报链接错误；
    维护性下降：修改任何逻辑都需同时编辑.hpp和.cpp，易出现声明 / 实现不一致的问题；
    移植性下降：部署时需同时拷贝.hpp和.cpp，且需保证编译时该.cpp被正确包含。

唯一的优点是减少头文件体积，但该类代码量适中，此优点完全抵消不了上述问题，因此拆分到.cpp是次优选择。
核心总结

    本质差异：模板类头文件写实现是语法强制要求（编译期实例化），SocketCAN（普通类）是工程设计选择（优化为主）；
    核心原因：类内实现自动内联（消除调用开销、提升实时性）+ 单模块编译（简化构建、避免链接错误）+ 代码聚合（提升维护 / 移植性）+ Linux 专属（无跨平台冲突）；
    适用边界：仅适用于「平台专属、项目内部专用、实时性要求高、代码量适中」的普通类；
    设计合理性：该写法完全符合 C++ 编译链接规则，且针对SocketCAN的底层封装、Linux 专属、实时性需求等场景，是工业级 C++ 开发的最优设计选择。*/