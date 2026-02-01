#ifndef TOOLS__RECORDER_HPP
#define TOOLS__RECORDER_HPP

/*#include <Eigen/Geometry> 是Eigen 线性代数库中专门用于几何变换、空间姿态表示的核心头文件，提供了三维空间中工程开发（如机器人、计算机视觉、图形学、无人机控制等）
最常用的几何操作类与方法，是处理旋转、平移、刚体变换的标准工具，无需依赖其他 Eigen 模块（可独立引入核心几何功能）。*/
#include <Eigen/Geometry>
#include <chrono>

/*1. 核心作用
C++ 标准库文件输入输出流头文件，封装了文件的打开、读写、关闭等操作，替代 C 语言的fopen/fread/fwrite/fclose，
基于 C++ 流对象实现，语法更简洁、面向对象特性更友好，是 C++ 处理文件的首选方式。
2. 核心类（3 个基础类，均位于std命名空间）
类名	中文名称	核心作用	常用场景
std::ifstream	输入文件流	从磁盘文件读取数据到程序	读取配置文件、日志、数据文件
std::ofstream	输出文件流	将程序数据写入磁盘文件	写入日志、保存结果、持久化数据
std::fstream	输入输出文件流	同时支持文件的读和写	需实时读写的文件场景（如配置修改）
3. 核心特性
  面向对象设计：通过流对象操作文件，支持<<（写入）、>>（读取）等流运算符，与std::cout/std::cin语法一致，学习成本低；
  自动资源管理：流对象生命周期结束时，会自动关闭文件，无需手动调用关闭函数，避免文件句柄泄漏；*/
#include <fstream>
/*总结（核心要点）
    核心定位不同：<fstream> 是文件内容读写库，<filesystem> 是文件 / 目录系统管理库，二者功能互补，无替代关系；
    操作对象不同：<fstream> 操作文件内部的内容，<filesystem> 操作文件 / 目录本身（路径、结构、属性）；
    标准要求不同：<fstream> 支持 C++98+，无需额外配置；<filesystem> 必须 C++17+，编译时需指定标准；
    核心搭配逻辑：<filesystem> 做前置管理（创目录、拼路径、判存在），<fstream> 做后续读写（打开文件、写 / 读内容、关文件）；
    功能边界清晰：<fstream> 不能创目录，<filesystem> 不能写内容，切勿混淆；
    工程必用组合：在你的Recorder类、自瞄项目及所有现代 C++ 工程中，二者配合使用是完成文件操作的标准方案，兼顾易用性、跨平台性和工程规范性。*/
#include <opencv2/opencv.hpp>

/*C++11 标准新增的原生多线程编程头文件，提供了操作线程的核心类和函数，让 C++ 无需依赖平台专属 API
（如 Linux 的 pthread、Windows 的 CreateThread），实现跨平台的多线程开发，是 C++ 处理并发任务的标准工具。*/
#include <thread>

#include "tools/thread_safe_queue.hpp"
namespace tools
{
/**
 * @brief 多线程帧数据记录器类
 * @details 同步记录图像、空间姿态（四元数）、高精度时间戳，
 *          图像保存为视频文件，姿态+时间戳等信息保存为文本文件，
 *          采用生产者-消费者模型，避免IO操作阻塞业务线程，保证实时性
 */
class Recorder
{
public:
    /**
     * @brief 构造函数
     * @param fps 视频保存的帧率，默认30帧/秒，用于控制视频写入的时间间隔
     */
    Recorder(double fps = 30);

    /**
     * @brief 析构函数
     * @details 负责停止保存线程、释放文件资源、关闭视频/文本写入流，
     *          保证所有缓存数据都被刷入文件，避免资源泄漏和数据丢失
     */
    ~Recorder();

    /**
     * @brief 核心记录接口（生产者接口）
     * @details 业务线程调用，将图像、姿态、时间戳封装为帧数据，
     *          推入线程安全队列，无阻塞（或轻阻塞），立即返回
     * @param img 待记录的图像帧，cv::Mat类型（OpenCV图像），支持彩色/灰度图
     * @param q 待记录的空间姿态，Eigen双精度四元数，描述物体/相机的旋转姿态
     * @param timestamp 帧数据的高精度时间戳，稳态时钟（不受系统时间修改影响）
     */
    void record(
        const cv::Mat & img, 
        const Eigen::Quaterniond & q,
        const std::chrono::steady_clock::time_point & timestamp);

private:
    /**
     * @brief 帧数据结构体，封装单帧的所有关联数据
     * @details 作为生产者-消费者之间的数据传输单元，
     *          包含图像、姿态、时间戳，保证数据的同步性和完整性
     */
    struct FrameData
    {
        cv::Mat img;  ///< 单帧图像数据（OpenCV矩阵）
        Eigen::Quaterniond q;  ///< 该帧对应的空间旋转姿态（Eigen双精度四元数，w/x/y/z存储）
        std::chrono::steady_clock::time_point timestamp;  ///< 该帧的高精度时间戳（稳态时钟，避免系统时间干扰）
    };

    bool init_;  ///< 初始化标志位：标记视频/文本写入器是否已完成初始化（首次record时触发初始化）

    /*1. std::atomic<bool> stop_thread_：轻量无锁的原子标志
    核心定位：C++11 引入的原子类型，专门用于保证单个变量的读、写、修改操作是原子的（不可被中断），无需互斥锁即可实现线程安全。
    设计目标：解决简单标志位的多线程同步问题（如线程退出标志、运行状态标记），追求极致的性能（无锁开销，比互斥锁快一个数量级）。
    典型用途：你的项目中用于标记 “是否停止工作线程”（如相机采集线程、图像处理线程的退出标志），对应变量名 stop_thread_ 是行业通用命名。
    2. std::atomic<bool>：硬件级别的无锁原子操作
    原子类型的操作由CPU 硬件指令直接支持（如 x86 的lock前缀指令），无需操作系统的线程调度介入，操作过程不可被中断：
    当线程 A 执行 stop_thread_ = true; 时，CPU 会保证这个写操作一次性完成，线程 B 在执行 if (stop_thread_) { ... } 时，读到的要么是修改前的 false，要么是修改后的 true，不会读到 “中间态”（普通 bool 变量可能出现的问题）。
    全程无锁、无阻塞（除非 CPU 的原子指令排队，开销可忽略），执行效率接近普通变量的操作。*/
    std::atomic<bool> stop_thread_;  ///< 线程安全的停止标志：原子布尔量，用于通知保存线程退出（无锁同步）

    double fps_;  ///< 视频保存帧率：构造函数传入，控制视频帧的写入间隔，保证视频播放速度正常
    std::string text_path_;  ///< 文本文件保存路径：用于存储姿态、时间戳等结构化数据
    std::string video_path_;  ///< 视频文件保存路径：用于存储图像帧的视频流（如mp4/avi格式）
    std::ofstream text_writer_;  ///< 文本文件写入流：面向对象的文件输出流，写入姿态+时间戳等信息
    cv::VideoWriter video_writer_;  ///< 视频写入器：OpenCV视频编码写入器，将图像帧编码为视频文件
    std::chrono::steady_clock::time_point start_time_;  ///< 记录开始时间戳：首次初始化的时间，用于计算相对时间
    std::chrono::steady_clock::time_point last_time_;  ///< 上一帧写入文件的时间戳：用于控制视频帧率，避免帧速过快
    tools::ThreadSafeQueue<FrameData> queue_;  ///< 线程安全队列：生产者（record）和消费者（save_to_file）的通信缓冲区
    std::thread saving_thread_;  ///< 数据保存线程（消费者线程）：独立线程，从队列读取帧数据并写入文件，避免IO阻塞业务线程

    /**
     * @brief 初始化函数（私有）
     * @details 首次调用record时触发，根据首帧图像初始化视频写入器（分辨率、编码格式），
     *          创建并打开文本文件，初始化时间戳，启动保存线程，仅执行一次
     * @param img 首帧图像：用于获取视频的分辨率、通道数等参数
     */
    void init(const cv::Mat & img);

    /**
     * @brief 数据保存核心函数（消费者线程入口）
     * @details 保存线程的执行函数，循环从线程安全队列读取帧数据，
     *          按帧率要求将图像写入视频文件，将姿态+时间戳写入文本文件，
     *          直到检测到stop_thread_为true时退出循环，完成资源清理
     */
    void save_to_file();

};  // namespace tools

}
#endif  // TOOLS__RECORDER_HPP