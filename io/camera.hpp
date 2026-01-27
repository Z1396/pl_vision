/*2. 避免宏名冲突的关键细节

    大写：宏名通常全大写（C++ 约定俗成，区分普通变量 / 函数）；
    双下划线分隔路径：单下划线可能出现在文件名中（比如 serial_port.hpp），双下划线 __ 作为路径分隔符更安全；
    包含后缀 HPP：如果有同名的 .h 文件（比如 io/camera.h），加 HPP 能区分，避免冲突；
    禁止开头用双下划线：C++ 标准规定，开头是 __ 或 _ 加大写字母的宏名是编译器保留的（比如 __STDC__），所以宏名开头用 IO 而非 __IO。*/
#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

#include <chrono>          // 时间戳类型（std::chrono::steady_clock::time_point）
#include <memory>          // 智能指针（std::unique_ptr）
#include <opencv2/opencv.hpp> // OpenCV 图像类型（cv::Mat）
#include <string>          // 配置文件路径（std::string）


/*代码整体功能总结
这份头文件定义了两个核心类，实现了相机模块的接口抽象 + 封装管理：

    CameraBase：纯虚基类（接口类），定义了所有相机（海康、大华、迈德威视）必须实现的统一接口 read；
    Camera：封装类，通过智能指针管理具体的相机实例（HikRobot/DHUA/MindVision），对外暴露统一的 read 接口，屏蔽不同相机的实现差异。

简单说：外部代码只需调用 Camera::read，无需关心底层是海康还是大华相机 —— 这是「面向接口编程」的典型应用，也是你能在 camera.cpp 中根据配置动态切换相机类型的核心。*/
namespace io
{
class CameraBase
{
public:
/*（1）virtual ~CameraBase() = default; —— 虚析构函数
    为什么必须加？如果基类析构函数不是虚函数，当通过基类指针（std::unique_ptr<CameraBase>）销毁子类对象（HikRobot）时，子类的析构函数不会执行，会导致内存泄漏 / 资源释放不彻底（比如相机句柄没关闭）；
    = default：显式声明使用编译器生成的默认析构函数，比手动写空析构更高效。*/
  virtual ~CameraBase() = default; // 虚析构函数（关键！）

  /*（2）virtual void read(...) = 0; —— 纯虚函数
    = 0：表示这是纯虚函数，CameraBase 因此成为「抽象类」，不能直接实例化（只能被继承）；
    接口定义：强制所有子类（HikRobot/DHUA）必须实现 read 方法，保证所有相机都有统一的「读取图像 + 获取时间戳」接口；
    参数设计：
        cv::Mat & img：传引用，避免图像拷贝（自瞄项目中图像数据量大，拷贝会严重降低帧率）；
        std::chrono::steady_clock::time_point & timestamp：传递图像的采集时间戳，用于后续的时间同步（比如和 IMU 数据对齐）。*/
  virtual void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) = 0; // 纯虚函数
};


class Camera
{
public:
  /*（2）Camera(const std::string & config_path) 构造函数

    对外只暴露「配置文件路径」参数，屏蔽了具体相机的初始化参数（比如海康需要 gain，大华需要 vid_pid），外部代码无需关心不同相机的差异；
    你在 camera.cpp 中实现的逻辑：根据配置文件中的 camera_name 动态创建对应的相机实例，赋值给 camera_。*/
  Camera(const std::string & config_path); // 构造函数：通过配置文件初始化具体相机
  
  /*（3）void read(...) 成员函数

    对外暴露统一的读取接口，内部调用 camera_->read(...)（多态调用具体相机的 read 实现）；
    外部代码使用示例（无需关心底层相机类型）：*/
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp); // 对外暴露的统一接口

private:
  /*（1）成员变量 std::unique_ptr<CameraBase> camera_;
    核心选择：用 std::unique_ptr 而非 std::shared_ptr，因为一个 Camera 对象应该独占一个相机实例（相机是独占资源，不能被多个对象同时管理）；
    多态支持：unique_ptr 支持多态（指向基类，实际存储子类对象），这是你能动态切换海康 / 大华相机的关键；
    内存安全：unique_ptr 会在 Camera 对象销毁时自动释放相机实例，无需手动 delete，避免内存泄漏。*/
  std::unique_ptr<CameraBase> camera_; // 智能指针：管理具体的相机实例

};


}  // namespace io

#endif  // IO__CAMERA_HPP