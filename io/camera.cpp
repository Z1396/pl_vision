#include "camera.hpp"

#include <stdexcept>

#include "hikrobot/hikrobot.hpp"
#include "mindvision/mindvision.hpp"
#include "dahua/dahua.hpp"
#include "tools/yaml.hpp"

namespace io
{
Camera::Camera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto camera_name = tools::read<std::string>(yaml, "camera_name");
  auto exposure_ms = tools::read<double>(yaml, "exposure_ms");

  // if (camera_name == "mindvision") 
  // {
  //   auto gamma = tools::read<double>(yaml, "gamma");
  //   auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
  //   camera_ = std::make_unique<mindvision>(exposure_ms, gamma, vid_pid);
  // }

  if (camera_name == "hikrobot") 
  {
    auto gain = tools::read<double>(yaml, "gain");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid);
  }

  else if (camera_name == "dahua") 
  {
    //auto gain = tools::read<double>(yaml, "gain");
    //auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    auto gain = 10.0;
    auto vid_pid = "174F:2435";
    camera_ = std::make_unique<DHUA>(exposure_ms, gain, vid_pid);
  }

  else {
    /*四、为什么用 <stdexcept> 而不是自定义错误码？
    自瞄项目中，新手常常用「返回错误码」处理错误（比如 int init_camera() { return -1; }），但对比 <stdexcept> 的异常处理，劣势明显：
    对比维度	std::runtime_error（异常）	                            自定义错误码（int）
    语义清晰	异常类型直接表示错误类型（运行时错误），错误信息可包含具体内容	  错误码只是数字（-1/-2），需查文档才知道含义
    中断可控	异常会自动中断当前函数，向上传递，无需手动层层返回错误码	      需手动检查每个函数的返回值，代码冗余（比如 if (init() < 0) return -1;）
    调试便捷	异常栈能定位到具体出错行（比如哪个相机名称未知）	             错误码只能知道「出错了」，无法定位具体位置
    资源安全	异常抛出时，栈上对象会自动析构（比如 unique_ptr 释放内存）	  错误码返回时，若忘记释放资源（比如相机句柄），会导致泄漏*/
    throw std::runtime_error("Unknow camera_name: " + camera_name + "!");
  }
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
}

}  // namespace io