#include "yolo.hpp"

#include <yaml-cpp/yaml.h>

#include "yolos/yolo11.hpp"
#include "yolos/yolov5.hpp"
#include "yolos/yolov8.hpp"

namespace auto_aim
{
YOLO::YOLO(const std::string & config_path, bool debug)
{
  auto yaml = YAML::LoadFile(config_path);
  auto yolo_name = yaml["yolo_name"].as<std::string>();

  if (yolo_name == "yolov8") 
  {
    yolo_ = std::make_unique<YOLOV8>(config_path, debug);
  }

  else if (yolo_name == "yolo11") 
  {
    yolo_ = std::make_unique<YOLO11>(config_path, debug);
  }

  else if (yolo_name == "yolov5") 
  {
    yolo_ = std::make_unique<YOLOV5>(config_path, debug);    
    /*返回值是 std::unique_ptr<YOLOV5> 类型的智能指针。                       debug：布尔类型（true/false），用于控制是否启用调试模式。若为 true，可能会：
      YOLOV5 是 YOLOBase 的派生类（继承关系）。                                输出模型加载过程的详细日志（如权重加载进度、层结构信息）；
      unique_ptr 支持向上转型（派生类指针隐式转换为基类指针），这是多态特性的体现。   保存中间处理结果（如特征图可视化);打印检测过程中的关键参数（如推理耗时、检测到的目标数量）。*/
         
  }

  else 
  {
    throw std::runtime_error("Unknown yolo name: " + yolo_name + "!");
  }
}

std::list<Armor> YOLO::detect(const cv::Mat & img, int frame_count)
{
  return yolo_->detect(img, frame_count);
}

std::list<Armor> YOLO::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return yolo_->postprocess(scale, output, bgr_img, frame_count);
}

}  // namespace auto_aim