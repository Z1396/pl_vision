#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <string>

#include "armor.hpp"
#include "solver.hpp"
#include "target.hpp"
#include "tasks/omniperception/perceptron.hpp"
#include "tools/thread_safe_queue.hpp"

namespace auto_aim
{
class Tracker
{
public:
  Tracker(const std::string & config_path, Solver & solver);

  std::string state() const;

  std::list<Target> track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t,
    bool use_enemy_color = true);

  std::tuple<omniperception::DetectionResult, std::list<Target>> track(
    const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
    std::chrono::steady_clock::time_point t, bool use_enemy_color = true);

private:
  Solver & solver_;
  Color enemy_color_; //标记敌方目标颜色
  int min_detect_count_;
  int max_temp_lost_count_;  //普通目标的 “最大临时丢失帧数”
  int detect_count_;  //目标的连续有效检测次数（初始为 0）
  int temp_lost_count_;  //目标的连续临时丢失次数（初始为 0）。
  int outpost_max_temp_lost_count_; //哨塔目标（固定目标，优先级更高）的 “最大临时丢失帧数”
  int normal_temp_lost_count_;  /*缓存 “普通目标的最大临时丢失帧数”简化逻辑：跟踪普通目标时，直接使用该变量判断丢失；
                                  无需每次都去读取max_temp_lost_count_，后续若需动态调整普通目标规则，也可单独修改该变量。*/	
  std::string state_, pre_state_;      
  Target target_; //存储当前跟踪目标的完整信息
  std::chrono::steady_clock::time_point last_timestamp_;  //上一帧跟踪的时间戳
  ArmorPriority omni_target_priority_; //标记全景相机（omni camera）目标的优先级

  void state_machine(bool found);

  bool set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);

  bool update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP