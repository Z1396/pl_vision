#include "tracker.hpp"

#include <yaml-cpp/yaml.h>

#include <tuple>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Tracker::Tracker(const std::string & config_path, Solver & solver)
: solver_{solver},
  detect_count_(0),
  temp_lost_count_(0),
  state_{"lost"},
  pre_state_{"lost"},
  last_timestamp_(std::chrono::steady_clock::now()),
  omni_target_priority_{ArmorPriority::fifth}
{
  auto yaml = YAML::LoadFile(config_path);
  enemy_color_ = (yaml["enemy_color"].as<std::string>() == "red") ? Color::red : Color::blue;
  min_detect_count_ = yaml["min_detect_count"].as<int>();
  max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();
  outpost_max_temp_lost_count_ = yaml["outpost_max_temp_lost_count"].as<int>();
  normal_temp_lost_count_ = max_temp_lost_count_;
}

std::string Tracker::state() const { return state_; }

/**
 * @brief 跟踪目标装甲板的主函数
 * 
 * 该函数是跟踪器的核心逻辑，负责处理每帧检测到的装甲板列表，
 * 实现目标的状态管理（丢失/跟踪中）、时间间隔计算、装甲板过滤与排序，
 * 并根据当前状态选择设置新目标或更新已有目标，最终返回有效的跟踪目标列表。
 * 
 * @param armors 检测到的装甲板列表
 * @param t 当前帧的时间戳
 * @param use_enemy_color 是否根据敌方颜色过滤装甲板
 * @return std::list<Target> 跟踪到的目标列表（通常包含一个目标）
 */
std::list<Target> Tracker::track(
  std::list<Armor> & armors, std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
    // 计算当前帧与上一帧的时间间隔（单位：秒）
    auto dt = tools::delta_time(t, last_timestamp_);
    last_timestamp_ = t;  // 更新上一帧时间戳为当前时间

    // 若时间间隔超过0.1秒且当前状态不是"丢失"，则判定为相机可能离线，将状态设为"丢失"
    if ((state_ != "lost" && dt > 1.0)) 
    {
        tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);  // 记录警告日志
        state_ = "lost";
    }
    

    // 过滤掉颜色不符合敌方颜色的装甲板（仅保留敌方目标）
    armors.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });

    // 注释掉的代码：过滤前哨站顶部装甲板（可能用于特定场景的优化）
    // armors.remove_if([this](const auto_aim::Armor & a) 
    // {
    //   return a.name == ArmorName::outpost &&
    //          solver_.oupost_reprojection_error(a, 27.5 * CV_PI / 180.0) <
    //            solver_.oupost_reprojection_error(a, -15 * CV_PI / 180.0);
    // });

    // 第一次排序：按装甲板中心到图像中心的距离从小到大排序（优先选择更居中的目标）
    armors.sort([](const Armor & a, const Armor & b) 
    {
        cv::Point2f img_center(1440.0 / 2, 1080.0 / 2);  // 图像中心坐标（TODO：应动态获取分辨率）  
        auto distance_1 = cv::norm(a.center - img_center);  // 装甲板a到中心的距离
        auto distance_2 = cv::norm(b.center - img_center);  // 装甲板b到中心的距离
        return distance_1 < distance_2;  // 距离小的排在前面
    });

    // 第二次排序：按装甲板优先级从小到大排序（优先级数值越小，优先级越高）
    armors.sort(
        [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });

    bool found;  // 标记是否找到可跟踪的目标
    if (state_ == "lost") 
    {
        // 若当前状态为"丢失"，则尝试从装甲板列表中设置新目标
        //tools::logger()->warn("lost", dt);  // 记录警告日志
        found = set_target(armors, t);
    } else 
    {
        //tools::logger()->warn("found", dt);  // 记录警告日志
        // 若当前处于跟踪状态，则更新已有目标的状态
        found = update_target(armors, t);
    }

    // 状态机处理：根据是否找到目标更新跟踪器状态（如从"搜索"到"跟踪"，或从"跟踪"到"丢失"）
    state_machine(found);

    // 发散检测：若目标状态发散（如估计误差过大），则将状态设为"丢失"，返回空列表
    if (state_ != "lost" && target_.diverged()) 
    {
        tools::logger()->debug("[Tracker] Target diverged!");  // 记录调试日志
        state_ = "lost";
        return {};
    }

    // 收敛效果检测：若最近NIS（归一化创新平方）失败次数过多，判定为收敛效果差，设为"丢失"
    if (
        std::accumulate(
            target_.ekf().recent_nis_failures.begin(), target_.ekf().recent_nis_failures.end(), 0) >=
        (0.4 * target_.ekf().window_size)) 
    {
        tools::logger()->debug("[Target] Bad Converge Found!");  // 记录调试日志
        state_ = "lost";
        return {};
    }

    // 若最终状态为"丢失"，返回空列表
    if (state_ == "lost") return {};

    // 若跟踪成功，返回包含当前目标的列表
    std::list<Target> targets = {target_};
    return targets;
}

std::tuple<omniperception::DetectionResult, std::list<Target>> Tracker::track(
  const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
  std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  omniperception::DetectionResult switch_target{std::list<Armor>(), t, 0, 0};
  omniperception::DetectionResult temp_target{std::list<Armor>(), t, 0, 0};
  if (!detection_queue.empty()) 
  {
    temp_target = detection_queue.front();
  }

  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) 
  {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) 
  {
    cv::Point2f img_center(1440.0 / 2, 1080.0 / 2);  // TODO: 应动态获取分辨率  
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort([](const Armor & a, const Armor & b) { return a.priority < b.priority; });

  bool found;
  if (state_ == "lost") 
  {
    found = set_target(armors, t);
  }

  // 此时主相机画面中出现了优先级更高的装甲板，切换目标
  else if (state_ == "tracking" && !armors.empty() && armors.front().priority < target_.priority) 
  {
    found = set_target(armors, t);
    tools::logger()->debug("auto_aim switch target to {}", ARMOR_NAMES[armors.front().name]);
  }

  // 此时全向感知相机画面中出现了优先级更高的装甲板，切换目标
  else if (
    state_ == "tracking" && !temp_target.armors.empty() &&
    temp_target.armors.front().priority < target_.priority && target_.convergened()) 
  {
    state_ = "switching";
    switch_target = omniperception::DetectionResult{
      temp_target.armors, t, temp_target.delta_yaw, temp_target.delta_pitch};
    omni_target_priority_ = temp_target.armors.front().priority;
    found = false;
    tools::logger()->debug("omniperception find higher priority target");
  }

  else if (state_ == "switching") 
  {
    found = !armors.empty() && armors.front().priority == omni_target_priority_;
  }

  else if (state_ == "detecting" && pre_state_ == "switching") 
  {
    found = set_target(armors, t);
  }

  else 
  {
    found = update_target(armors, t);
  }

  pre_state_ = state_;
  // 更新状态机
  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_.diverged()) 
  {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {switch_target, {}};  // 返回switch_target和空的targets
  }

  if (state_ == "lost") return {switch_target, {}};  // 返回switch_target和空的targets

  std::list<Target> targets = {target_};
  return {switch_target, targets};
}

void Tracker::state_machine(bool found)
{
  if (state_ == "lost") 
  {
    if (!found) return;

    state_ = "detecting";
    detect_count_ = 1;
  }

  else if (state_ == "detecting") 
  {
    if (found) 
    {
      detect_count_++;
      if (detect_count_ >= min_detect_count_) state_ = "tracking";
    } else 
    {
      detect_count_ = 0;
      state_ = "lost";
    }
  }

  else if (state_ == "tracking") 
  {
    if (found) return;

    temp_lost_count_ = 1;
    state_ = "temp_lost";
  }

  else if (state_ == "switching") 
  {
    if (found) 
    {
      state_ = "detecting";
    } else 
    {
      temp_lost_count_++;
      if (temp_lost_count_ > 200) state_ = "lost";
    }
  }

  else if (state_ == "temp_lost") 
  {
    if (found) 
    {
      state_ = "tracking";
    } else 
    {
      temp_lost_count_++;
      if (target_.name == ArmorName::outpost)
        //前哨站的temp_lost_count需要设置的大一些
        max_temp_lost_count_ = outpost_max_temp_lost_count_;
      else
        max_temp_lost_count_ = normal_temp_lost_count_;

      if (temp_lost_count_ > max_temp_lost_count_) state_ = "lost";
    }
  }
}

/**
 * @brief 设置目标跟踪器设置目标装甲板，并
 * 该函数从检测到的装甲板列表中选择第一个装甲板作为跟踪目标，
 * 根据装甲板的类型（兵种兵种）初始化目标跟踪器的参数，
 * 包括状态协方差矩阵的初始值、过程噪声等，
 * 为后续的目标跟踪（如卡尔曼滤波）提供初始状态。
 * 
 * @param armors 检测到的装甲板列表（非空），通常已按优先级排序（如最靠近图像中心的在前）
 * @param t 时间戳，记录目标设置的时刻（用于跟踪时序同步）
 * @return bool 目标设置成功返回true，若装甲板列表为空则返回false
 */
bool Tracker::set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
    // 若装甲板列表为空，无法设置目标，返回失败
    if (armors.empty()) 
    {
      tools::logger()->warn("armors_empty");  // 记录警告日志
      return false;
    }

    // 取列表中第一个装甲板作为目标（假设列表已按优先级排序，如最优先攻击的在前）
    auto & armor = armors.front();
    // 通过解算器处理该装甲板（可能是计算其在世界坐标系中的位置等）
    solver_.solve(armor);

    // 判断是否为平衡步兵装甲板（根据装甲板类型和名称）
    // 平衡步兵通常有特定的装甲板标识（big类型且名称为three/four/five）
    auto is_balance = (armor.type == ArmorType::big) &&
                      (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                       armor.name == ArmorName::five);

    tools::logger()->warn(is_balance); 

    // 根据不同兵种的装甲板类型，初始化目标跟踪的参数（适配不同目标的运动特性）
    if (is_balance) 
    {
        // 平衡步兵的初始状态协方差矩阵（P0_dig），数值根据经验或标定确定
        // 矩阵元素反映对各状态量（如位置、速度等）初始不确定性的估计
        Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
        // 初始化目标对象，参数包括：装甲板数据、时间戳、过程噪声系数、状态维度、初始协方差
        target_ = Target(armor, t, 0.2, 2, P0_dig);
    }

    // 若为前哨站装甲板
    else if (armor.name == ArmorName::outpost) 
    {
        Eigen::VectorXd P0_dig{{1, 64, 1, 64, 5, 81, 0.4, 100, 1e-4, 1e-6, 1e-6}};
        target_ = Target(armor, t, 0.2765, 3, P0_dig);
        
    }

    // 若为基地装甲板
    else if (armor.name == ArmorName::base) 
    {
        Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
        target_ = Target(armor, t, 0.3205, 3, P0_dig);
    }

    // 其他类型装甲板（如普通步兵、英雄机器人等）
    else 
    {
        Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};  //这是一个包含 11 个元素的动态大小列向量
        target_ = Target(armor, t, 0.2, 4, P0_dig);
    }

    // 目标设置成功
    return true;
}

/**
 * @brief 更新跟踪器中的目标状态
 * 
 * 该函数首先预测目标当前状态，然后从检测到的装甲板列表中筛选出与当前跟踪目标匹配的装甲板，
 * 计算最优匹配的装甲板（最左侧），并使用该装甲板的信息更新目标状态，实现对目标的持续跟踪。
 * 
 * @param armors 最新检测到的装甲板列表
 * @param t 当前时间戳（用于状态预测的时间同步）
 * @return bool 目标更新成功返回true，未找到匹配的装甲板返回false
 */
bool Tracker::update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
    // 基于当前时间戳预测目标的状态（如位置、速度等，通常使用卡尔曼滤波等算法）
    target_.predict(t);

    // 统计与当前目标匹配的装甲板数量，并寻找其中最左侧的装甲板（x坐标最小）
    int found_count = 0;               // 匹配到的装甲板数量
    double min_x = 1e10;               // 初始化最左侧x坐标为一个很大的值
    for (const auto & armor : armors) 
    {
        // 筛选条件：装甲板名称和类型需与当前跟踪目标一致（确保是同一类目标）
        if (armor.name != target_.name || armor.type != target_.armor_type) continue;
        
        found_count++;                 // 找到匹配的装甲板，计数加1
        // 更新最左侧x坐标（取最小的x值）
        min_x = armor.center.x < min_x ? armor.center.x : min_x;
    }

    // 若未找到任何匹配的装甲板，无法更新目标，返回l失败
    if (found_count == 0) return false;

    // 遍历装甲板列表，找到最左侧的匹配装甲板并更新目标状态
    for (auto & armor : armors) 
    {
        // 筛选条件：
        // 1. 装甲板名称和类型与目标一致
        // 2. 装甲板是最左侧的（x坐标等于min_x）
        if 
        (
            armor.name != target_.name || armor.type != target_.armor_type
            // || armor.center.x != min_x  // 注释掉的条件，可能用于调试或特殊场景
        )
            continue;

        // 解算该装甲板的相关信息（如世界坐标系下的位置等）
        solver_.solve(armor);
        // 使用该装甲板的信息更新目标状态（如修正预测值，更新滤波参数等）
        target_.update(armor);
    }

    // 目标更新成功
    return true;
}

}  // namespace auto_aim