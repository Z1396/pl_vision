// 引入决策器类头文件，包含类声明、成员函数原型、私有成员及依赖类型定义
#include "decider.hpp"

// 引入YAML配置文件解析库，用于加载相机参数、敌方颜色、运行模式等配置项
#include <yaml-cpp/yaml.h>

// 引入C++17文件系统库，用于配置文件路径的合法性校验（隐式使用）
#include <filesystem>
// 引入OpenCV核心库，用于图像数据存储与处理，接收相机采集的图像帧
#include <opencv2/opencv.hpp>

// 引入项目工具模块：日志工具（打印调试/信息/警告日志）、数学工具（角度限制、坐标转换等）
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

// 全景感知模块命名空间，隔离模块内代码，避免命名冲突
namespace omniperception
{

/**
 * @brief 决策器类构造函数
 * @details 初始化决策器核心参数，加载配置文件、初始化检测器、设置成员变量初始值，
 *          是决策器的入口初始化逻辑，所有配置项一次性从YAML文件加载，避免硬编码
 * @param config_path YAML配置文件路径，包含相机内参、视场角、敌方颜色、运行模式等核心配置
 * @note 初始化列表完成detector_（检测器）和count_（相机轮询计数器）的初始化，符合C++高效初始化规范
 */
Decider::Decider(const std::string & config_path) : detector_(config_path), count_(0)
{
  // 加载YAML配置文件，若路径错误/文件损坏会抛出异常
  auto yaml = YAML::LoadFile(config_path);
  // 读取相机图像分辨率（像素），用于后续图像归一化坐标解算
  img_width_ = yaml["image_width"].as<double>();
  img_height_ = yaml["image_height"].as<double>();
  // 读取相机原始水平/垂直视场角（FOV，单位：度），相机硬件固有参数
  fov_h_ = yaml["fov_h"].as<double>();
  fov_v_ = yaml["fov_v"].as<double>();
  // 读取相机校正后水平/垂直视场角，用于实际角度解算（补偿相机畸变/安装偏差）
  new_fov_h_ = yaml["new_fov_h"].as<double>();
  new_fov_v_ = yaml["new_fov_v"].as<double>();
  // 解析敌方装甲板颜色：配置文件"red"对应红色，否则为蓝色（关联auto_aim模块Color枚举）
  enemy_color_ =
    (yaml["enemy_color"].as<std::string>() == "red") ? auto_aim::Color::red : auto_aim::Color::blue;
  // 读取运行模式（整型），用于切换装甲板优先级策略（mode1/mode2）
  mode_ = yaml["mode"].as<double>();
}

/**
 * @brief 多相机融合决策核心接口（主决策接口）
 * @details 实现**双USB相机+后向相机**的轮询采集、装甲板检测、过滤、角度解算，
 *          最终输出云台控制指令，是全景感知自动瞄准的核心业务逻辑入口
 * @param yolo auto_aim模块YOLO检测器引用，用于图像装甲板检测
 * @param gimbal_pos 云台当前位姿（Eigen三维向量），包含云台当前横滚/俯仰/偏航角
 * @param usbcam1 左USB相机对象引用，负责左侧视野采集
 * @param usbcam2 右USB相机对象引用，负责右侧视野采集
 * @param back_camera 后向相机对象引用，负责后方视野采集
 * @return io::Command 云台控制指令，包含是否有目标、是否开火、目标偏航角、目标俯仰角
 * @note 采用3相机轮询策略（count_:0-左、1-右、2-后），覆盖全向视野，无视野盲区
 */
io::Command Decider::decide(
  auto_aim::YOLO & yolo, const Eigen::Vector3d & gimbal_pos, io::USBCamera & usbcam1,
  io::USBCamera & usbcam2, io::Camera & back_camera)
{
  // 存储解算后的云台目标角度增量（偏航yaw:0, 俯仰pitch:1）
  Eigen::Vector2d delta_angle;
  // 构建USB相机指针数组，简化相机轮询逻辑
  io::USBCamera * cams[] = {&usbcam1, &usbcam2};

  // 存储相机采集的图像帧、采集时间戳
  cv::Mat usb_img;
  std::chrono::steady_clock::time_point timestamp;
  
  // 相机轮询计数器合法性校验，超出[0,2]范围抛出运行时异常，防止数组越界
  if (count_ < 0 || count_ > 2) {
    throw std::runtime_error("count_ out of valid range [0,2]");
  }
  // 相机轮询采集逻辑：count_=2为后向相机，否则为对应索引的USB相机
  if (count_ == 2) {
    back_camera.read(usb_img, timestamp);
  } else {
    cams[count_]->read(usb_img, timestamp);
  }
  
  // 调用YOLO检测器检测当前图像帧中的装甲板，返回检测结果列表
  auto armors = yolo.detect(usb_img);
  // 装甲板过滤（敌方颜色、禁用装甲板、无敌装甲板），返回是否过滤后为空
  auto empty = armor_filter(armors);

  // 检测到有效装甲板：解算角度并生成控制指令
  if (!empty) {
    // 根据当前相机类型解算角度增量：后向相机传"back"，USB相机传设备名（left/right）
    if (count_ == 2) {
      delta_angle = this->delta_angle(armors, "back");
    } else {
      delta_angle = this->delta_angle(armors, cams[count_]->device_name);
    }

    // 打印调试日志：包含相机类型、角度增量、装甲板数量、目标装甲板名称
    tools::logger()->debug(
      "[{} camera] delta yaw:{:.2f},target pitch:{:.2f},armor number:{},armor name:{}",
      (count_ == 2 ? "back" : cams[count_]->device_name), delta_angle[0], delta_angle[1],
      armors.size(), auto_aim::ARMOR_NAMES[armors.front().name]);

    // 相机轮询计数器自增并取模3，实现0→1→2→0的循环轮询
    count_ = (count_ + 1) % 3;

    // 生成并返回云台控制指令：
    // 1. has_target=true（有有效目标）；2. fire=false（暂不开火）；
    // 3. yaw=当前云台偏航角+角度增量/57.3（弧度转换，57.3≈180/π）；
    // 4. pitch=角度增量/57.3（弧度转换）；5. 角度限制在合法范围（limit_rad）
    return io::Command{
      true, false, tools::limit_rad(gimbal_pos[0] + delta_angle[0] / 57.3),
      tools::limit_rad(delta_angle[1] / 57.3)};
  }

  // 未检测到有效装甲板：计数器自增轮询下一个相机，返回空指令（无目标）
  count_ = (count_ + 1) % 3;
  return io::Command{false, false, 0, 0};
}

/**
 * @brief 单后向相机决策接口（重载接口）
 * @details 仅使用后向相机进行图像采集和决策，适用于**仅后方视野警戒**的场景，
 *          逻辑与多相机接口一致，简化单相机使用流程
 * @param yolo auto_aim模块YOLO检测器引用
 * @param gimbal_pos 云台当前位姿（Eigen三维向量）
 * @param back_cammera 后向相机对象引用（参数名笔误：应为back_camera，保持业务一致性）
 * @return io::Command 云台控制指令，格式与主接口一致
 */
io::Command Decider::decide(
  auto_aim::YOLO & yolo, const Eigen::Vector3d & gimbal_pos, io::Camera & back_cammera)
{
  // 存储后向相机采集图像、时间戳
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  // 采集后向相机图像
  back_cammera.read(img, timestamp);
  // YOLO检测装甲板
  auto armors = yolo.detect(img);
  // 装甲板过滤，判断是否有有效目标
  auto empty = armor_filter(armors);

  // 检测到有效目标：解算后向相机角度增量，生成控制指令
  if (!empty) {
    auto delta_angle = this->delta_angle(armors, "back");
    // 打印调试日志
    tools::logger()->debug(
      "[back camera] delta yaw:{:.2f},target pitch:{:.2f},armor number:{},armor name:{}",
      delta_angle[0], delta_angle[1], armors.size(), auto_aim::ARMOR_NAMES[armors.front().name]);

    // 生成云台控制指令，格式与主接口一致
    return io::Command{
      true, false, tools::limit_rad(gimbal_pos[0] + delta_angle[0] / 57.3),
      tools::limit_rad(delta_angle[1] / 57.3)};
  }

  // 未检测到有效目标，返回空指令
  return io::Command{false, false, 0, 0};
}

/**
 * @brief 检测结果队列决策接口（重载接口）
 * @details 接收外部传入的检测结果队列，直接解析并生成云台控制指令，
 *          适用于**多线程检测+主线程决策**的解耦架构，检测与决策异步执行
 * @param detection_queue 检测结果队列（std::vector<DetectionResult>），包含多帧/多相机检测结果
 * @return io::Command 云台控制指令，格式与其他接口一致
 * @note 检测结果已完成装甲板检测与角度解算，本接口仅做结果解析和指令生成
 */
io::Command Decider::decide(const std::vector<DetectionResult> & detection_queue)
{
  // 检测结果队列为空，返回空指令
  if (detection_queue.empty()) {
    return io::Command{false, false, 0, 0};
  }

  // 取队列首个检测结果（最新/优先级最高结果）
  DetectionResult dr = detection_queue.front();
  // 检测结果中无装甲板，返回空指令
  if (dr.armors.empty()) return io::Command{false, false, 0, 0};
  
  // 打印信息日志：输出检测到的装甲板名称、偏航角增量（弧度转角度）
  tools::logger()->info(
    "omniperceptron find {},delta yaw is {:.4f}", auto_aim::ARMOR_NAMES[dr.armors.front().name],
    dr.delta_yaw * 57.3);

  // 生成控制指令：直接使用检测结果中的角度增量（已为弧度制）
  return io::Command{true, false, dr.delta_yaw, dr.delta_pitch};
};

/**
 * @brief 云台目标角度增量解算核心函数
 * @details 根据**相机类型**和装甲板归一化中心坐标，解算云台需要转动的偏航/俯仰角度增量，
 *          是图像像素坐标到云台机械角度的核心转换逻辑，适配不同相机的安装位置和视场角
 * @param armors 过滤后的有效装甲板列表，取首个为目标装甲板
 * @param camera 相机类型标识（left/right/back），对应不同的解算公式
 * @return Eigen::Vector2d 角度增量（单位：度），[0]偏航yaw，[1]俯仰pitch
 * @note 装甲板center_norm为归一化坐标（x/y∈[0,1]），已消除图像分辨率的影响
 */
Eigen::Vector2d Decider::delta_angle(
  const std::list<auto_aim::Armor> & armors, const std::string & camera)
{
  Eigen::Vector2d delta_angle;
  // 左相机角度解算：补偿左相机安装偏角62度，使用校正后视场角
  if (camera == "left") {
    delta_angle[0] = 62 + (new_fov_h_ / 2) - armors.front().center_norm.x * new_fov_h_;
    delta_angle[1] = armors.front().center_norm.y * new_fov_v_ - new_fov_v_ / 2;
    return delta_angle;
  }
  // 右相机角度解算：补偿右相机安装偏角-62度，使用校正后视场角
  else if (camera == "right") {
    delta_angle[0] = -62 + (new_fov_h_ / 2) - armors.front().center_norm.x * new_fov_h_;
    delta_angle[1] = armors.front().center_norm.y * new_fov_v_ - new_fov_v_ / 2;
    return delta_angle;
  }
  // 后向相机角度解算：使用固定视场角（水平54.2度，垂直44.5度），无安装偏角补偿
  else {
    delta_angle[0] = 170 + (54.2 / 2) - armors.front().center_norm.x * 54.2;
    delta_angle[1] = armors.front().center_norm.y * 44.5 - 44.5 / 2;
    return delta_angle;
  }
}

/**
 * @brief 装甲板多条件过滤函数
 * @details 对检测到的装甲板进行**多层过滤**，剔除无效目标，保证传入后续逻辑的为合法装甲板，
 *          是提升瞄准精度、降低误判的关键过滤环节
 * @param armors 待过滤的装甲板列表（引用传递，直接修改原列表）
 * @return bool 过滤后列表是否为空，true=无有效装甲板，false=有有效装甲板
 * @note 过滤规则按**优先级从高到低**执行，提前剔除无效目标，减少后续计算量
 */
bool Decider::armor_filter(std::list<auto_aim::Armor> & armors)
{
  // 初始为空直接返回true，无需过滤
  if (armors.empty()) return true;
  
  // 第一层过滤：剔除非敌方颜色的装甲板（核心过滤，优先保留敌方目标）
  armors.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });

  // 第二层过滤：剔除25赛季禁用的5号装甲板（业务规则约束）
  armors.remove_if([&](const auto_aim::Armor & a) { return a.name == auto_aim::ArmorName::five; });
  // 注释：业务可选规则，不攻击工程车2号装甲板
  // armors.remove_if([&](const auto_aim::Armor & a) { return a.name == auto_aim::ArmorName::two; });
  // 第三层过滤：剔除前哨站装甲板（业务规则，不攻击前哨站）
  armors.remove_if(
    [&](const auto_aim::Armor & a) { return a.name == auto_aim::ArmorName::outpost; });

  // 第四层过滤：剔除刚复活处于无敌状态的装甲板（接收自通信模块的无敌列表）
  armors.remove_if([&](const auto_aim::Armor & a) {
    return std::find(invincible_armor_.begin(), invincible_armor_.end(), a.name) !=
           invincible_armor_.end();
  });

  // 返回过滤后是否为空，标识是否有有效目标
  return armors.empty();
}

/**
 * @brief 装甲板优先级设置函数
 * @details 根据当前运行模式（mode_），为有效装甲板设置**攻击优先级**，
 *          优先级数值越小，攻击优先级越高，为后续装甲板排序提供依据
 * @param armors 有效装甲板列表（引用传递，修改装甲板priority成员）
 * @note 优先级映射表（priority_map）分为mode1和mode2，由配置文件指定，适配不同战场策略
 */
void Decider::set_priority(std::list<auto_aim::Armor> & armors)
{
  if (armors.empty()) return;

  // 根据运行模式选择对应的优先级映射表：MODE_ONE对应mode1，否则为mode2
  const PriorityMap & priority_map = (mode_ == MODE_ONE) ? mode1 : mode2;

  // 遍历所有装甲板，从优先级映射表中获取对应优先级并赋值
  if (!armors.empty()) {
    for (auto & armor : armors) {
      armor.priority = priority_map.at(armor.name);
    }
  }
}

/**
 * @brief 检测结果队列排序函数
 * @details 对多帧/多相机的检测结果队列进行**双层排序**，先排装甲板再排检测结果，
 *          保证最终取到的是**全局优先级最高**的目标，实现精准选靶
 * @param detection_queue 检测结果队列（引用传递，直接修改队列顺序）
 * @note 排序规则：优先级数值越小，越靠前；首个元素为全局最优目标
 */
void Decider::sort(std::vector<DetectionResult> & detection_queue)
{
  if (detection_queue.empty()) return;

  // 第一层排序：遍历每个检测结果，先过滤装甲板→设置优先级→对装甲板按优先级升序排序
  for (auto & dr : detection_queue) {
    armor_filter(dr.armors);
    set_priority(dr.armors);
    // 装甲板排序：优先级小的在前（优先攻击）
    dr.armors.sort(
      [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });
  }

  // 第二层排序：对检测结果队列按**首个装甲板的优先级**升序排序，全局选最优目标
  std::sort(
    detection_queue.begin(), detection_queue.end(),
    [](const DetectionResult & a, const DetectionResult & b) {
      return a.armors.front().priority < b.armors.front().priority;
    });
}

/**
 * @brief 目标信息获取函数
 * @details 结合装甲板检测结果和目标跟踪结果，解算**目标在云台坐标系下的三维坐标**，
 *          并生成目标标识信息，为通信模块/跟踪模块提供目标精准位置
 * @param armors 有效装甲板列表
 * @param targets 目标跟踪结果列表（auto_aim模块Target）
 * @return Eigen::Vector4d 目标信息向量：[x,y,有效标识,装甲板名称编码]
 *         1. x/y：云台坐标系下目标横/纵坐标；2. 有效标识：1=有效，0=无效；
 *         3. 装甲板名称编码：原名称+1（避免0值歧义，适配通信协议）
 */
Eigen::Vector4d Decider::get_target_info(
  const std::list<auto_aim::Armor> & armors, const std::list<auto_aim::Target> & targets)
{
  // 装甲板或跟踪结果为空，返回零向量（无效目标）
  if (armors.empty() || targets.empty()) return Eigen::Vector4d::Zero();

  // 取首个跟踪目标（跟踪优先级最高）
  auto target = targets.front();

  // 遍历装甲板，匹配跟踪目标名称，找到对应装甲板的三维坐标
  for (const auto & armor : armors) {
    if (armor.name == target.name) {
      // 生成目标信息向量，装甲板名称+1避免通信协议0值歧义
      return Eigen::Vector4d{
        armor.xyz_in_gimbal[0], armor.xyz_in_gimbal[1], 1,
        static_cast<double>(armor.name) + 1};
    }
  }

  // 无匹配目标，返回零向量
  return Eigen::Vector4d::Zero();
}

/**
 * @brief 无敌装甲板列表更新函数
 * @details 接收从**通信模块/导航模块**传入的敌方无敌装甲板ID列表，
 *          转换为auto_aim模块ArmorName枚举，更新本地无敌装甲板列表，为过滤提供依据
 * @param invincible_enemy_ids 无敌装甲板ID列表（int8_t），来自外部通信
 * @note ID与ArmorName枚举的映射关系：ID-1 = 枚举值（枚举从0开始，ID从1开始）
 */
void Decider::get_invincible_armor(const std::vector<int8_t> & invincible_enemy_ids)
{
  // 清空本地无敌列表，避免旧数据干扰
  invincible_armor_.clear();

  if (invincible_enemy_ids.empty()) return;

  // 遍历无敌ID列表，转换为ArmorName枚举并添加到本地列表，打印信息日志
  for (const auto & id : invincible_enemy_ids) {
    tools::logger()->info("invincible armor id: {}", id);
    invincible_armor_.push_back(auto_aim::ArmorName(id - 1));
  }
}

/**
 * @brief 自动瞄准目标指定函数
 * @details 接收外部（导航模块/遥控端）传入的**指定攻击目标列表**，
 *          过滤装甲板列表仅保留指定目标，实现**人工/导航选靶**功能，适配战术需求
 * @param armors 有效装甲板列表（引用传递，直接过滤为指定目标）
 * @param auto_aim_target 外部指定的装甲板目标ID列表（int8_t）
 * @note 包含入参合法性校验，对无效ID打印警告日志，保证程序鲁棒性
 */
void Decider::get_auto_aim_target(
  std::list<auto_aim::Armor> & armors, const std::vector<int8_t> & auto_aim_target)
{
  if (auto_aim_target.empty()) return;

  // 存储转换后的合法指定目标枚举列表
  std::vector<auto_aim::ArmorName> auto_aim_targets;

  // 遍历外部指定ID，转换为ArmorName枚举并做合法性校验
  for (const auto & target : auto_aim_target) {
    // 合法性校验：ID需大于0且不超过装甲板名称总数，否则为无效ID
    if (target <= 0 || static_cast<size_t>(target) > auto_aim::ARMOR_NAMES.size()) {
      tools::logger()->warn("Received invalid auto_aim target value: {}", int(target));
      continue;
    }
    // ID转换为枚举：ID-1 = 枚举值，添加到合法列表并打印信息日志
    auto_aim_targets.push_back(static_cast<auto_aim::ArmorName>(target - 1));
    tools::logger()->info("nav send auto_aim target is {}", auto_aim::ARMOR_NAMES[target - 1]);
  }

  // 无合法指定目标，直接返回
  if (auto_aim_targets.empty()) return;

  // 过滤装甲板列表：仅保留指定目标，剔除其他所有装甲板
  armors.remove_if([&](const auto_aim::Armor & a) {
    return std::find(auto_aim_targets.begin(), auto_aim_targets.end(), a.name) ==
           auto_aim_targets.end();
  });
}

}  // namespace omniperception
