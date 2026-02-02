#include "solver.hpp"

// YAML配置文件解析库：加载相机内参、硬件外参等配置参数
#include <yaml-cpp/yaml.h>
// 标准容器：存储装甲板三维特征点、像素点等集合数据
#include <vector>

// 项目工具库：日志打印、数学工具函数（欧拉角转换、角度限制、坐标转ypd等）
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

// 自动瞄准模块命名空间，隔离模块内代码，避免命名冲突
namespace auto_aim
{
// 装甲板物理尺寸常量定义（单位：米，基于实际靶标尺寸标定，保证PnP解算精度）
constexpr double LIGHTBAR_LENGTH = 56e-3;     // 装甲板灯条长度：56毫米
constexpr double BIG_ARMOR_WIDTH = 230e-3;    // 大型装甲板整体宽度：230毫米
constexpr double SMALL_ARMOR_WIDTH = 135e-3;  // 小型装甲板整体宽度：135毫米

// 大型装甲板三维特征点（装甲板本地坐标系）
// 坐标系定义：原点在装甲板中心，X轴向前（垂直装甲板平面），Y轴向左，Z轴向上
// 点顺序：右上、左上、左下、右下（与图像中检测到的装甲板角点顺序严格对应，保证PnP匹配）
const std::vector<cv::Point3f> BIG_ARMOR_POINTS{
  {0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},   // 右上
  {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},  // 左上
  {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}, // 左下
  {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}}; // 右下
  
// 小型装甲板三维特征点（坐标系、点顺序与大型装甲板完全一致，仅尺寸不同）
const std::vector<cv::Point3f> SMALL_ARMOR_POINTS{
  {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};

/**
 * @brief Solver类构造函数：初始化硬件外参、相机内参/畸变系数，加载配置文件
 * @param config_path YAML配置文件路径（包含相机内参、硬件安装外参）
 * @details 1. 初始化云台→世界旋转矩阵为单位矩阵（初始无旋转，云台朝向与世界坐标系一致）；
 *          2. 加载YAML配置文件，提取旋转矩阵、平移向量、相机参数的扁平化数据；
 *          3. 将扁平化vector数据转换为Eigen矩阵/向量（按行存储，匹配配置文件格式）；
 *          4. 将Eigen格式的相机参数转换为OpenCV格式，用于后续PnP、投影等操作；
 * @note 成员初始化列表：R_gimbal2world_(Eigen::Matrix3d::Identity()) 确保初始旋转无偏移
 * @note 配置文件数据格式要求：所有矩阵为3×3扁平化（9个元素），平移向量为3个元素，相机内参3×3，畸变系数5个元素
 */
Solver::Solver(const std::string & config_path) : R_gimbal2world_(Eigen::Matrix3d::Identity())  
{
  // 1. 加载YAML配置文件：返回根节点，通过键值对访问所有配置项
  auto yaml = YAML::LoadFile(config_path);

  // 2. 提取硬件外参扁平化数据（配置文件中为数组格式，直接转换为double类型vector）
  auto R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();  // 云台→IMU机体旋转矩阵（9元素）
  auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();    // 相机→云台旋转矩阵（9元素）
  auto t_camera2gimbal_data = yaml["t_camera2gimbal"].as<std::vector<double>>();    // 相机→云台平移向量（3元素：x,y,z，单位米）

  // 3. 扁平化vector转Eigen矩阵/向量（Eigen::RowMajor 按行存储，与配置文件数组顺序一致）
  // 旋转矩阵为3×3 Eigen矩阵，平移向量为3×1 Eigen列向量，直接通过数据首地址构造，高效无拷贝
  R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2world_.data());
  R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
  t_camera2gimbal_ = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());

  // 4. 提取相机参数扁平化数据（内参3×3，畸变系数1×5）
  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();    // 相机内参矩阵（9元素：[fx,0,cx;0,fy,cy;0,0,1]）
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();  // 畸变系数（5元素：k1,k2,p1,p2,k3，径向+切向）

  // 5. 相机参数转Eigen格式（临时变量，用于后续转OpenCV）
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());  // 3×3内参矩阵
  Eigen::Matrix<double, 1, 5> distort_coeffs(distort_coeffs_data.data());                // 1×5畸变系数行向量

  // 6. Eigen→OpenCV格式转换（cv::eigen2cv 无缝转换，数据类型、尺寸完全匹配，无精度损失）
  // 转换后存入类成员变量，用于OpenCV的solvePnP、projectPoints等函数（仅支持cv::Mat格式）
  cv::eigen2cv(camera_matrix, camera_matrix_);
  cv::eigen2cv(distort_coeffs, distort_coeffs_);
}

/**
 * @brief 只读接口：获取云台→世界坐标系的旋转矩阵
 * @return Eigen::Matrix3d 3×3正交单位旋转矩阵
 * @const 方法不修改类内任何成员，仅返回实时姿态矩阵
 */
Eigen::Matrix3d Solver::R_gimbal2world() const { return R_gimbal2world_; }

/**
 * @brief 更新云台→世界坐标系的旋转矩阵（通过IMU四元数）
 * @param q 云台实时姿态四元数（Eigen::Quaterniond，单位化四元数）
 * @details 1. 将IMU四元数转换为IMU机体→IMU绝对坐标系的旋转矩阵；
 *          2. 通过**相似变换**计算云台→世界旋转矩阵，消除IMU与云台的安装姿态偏差；
 *          3. 旋转矩阵转置=逆（正交矩阵特性），避免求逆运算，提升效率并减少数值误差；
 * @note 四元数需保证单位化，否则转换的旋转矩阵非正交，会导致坐标解算失真
 * @note 相似变换公式：R_gimbal2world = R_gimbal2imubody^T * R_imubody2imuabs * R_gimbal2imubody
 */
void Solver::set_R_gimbal2world(const Eigen::Quaterniond & q)
{
  // 四元数转旋转矩阵：IMU机体坐标系→IMU绝对坐标系（世界坐标系）
  Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
  // 相似变换：将IMU绝对姿态转换为云台绝对姿态，补偿云台与IMU的安装旋转偏差
  R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}

/**
 * @brief 核心解算接口：单目视觉PnP解算+多坐标系转换+云台角度优化
 * @param armor 输入输出参数：传入检测到的装甲板（含2D像素角点、类型、名称），
 *              解算后更新3D坐标、欧拉角、ypd瞄准参数等核心信息
 * @details 完整解算流程：
 *          1. 按装甲板类型选择匹配的3D特征点（大/小）；
 *          2. 调用OpenCV IPPE-PnP算法，解算装甲板在相机坐标系下的位姿（旋转向量+平移向量）；
 *          3. 相机坐标系→云台坐标系→世界坐标系，完成3D坐标链式转换；
 *          4. 旋转向量转旋转矩阵，完成装甲板→相机/云台/世界的旋转矩阵转换；
 *          5. 旋转矩阵转欧拉角（yaw/pitch/roll），分别存储在云台/世界坐标系下；
 *          6. 世界坐标系3D坐标转ypd（方位角/俯仰角/距离），为云台控制提供直接瞄准参数；
 *          7. 平衡步兵装甲板跳过yaw优化（运动特性导致pitch假设不成立，优化会引入误差）；
 *          8. 非平衡步兵装甲板调用optimize_yaw，优化yaw角以减小重投影误差，提升瞄准精度；
 * @const 方法不修改类内成员，仅修改输入的Armor对象
 * @note IPPE-PnP算法：专为平面目标设计，单目相机鲁棒性强、计算速度快，适配装甲板平面特性
 * @note 坐标转换公式：P_target = R * P_prev + t（刚体变换，旋转+平移）
 */
void Solver::solve(Armor & armor) const
{
  // 1. 选择匹配的3D特征点（与图像2D角点顺序严格一致，保证PnP匹配精度）
  const auto & object_points =
      (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

  // 2. IPPE-PnP解算相机坐标系下的装甲板位姿
  cv::Vec3d rvec, tvec;  // 输出：旋转向量（轴角表示，3×1）、平移向量（3×1，相机→装甲板，单位米）
  cv::solvePnP(
      object_points,        // 输入：装甲板3D特征点（本地坐标系）
      armor.points,         // 输入：装甲板2D像素角点（图像坐标系）
      camera_matrix_,       // 输入：相机内参矩阵（3×3）
      distort_coeffs_,      // 输入：相机畸变系数（1×5）
      rvec, tvec,           // 输出：旋转向量、平移向量
      false,                // 不使用初始位姿猜测
      cv::SOLVEPNP_IPPE);   // PnP算法类型：IPPE（平面目标最优解）

  // 3. 相机坐标系→云台坐标系：3D坐标转换
  Eigen::Vector3d xyz_in_camera;                // 装甲板在相机坐标系下的3D坐标（Eigen格式）
  cv::cv2eigen(tvec, xyz_in_camera);            // OpenCV Vec3d → Eigen Vector3d（平移向量即3D坐标）
  armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;  // 刚体变换公式

  // 4. 云台坐标系→世界坐标系：3D坐标转换（无平移，仅旋转）
  armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

  // 5. 旋转向量→旋转矩阵：装甲板→相机坐标系（Rodrigues变换，轴角→3×3矩阵）
  cv::Mat rmat;  // OpenCV格式旋转矩阵（3×3）
  cv::Rodrigues(rvec, rmat);
  Eigen::Matrix3d R_armor2camera;                // Eigen格式旋转矩阵
  cv::cv2eigen(rmat, R_armor2camera);            // OpenCV Mat → Eigen Matrix3d

  // 6. 装甲板→云台/世界坐标系：旋转矩阵链式转换（旋转矩阵相乘=姿态叠加）
  Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;  // 相机→云台旋转 × 装甲板→相机旋转
  Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;    // 云台→世界旋转 × 装甲板→云台旋转

  // 7. 旋转矩阵→欧拉角（yaw/pitch/roll，弧度制）
  // 旋转顺序：2→1→0（Z轴→Y轴→X轴，即yaw绕Z、pitch绕Y、roll绕X），符合云台控制角度定义
  // 结果分别存储在云台/世界坐标系下，用于本地控制和全局定位
  armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
  armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);

  // 8. 世界坐标系3D坐标→ypd（方位角/俯仰角/距离）
  // ypd：云台瞄准核心参数，yaw/pitch为云台需要转动的角度，distance为目标直线距离（单位：弧度/米）
  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);

  // 9. 平衡步兵装甲板判断：跳过yaw优化（运动特性导致pitch假设不成立，优化会引入误差）
  // 平衡步兵特征：大型装甲板 + 名称为3/4/5号
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);
  if (is_balance) return;

  // 10. 非平衡步兵装甲板：yaw角优化，减小重投影误差，提升瞄准精度
  optimize_yaw(armor);
}

/**
 * @brief 装甲板重投影：世界坐标系3D位姿→图像2D像素坐标
 * @param xyz_in_world 装甲板中心在世界坐标系下的3D坐标（x,y,z，单位米）
 * @param yaw 装甲板绕世界Z轴的偏航角（弧度制，逆时针为正）
 * @param type 装甲板类型（big/small），选择匹配的3D特征点
 * @param name 装甲板名称（outpost/普通），确定预设俯仰角（±15°）
 * @return std::vector<cv::Point2f> 装甲板4个角点的2D像素坐标（与3D特征点顺序一致）
 * @details 核心流程：
 *          1. 按装甲板类型/名称，确定pitch角（前哨站-15°，普通+15°，角度转弧度）；
 *          2. 基于yaw/pitch构建装甲板→世界坐标系的旋转矩阵（Z-Y顺序旋转，右手坐标系）；
 *          3. 构建装甲板→相机坐标系的旋转/平移向量（逆变换链：世界→云台→相机，转置=逆）；
 *          4. Eigen旋转矩阵→OpenCV旋转向量（Rodrigues变换），适配projectPoints接口；
 *          5. 调用cv::projectPoints，完成3D特征点→2D像素点的透视投影（考虑畸变校正）；
 * @note 逆变换链：旋转矩阵为正交矩阵，逆=转置，避免数值求逆误差，提升计算效率
 * @note 预设pitch角：基于实际装甲板安装姿态标定，提升重投影精度
 */
std::vector<cv::Point2f> Solver::reproject_armor(
  const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const
{
  // 1. 计算yaw角正余弦值（用于构建旋转矩阵）
  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);

  // 2. 确定装甲板预设pitch角（基于业务场景标定，前哨站向下15°，普通向上15°）
  auto pitch = (name == ArmorName::outpost) ? -15.0 * CV_PI / 180.0 : 15.0 * CV_PI / 180.0;
  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  // 3. 构建装甲板→世界坐标系的旋转矩阵（Z-Y顺序旋转，右手坐标系，忽略roll）
  // 矩阵元素由yaw/pitch的正余弦值组合而成，严格遵循欧拉角旋转矩阵推导公式
  // clang-format off
  const Eigen::Matrix3d R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw,          cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw,          sin_yaw * sin_pitch},
    {         -sin_pitch,        0,                    cos_pitch}
  };
  // clang-format on

  // 4. 装甲板→世界坐标系的平移向量（即装甲板中心在世界坐标系的3D坐标）
  const Eigen::Vector3d & t_armor2world = xyz_in_world;

  // 5. 构建装甲板→相机坐标系的旋转矩阵（逆变换链：世界→云台→相机，转置=逆）
  // 旋转链：R_armor2camera = R_camera2gimbal^T * R_gimbal2world^T * R_armor2world
  Eigen::Matrix3d R_armor2camera =
    R_camera2gimbal_.transpose()  // 云台→相机（R_camera2gimbal的逆）
    * R_gimbal2world_.transpose() // 世界→云台（R_gimbal2world的逆）
    * R_armor2world;              // 装甲板→世界（基础旋转）
  
  // 6. 构建装甲板→相机坐标系的平移向量（遵循逆变换链，统一坐标空间后计算）
  Eigen::Vector3d t_armor2camera =
    R_camera2gimbal_.transpose()  // 云台→相机旋转，统一坐标空间
    * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);
    // 括号内：世界坐标系装甲板位置 → 云台坐标系装甲板位置

  // 7. Eigen旋转矩阵→OpenCV旋转向量（Rodrigues变换，适配projectPoints接口）
  cv::Vec3d rvec;                  // 输出：旋转向量（3×1）
  cv::Mat R_armor2camera_cv;       // 临时：Eigen→OpenCV Mat
  cv::eigen2cv(R_armor2camera, R_armor2camera_cv);
  cv::Rodrigues(R_armor2camera_cv, rvec);

  // 8. Eigen平移向量→OpenCV Vec3d（直接赋值，数据类型匹配）
  cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  // 9. 选择匹配的3D特征点（按装甲板类型）
  const auto & object_points = (type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

  // 10. 3D→2D透视投影（考虑相机畸变校正）
  std::vector<cv::Point2f> image_points;  // 输出：2D像素坐标
  cv::projectPoints(
      object_points,  // 输入：3D特征点
      rvec, tvec,     // 输入：相机坐标系下的旋转/平移向量
      camera_matrix_, // 输入：相机内参
      distort_coeffs_,// 输入：畸变系数
      image_points);  // 输出：重投影2D像素点

  return image_points;
}

/**
 * @brief 前哨站装甲板专属重投影误差计算
 * @param armor 检测到的前哨站装甲板对象（含2D像素角点、3D坐标等）
 * @param pitch 给定的俯仰角（弧度制，前哨站专用）
 * @return double 重投影误差（所有角点的实际像素坐标与重投影坐标的欧氏距离之和，单位：像素）
 * @details 1. 重新执行PnP解算，获取装甲板最新位姿（保证坐标准确性）；
 *          2. 完成相机→云台→世界的坐标/旋转矩阵转换；
 *          3. 基于给定pitch和装甲板当前yaw，构建装甲板→世界旋转矩阵；
 *          4. 逆变换得到装甲板→相机的旋转/平移向量，完成3D特征点重投影；
 *          5. 计算实际像素点与重投影点的欧氏距离之和，作为误差指标；
 * @note 误差越小，位姿估计越准确，云台瞄准精度越高
 * @note 前哨站专属：适配前哨站装甲板的特殊安装姿态，提升误差评估的针对性
 */
double Solver::oupost_reprojection_error(Armor armor, const double & pitch)
{
  // 1. 选择匹配的3D特征点
  const auto & object_points =
    (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

  // 2. 重新PnP解算装甲板位姿（保证坐标准确性）
  cv::Vec3d rvec, tvec;
  cv::solvePnP(
    object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);

  // 3. 相机→云台→世界：3D坐标转换
  Eigen::Vector3d xyz_in_camera;
  cv::cv2eigen(tvec, xyz_in_camera);
  armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

  // 4. 旋转向量→旋转矩阵→欧拉角（更新装甲板姿态）
  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  Eigen::Matrix3d R_armor2camera;
  cv::cv2eigen(rmat, R_armor2camera);
  Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
  Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
  armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
  armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);

  // 5. 3D坐标→ypd（更新瞄准参数）
  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);

  // 6. 获取装甲板当前yaw角和世界坐标系3D坐标
  auto yaw = armor.ypr_in_world[0];
  auto xyz_in_world = armor.xyz_in_world;

  // 7. 计算yaw/pitch正余弦值，构建装甲板→世界旋转矩阵
  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);
  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  // clang-format off
  const Eigen::Matrix3d _R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };
  // clang-format on

  // 8. 逆变换得到装甲板→相机的旋转/平移向量
  const Eigen::Vector3d & t_armor2world = xyz_in_world;
  Eigen::Matrix3d _R_armor2camera =
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * _R_armor2world;
  Eigen::Vector3d t_armor2camera =
    R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

  // 9. Eigen→OpenCV旋转向量/平移向量，完成重投影
  cv::Vec3d _rvec;
  cv::Mat R_armor2camera_cv;
  cv::eigen2cv(_R_armor2camera, R_armor2camera_cv);
  cv::Rodrigues(R_armor2camera_cv, _rvec);
  cv::Vec3d _tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  std::vector<cv::Point2f> image_points;
  cv::projectPoints(object_points, _rvec, _tvec, camera_matrix_, distort_coeffs_, image_points);

  // 10. 计算重投影误差：欧氏距离之和
  auto error = 0.0;
  for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
  return error;
}

/**
 * @brief 云台偏航角（yaw）优化：遍历搜索最优yaw角，最小化重投影误差
 * @param armor 输入输出参数：传入解算后的装甲板，优化后更新最优yaw角
 * @details 核心优化逻辑：**网格遍历搜索**（简单高效，适合实时性要求高的场景）
 *          1. 将云台→世界旋转矩阵转换为欧拉角，获取云台当前yaw角；
 *          2. 定义搜索范围±70°（总140°），计算搜索起始yaw角（弧度制，限制在[-π, π]）；
 *          3. 以1°为步长，遍历搜索范围内所有yaw角，计算每个角度的重投影误差；
 *          4. 记录最小误差对应的yaw角，作为最优yaw角；
 *          5. 备份原始yaw角，将最优yaw角更新到装甲板世界坐标系的欧拉角中；
 * @const 方法不修改类内成员，仅修改Armor对象的yaw角
 * @note 搜索步长1°：兼顾优化精度和实时性（140次循环，计算量小，满足帧率要求）
 * @note 角度限制：tools::limit_rad 确保所有角度在[-π, π]范围内，避免角度溢出
 */
void Solver::optimize_yaw(Armor & armor) const
{
  // 1. 获取云台当前世界坐标系下的欧拉角（yaw/pitch/roll）
  Eigen::Vector3d gimbal_ypr = tools::eulers(R_gimbal2world_, 2, 1, 0);

  // 2. 定义搜索参数：范围140°（±70°），步长1°，计算起始yaw角（弧度制）
  constexpr double SEARCH_RANGE = 140;  // 总搜索范围（度）
  auto yaw0 = tools::limit_rad(gimbal_ypr[0] - SEARCH_RANGE / 2 * CV_PI / 180.0);  // 起始角，限制在[-π, π]

  // 3. 初始化最小误差和最优yaw角（初始误差设为极大值，最优角为装甲板当前yaw）
  auto min_error = 1e10;
  auto best_yaw = armor.ypr_in_world[0];

  // 4. 网格遍历搜索：1°步长，遍历所有可能的yaw角
  for (int i = 0; i < SEARCH_RANGE; i++)
  {
    // 计算当前搜索yaw角，限制在[-π, π]
    double yaw = tools::limit_rad(yaw0 + i * CV_PI / 180.0);
    // 计算当前yaw角的重投影误差（inclined为角度偏移量，用于SJTU_cost加权）
    auto error = armor_reprojection_error(armor, yaw, (i - SEARCH_RANGE / 2) * CV_PI / 180.0);
    // 更新最小误差和最优yaw角
    if (error < min_error)
    {
      min_error = error;
      best_yaw = yaw;
    }
  }

  // 5. 备份原始yaw角，更新最优yaw角到装甲板对象
  armor.yaw_raw = armor.ypr_in_world[0];  // 备份原始值，用于后续校验
  armor.ypr_in_world[0] = best_yaw;       // 写入最优yaw角，提升瞄准精度
}

/**
 * @brief SJTU成本函数：上海交通大学定制版重投影误差计算（加权优化）
 * @param cv_refs 实际检测到的装甲板2D像素角点（参考点）
 * @param cv_pts 重投影得到的装甲板2D像素角点（评估点）
 * @param inclined 角度偏移量（弧度制，用于权重调整）
 * @return double 成本函数值：值越小，参考点与评估点匹配度越高
 * @details 对传统欧氏距离误差的**加权优化**，贴合装甲板检测实际场景：
 *          1. 将OpenCV像素点转换为Eigen向量，方便矩阵运算；
 *          2. 遍历每个角点，计算相邻角点的线段向量（ref_d/pt_d）；
 *          3. 计算像素距离误差（起点差+长度差，归一化）和角度距离误差（线段方向差）；
 *          4. 基于inclined的正余弦值加权：像素误差×sin²(inclined) + 角度误差×cos²(inclined)×2；
 *          5. 累加所有角点的加权误差平方根，得到最终成本值；
 * @note 加权逻辑：角度偏移大时，侧重像素误差；角度偏移小时，侧重角度误差，提升鲁棒性
 * @note 归一化处理：消除装甲板距离对误差的影响，保证不同距离下误差评估的一致性
 */
double Solver::SJTU_cost(
  const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
  const double & inclined) const
{
  // 1. 获取点集大小，初始化Eigen向量容器（存储转换后的像素点）
  std::size_t size = cv_refs.size();
  std::vector<Eigen::Vector2d> refs;
  std::vector<Eigen::Vector2d> pts;
  // 2. OpenCV Point2f → Eigen Vector2d（方便后续向量运算）
  for (std::size_t i = 0u; i < size; ++i) {
    refs.emplace_back(cv_refs[i].x, cv_refs[i].y);
    pts.emplace_back(cv_pts[i].x, cv_pts[i].y);
  }

  double cost = 0.;
  // 3. 遍历每个角点，计算线段级误差（i-p为相邻角点，构成装甲板边缘线段）
  for (std::size_t i = 0u; i < size; ++i) {
    std::size_t p = (i + 1u) % size;  // 相邻角点索引（循环取，保证最后一个点与第一个点相连）
    // 4. 计算相邻角点的线段向量（参考线段/评估线段）
    Eigen::Vector2d ref_d = refs[p] - refs[i];
    Eigen::Vector2d pt_d = pts[p] - pts[i];

    // 5. 计算像素距离误差（归一化）：(起点差均值 + 长度差绝对值) / 参考线段长度
    double pixel_dis =
      (0.5 * ((refs[i] - pts[i]).norm() + (refs[p] - pts[p]).norm()) +
       std::fabs(ref_d.norm() - pt_d.norm())) /
      ref_d.norm();
    // 6. 计算角度距离误差：线段方向的绝对角度差（弧度制）
    double angular_dis = tools::get_abs_angle(ref_d, pt_d);

    // 7. 加权计算单段误差：像素误差×sin² + 角度误差×cos²×2（角度误差权重加倍）
    double cost_i =
      tools::square(pixel_dis * std::sin(inclined)) +
      tools::square(angular_dis * std::cos(inclined)) * 2.0;
    // 8. 累加平方根误差，提升小误差的区分度
    cost += std::sqrt(cost_i);
  }
  return cost;
}

/**
 * @brief 通用装甲板重投影误差计算
 * @param armor 检测到的装甲板对象（含世界坐标系3D坐标、类型、名称）
 * @param yaw 待评估的云台偏航角（弧度制）
 * @param inclined 角度偏移量（弧度制，传递给SJTU_cost）
 * @return double 重投影误差（欧氏距离之和，单位：像素）
 * @details 1. 调用reproject_armor，基于待评估yaw角重投影得到2D像素点；
 *          2. 计算实际检测像素点与重投影像素点的欧氏距离之和；
 *          3. 预留SJTU_cost接口（注释行），可按需切换误差计算方式；
 * @note 作为optimize_yaw的**误差评估函数**，为yaw角搜索提供核心指标
 * @note 欧氏距离之和：计算简单、速度快，满足实时优化要求
 */
double Solver::armor_reprojection_error(
  const Armor & armor, double yaw, const double & inclined) const
{
  // 1. 基于待评估yaw角，重投影得到装甲板2D像素点
  auto image_points = reproject_armor(armor.xyz_in_world, yaw, armor.type, armor.name);
  // 2. 计算欧氏距离之和
  auto error = 0.0;
  for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
  // 可选：使用SJTU_cost加权误差（注释行，按需启用）
  // auto error = SJTU_cost(image_points, armor.points, inclined);

  return error;
}

/**
 * @brief 通用接口：世界坐标系3D点→图像2D像素点
 * @param worldPoints 世界坐标系下的3D点集（cv::Point3f，单位米）
 * @return std::vector<cv::Point2f> 对应的2D像素点集（仅保留相机前方的有效点）
 * @details 核心流程：
 *          1. 构建世界→相机坐标系的旋转/平移向量（逆变换链：云台→相机，转置=逆）；
 *          2. 遍历输入3D点，转换为相机坐标系下的点，过滤掉相机后方的点（z≤0，无效）；
 *          3. Eigen→OpenCV格式转换，调用cv::projectPoints完成透视投影；
 *          4. 返回有效点的2D像素坐标，无效点直接过滤；
 * @note 有效性过滤：相机后方的点（z≤0）无法投影到图像，直接丢弃，避免无效计算
 * @note 通用接口：支持任意世界坐标系3D点的投影，不仅限于装甲板特征点
 */
std::vector<cv::Point2f> Solver::world2pixel(const std::vector<cv::Point3f> & worldPoints)
{
  // 1. 构建世界→相机坐标系的旋转矩阵（逆变换链：R_world2camera = R_camera2gimbal^T * R_gimbal2world^T）
  Eigen::Matrix3d R_world2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose();
  // 2. 构建世界→相机坐标系的平移向量
  Eigen::Vector3d t_world2camera = -R_camera2gimbal_.transpose() * t_camera2gimbal_;

  // 3. Eigen→OpenCV格式转换（旋转矩阵/平移向量）
  cv::Mat rvec, tvec;
  cv::eigen2cv(R_world2camera, rvec);
  cv::eigen2cv(t_world2camera, tvec);

  // 4. 过滤有效3D点：仅保留相机前方的点（camera_point.z > 0）
  std::vector<cv::Point3f> valid_world_points;
  for (const auto & world_point : worldPoints) {
    // 世界→相机坐标系3D点转换
    Eigen::Vector3d world_point_eigen(world_point.x, world_point.y, world_point.z);
    Eigen::Vector3d camera_point = R_world2camera * world_point_eigen + t_world2camera;
    // 过滤相机后方的点
    if (camera_point.z() > 0) {
      valid_world_points.push_back(world_point);
    }
  }

  // 5. 无有效点，返回空向量
  if (valid_world_points.empty()) {
    return std::vector<cv::Point2f>();
  }

  // 6. 有效3D点→2D像素点投影（考虑畸变校正）
  std::vector<cv::Point2f> pixelPoints;
  cv::projectPoints(valid_world_points, rvec, tvec, camera_matrix_, distort_coeffs_, pixelPoints);

  return pixelPoints;
}

}  // namespace auto_aim
