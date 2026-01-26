#include "solver.hpp"

#include <yaml-cpp/yaml.h>

#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
constexpr double LIGHTBAR_LENGTH = 56e-3;     // m
constexpr double BIG_ARMOR_WIDTH = 230e-3;    // m
constexpr double SMALL_ARMOR_WIDTH = 135e-3;  // m

const std::vector<cv::Point3f> BIG_ARMOR_POINTS{
  {0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};
  
const std::vector<cv::Point3f> SMALL_ARMOR_POINTS{
  {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};

// Solver类的构造函数，接收配置文件路径作为参数
// 成员初始化列表：将R_gimbal2world_（云台到世界坐标系的旋转矩阵）初始化为3×3单位矩阵
// Eigen::Matrix3d::Identity()：生成3×3单位矩阵（对角线为1，其余为0），确保初始旋转无偏移
Solver::Solver(const std::string & config_path) : R_gimbal2world_(Eigen::Matrix3d::Identity())  
{
  // 1. 加载YAML配置文件：通过YAML库读取指定路径的配置文件，返回yaml节点对象（可通过键值访问配置项）
  auto yaml = YAML::LoadFile(config_path);

  // 2. 读取坐标系间的旋转矩阵配置（云台→IMU机体、相机→云台）和相机平移向量配置（相机→云台）
  // 从yaml节点中按键提取数据，转换为double类型的vector（存储矩阵的扁平化数据）
  auto R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();  // 云台到IMU机体的旋转矩阵数据
  auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();    // 相机到云台的旋转矩阵数据
  auto t_camera2gimbal_data = yaml["t_camera2gimbal"].as<std::vector<double>>();    // 相机到云台的平移向量数据（3个元素：x,y,z）

  // 3. 将vector扁平化数据转换为Eigen矩阵/向量（RowMajor表示按行存储，匹配常见的配置文件数据格式）
  // 初始化云台到IMU机体的旋转矩阵（3×3）：用vector的首地址作为数据源，按行构造矩阵
  R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());
  // 初始化相机到云台的旋转矩阵（3×3）
  R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
  // 初始化相机到云台的平移向量（3×1列向量）
  t_camera2gimbal_ = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());

  // 4. 读取相机内参和畸变系数配置
  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();    // 相机内参矩阵数据（3×3，如f_x, 0, c_x; 0, f_y, c_y; 0,0,1）
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();  // 相机畸变系数数据（通常5个元素：k1,k2,p1,p2,k3）

  // 5. 转换相机内参和畸变系数为Eigen矩阵
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());  // 3×3相机内参矩阵
  Eigen::Matrix<double, 1, 5> distort_coeffs(distort_coeffs_data.data());                // 1×5畸变系数行向量

  // 6. 将Eigen矩阵转换为OpenCV的cv::Mat格式（用于后续OpenCV图像处理函数）
  cv::eigen2cv(camera_matrix, camera_matrix_);    // 内参矩阵：Eigen→cv::Mat，结果存入成员变量camera_matrix_
  cv::eigen2cv(distort_coeffs, distort_coeffs_);  // 畸变系数：Eigen→cv::Mat，结果存入成员变量distort_coeffs_
  
  /*补充说明cv::eigen2cv：
    输入：camera_matrix（Eigen矩阵/向量，如Eigen::Matrix3d）
    输出：camera_matrix_（OpenCV的cv::Mat对象，数据类型、尺寸与输入完全匹配）
    核心作用：打通Eigen（擅长矩阵运算、坐标变换）与OpenCV（擅长图像处理、相机标定）的数据交互，
              避免手动循环复制数据的繁琐操作，同时保证数据准确性*/
}

Eigen::Matrix3d Solver::R_gimbal2world() const { return R_gimbal2world_; }

void Solver::set_R_gimbal2world(const Eigen::Quaterniond & q)  //Eigen::Quaterniond 是用于表示 双精度浮点数（double）类型四元数 的核心类
{
  Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();    //由四元数q转换而来的旋转矩阵
  R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;   //transpose()转置
}

//solvePnP（获得姿态）
/**
 * @brief 求解装甲板的空间位姿与坐标转换
 * 
 * 该函数是视觉定位的核心计算接口，通过相机内参、畸变系数和PnP算法求解装甲板在相机坐标系下的位姿，
 * 并完成从相机坐标系到云台坐标系、世界坐标系的坐标转换，同时计算装甲板的欧拉角（yaw/pitch/roll）
 * 及方位角-俯仰角-距离（ypd）信息，最后对非平衡步兵类型的装甲板进行yaw角优化。
 * 
 * @param armor 待求解的装甲板对象（引用传递，计算结果会写入该对象的成员变量）
 */
void Solver::solve(Armor & armor) const
{
    // 1. 选择装甲板的3D模型点（根据装甲板类型选择大/小装甲板的预设三维点）
    // BIG_ARMOR_POINTS/SMALL_ARMOR_POINTS：预定义的装甲板角点3D坐标（单位通常为米），基于装甲板实际尺寸
    const auto & object_points =
        (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

    // 2. 调用PnP算法求解装甲板在相机坐标系下的位姿（旋转向量rvec + 平移向量tvec）
    cv::Vec3d rvec, tvec;  // rvec：旋转向量（轴角表示），tvec：平移向量（相机到装甲板的位置）
    cv::solvePnP(
        object_points,        // 装甲板3D模型点
        armor.points,         // 装甲板在图像中的2D像素点
        camera_matrix_,       // 相机内参矩阵（fx, fy, cx, cy等）
        distort_coeffs_,      // 相机畸变系数（k1, k2, p1, p2, k3等）
        rvec, tvec,           // 输出：旋转向量、平移向量
        false,                // 是否使用初始猜测（此处为false，不使用）
        cv::SOLVEPNP_IPPE);   // PnP求解器类型（IPPE算法，适用于单目相机，鲁棒性强）

    // 3. 坐标转换：从相机坐标系转换到云台坐标系
    Eigen::Vector3d xyz_in_camera;  // 装甲板在相机坐标系下的3D坐标
    cv::cv2eigen(tvec, xyz_in_camera);  // OpenCV的Vec3d（tvec）转换为Eigen的Vector3d
    // 利用相机到云台的旋转矩阵（R_camera2gimbal_）和平移向量（t_camera2gimbal_）完成坐标变换
    armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;

    // 4. 坐标转换：从云台坐标系转换到世界坐标系
    // 利用云台到世界的旋转矩阵（R_gimbal2world_）完成变换（假设云台到世界无平移，或平移已包含在R中）
    armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

    // 5. 求解装甲板的旋转矩阵（从装甲板坐标系到各目标坐标系）
    cv::Mat rmat;  // 旋转矩阵（3×3，OpenCV类型）
    cv::Rodrigues(rvec, rmat);  // 将旋转向量rvec转换为旋转矩阵rmat（轴角→矩阵）
    Eigen::Matrix3d R_armor2camera;  // 装甲板到相机的旋转矩阵（Eigen类型）
    cv::cv2eigen(rmat, R_armor2camera);  // OpenCV的Mat（rmat）转换为Eigen的Matrix3d

    // 装甲板到云台的旋转矩阵 = 相机到云台的旋转矩阵 × 装甲板到相机的旋转矩阵
    Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
    // 装甲板到世界的旋转矩阵 = 云台到世界的旋转矩阵 × 装甲板到云台的旋转矩阵
    Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;

    // 6. 旋转矩阵转换为欧拉角（yaw-pitch-roll，偏航-俯仰-横滚）
    // 旋转顺序为2→1→0（对应Z轴→Y轴→X轴，即yaw→pitch→roll），符合云台控制的角度定义
    armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);  // 云台坐标系下的欧拉角
    armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);    // 世界坐标系下的欧拉角

    // 7. 计算世界坐标系下的方位角-俯仰角-距离（ypd）
    // xyz2ypd：将3D坐标（x,y,z）转换为yaw（水平方位角）、pitch（垂直俯仰角）、distance（直线距离）
    // 结果用于云台瞄准控制（直接指导云台转动到目标方向）
    armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);

    // 8. 特殊判断：平衡步兵装甲板不进行yaw角优化（因平衡步兵运动特性，pitch角假设不成立，优化会引入误差）
    auto is_balance = (armor.type == ArmorType::big) &&  // 大装甲板
                      (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                       armor.name == ArmorName::five);   // 平衡步兵专属装甲板名称
    if (is_balance) return;  // 平衡步兵直接返回，跳过yaw优化

    // 9. 对非平衡步兵类型的装甲板进行yaw角优化（提升yaw角估计精度，优化瞄准效果）
    optimize_yaw(armor);
}

/**
 * @brief 将世界坐标系下的装甲板3D位姿，投影到相机图像平面，输出2D像素坐标
 * @param xyz_in_world 装甲板中心在世界坐标系中的3D坐标 (x, y, z)
 * @param yaw 装甲板绕世界坐标系Z轴的水平旋转角（弧度制，逆时针为正方向）
 * @param type 装甲板尺寸类型（big/小small），决定使用的3D特征点集合
 * @param name 装甲板名称（outpost/普通），决定俯仰角方向（±15°）
 * @return 装甲板4个角点在图像上的2D像素坐标（顺序与3D特征点一一对应）
 */
std::vector<cv::Point2f> Solver::reproject_armor(
  const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const
{
  // 计算yaw角（水平旋转）的正余弦值，用于构建旋转矩阵
  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);

  // 确定装甲板俯仰角pitch：前哨站(outpost)装甲板向下15°，普通装甲板向上15°（角度转弧度）
  auto pitch = (name == ArmorName::outpost) ? -15.0 * CV_PI / 180.0 : 15.0 * CV_PI / 180.0;
  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  // clang-format off
  // 构建「装甲板坐标系 → 世界坐标系」的旋转矩阵R_armor2world
  // 欧拉角顺序：Z轴(yaw)→Y轴(pitch)→X轴(无旋转)，右手坐标系
  // 作用：将装甲板本地坐标系下的点，旋转到世界坐标系
  const Eigen::Matrix3d R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw,          cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw,          sin_yaw * sin_pitch},
    {         -sin_pitch,        0,                    cos_pitch}
  };
  // clang-format on

  // 装甲板在世界坐标系的平移向量 = 输入的装甲板中心世界坐标
  const Eigen::Vector3d & t_armor2world = xyz_in_world;

  // 计算「装甲板坐标系 → 相机坐标系」的旋转矩阵R和平移向量t
  // 旋转链：装甲板→世界→云台→相机（旋转矩阵逆=转置，避免求逆误差，提升效率）
  Eigen::Matrix3d R_armor2camera =
    R_camera2gimbal_.transpose()  // 云台→相机的旋转（R_camera2gimbal的逆）
    * R_gimbal2world_.transpose() // 世界→云台的旋转（R_gimbal2world的逆）
    * R_armor2world;              // 装甲板→世界的旋转（基础旋转）
  
  // 平移链：世界坐标系装甲板位置 → 云台坐标系 → 相机坐标系（遵循旋转链顺序推导）
  Eigen::Vector3d t_armor2camera =
    R_camera2gimbal_.transpose()  // 云台→相机的旋转（统一坐标空间）
    * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);
    // 括号内：世界坐标系装甲板位置 → 云台坐标系装甲板位置

  // 将Eigen旋转矩阵转换为OpenCV支持的旋转向量（Rodrigues变换：3x3矩阵→3x1向量）
  cv::Vec3d rvec;                  // 相机坐标系下的旋转向量（输入projectPoints）
  cv::Mat R_armor2camera_cv;       // Eigen矩阵转OpenCV Mat格式
  cv::eigen2cv(R_armor2camera, R_armor2camera_cv);
  cv::Rodrigues(R_armor2camera_cv, rvec);

  // 平移向量格式转换：Eigen::Vector3d → OpenCV Vec3d
  cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  // 3D→2D投影：将装甲板3D特征点投影到图像平面
  std::vector<cv::Point2f> image_points;  // 输出：图像平面2D像素坐标
  // 根据装甲板类型选择对应的3D特征点（装甲板本地坐标系下的角点定义）
  const auto & object_points = (type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;
  // OpenCV透视投影函数：考虑相机内参和畸变，输出2D像素坐标
  cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distort_coeffs_, image_points);

  return image_points;
}

/**
 * @brief 计算装甲板的重投影误差，用于评估位姿估计的准确性
 * @param armor 装甲板对象，包含其在图像中的特征点等信息
 * @param pitch 给定的俯仰角，用于构建装甲板到世界坐标系的旋转矩阵
 * @return double 重投影误差，即装甲板特征点实际图像坐标与重投影坐标的欧氏距离之和
 */
double Solver::oupost_reprojection_error(Armor armor, const double & pitch)
{
  // 根据装甲板类型选择对应的三维特征点（大装甲板或小装甲板的标准尺寸点）
  const auto & object_points =
    (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

  // 声明旋转向量和平移向量，用于存储PNP求解结果
  cv::Vec3d rvec, tvec;
  // 使用IPPE算法求解PNP问题，得到装甲板在相机坐标系下的旋转和平移
  // 参数：三维特征点、图像特征点、相机内参、畸变系数、输出旋转向量、输出平移向量、是否使用初始值、求解方法
  cv::solvePnP(
    object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);

  // 将平移向量从OpenCV格式转换为Eigen格式，得到装甲板在相机坐标系下的三维坐标
  Eigen::Vector3d xyz_in_camera;
  cv::cv2eigen(tvec, xyz_in_camera);
  // 将相机坐标系下的坐标转换到云台坐标系（相机到云台的旋转和平移变换）
  armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  // 将云台坐标系下的坐标转换到世界坐标系（云台到世界的旋转变换，假设无平移）
  armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

  // 将旋转向量转换为旋转矩阵（Rodrigues公式）
  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  // 旋转矩阵格式转换（OpenCV->Eigen），得到装甲板到相机的旋转矩阵
  Eigen::Matrix3d R_armor2camera;
  cv::cv2eigen(rmat, R_armor2camera);
  // 计算装甲板到云台的旋转矩阵（相机到云台的旋转 × 装甲板到相机的旋转）
  Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
  // 计算装甲板到世界的旋转矩阵（云台到世界的旋转 × 装甲板到云台的旋转）
  Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
  // 将旋转矩阵转换为欧拉角（yaw-pitch-roll顺序），存储在装甲板对象中
  armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
  armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);

  // 将世界坐标系下的三维坐标转换为yaw-pitch-distance（方位角-俯仰角-距离）
  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);

  // 获取世界坐标系下的偏航角和三维坐标
  auto yaw = armor.ypr_in_world[0];
  auto xyz_in_world = armor.xyz_in_world;

  // 计算偏航角和俯仰角的正余弦值（用于构建旋转矩阵）
  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);
  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  // 构建装甲板到世界坐标系的旋转矩阵（基于yaw和pitch的旋转，忽略roll）
  // 旋转顺序：先绕z轴（yaw），再绕y轴（pitch）
  // clang-format off
  const Eigen::Matrix3d _R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };
  // clang-format on

  // 定义装甲板到世界坐标系的平移向量（即世界坐标系下的装甲板坐标）
  const Eigen::Vector3d & t_armor2world = xyz_in_world;
  // 计算装甲板到相机的旋转矩阵：
  // 世界到云台的旋转逆（转置） × 云台到相机的旋转逆（转置） × 装甲板到世界的旋转
  Eigen::Matrix3d _R_armor2camera =
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * _R_armor2world;
  // 计算装甲板到相机的平移向量：
  // 云台到相机的旋转逆 × [世界到云台的旋转逆 × 装甲板到世界的平移 - 相机到云台的平移]
  Eigen::Vector3d t_armor2camera =
    R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

  // 将旋转矩阵转换为旋转向量（用于重投影）
  cv::Vec3d _rvec;
  cv::Mat R_armor2camera_cv;
  cv::eigen2cv(_R_armor2camera, R_armor2camera_cv);
  cv::Rodrigues(R_armor2camera_cv, _rvec);
  // 将平移向量转换为OpenCV格式
  cv::Vec3d _tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  // 使用计算出的旋转向量和平移向量，将三维特征点重投影到图像平面
  std::vector<cv::Point2f> image_points;
  cv::projectPoints(object_points, _rvec, _tvec, camera_matrix_, distort_coeffs_, image_points);

  // 计算重投影误差：所有特征点的实际图像坐标与重投影坐标的欧氏距离之和
  auto error = 0.0;
  for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
  return error;
}


/**
 * @brief 优化目标装甲板的偏航角（yaw），通过搜索最优角度减小投影误差
 * @param armor 待优化的装甲板对象（包含世界坐标系下的姿态信息）
 */
void Solver::optimize_yaw(Armor & armor) const
{
  // 1. 获取当前云台在世界坐标系下的姿态角（yaw, pitch, roll）
  //    调用工具函数将"云台→世界"的旋转矩阵转换为欧拉角
  //    参数(2,1,0)表示采用Z-Y-X顺序（yaw绕Z轴，pitch绕Y轴，roll绕X轴），默认内旋模式
  Eigen::Vector3d gimbal_ypr = tools::eulers(R_gimbal2world_, 2, 1, 0);

  // 2. 定义搜索范围（140度），计算搜索起始角度
  //    将角度从度转换为弧度，并用limit_rad限制在[-π, π]范围内
  constexpr double SEARCH_RANGE = 140;  // 搜索范围：±70度（总140度）
  auto yaw0 = tools::limit_rad(gimbal_ypr[0] - SEARCH_RANGE / 2 * CV_PI / 180.0);

  // 3. 初始化误差最小值和最优yaw角（初始值设为装甲板当前的yaw角）
  auto min_error = 1e10;         // 初始误差设为一个很大的值
  auto best_yaw = armor.ypr_in_world[0];  // 最优yaw角初始值

  // 4. 在搜索范围内遍历每个可能的yaw角，寻找投影误差最小的角度
  //    步长为1度（循环140次，覆盖整个搜索范围）
  for (int i = 0; i < SEARCH_RANGE; i++) 
  {
    // 计算当前搜索的yaw角（从起始角开始，每次增加1度的弧度值）
    double yaw = tools::limit_rad(yaw0 + i * CV_PI / 180.0);
    
    // 计算该yaw角对应的装甲板投影误差
    // 第二个参数为当前搜索的yaw角，第三个参数为相对于中心的偏移量（用于权重调整）
    auto error = armor_reprojection_error(armor, yaw, (i - SEARCH_RANGE / 2) * CV_PI / 180.0);

    // 更新最小误差和最优yaw角：若当前误差更小，则记录该角度
    if (error < min_error) 
    {
      min_error = error;
      best_yaw = yaw;
    }
  }

  // 5. 保存原始yaw角，并用最优yaw角更新装甲板的世界坐标系姿态
  armor.yaw_raw = armor.ypr_in_world[0];  // 备份原始yaw角
  armor.ypr_in_world[0] = best_yaw;       // 更新为最优yaw角
}

double Solver::SJTU_cost(
  const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
  const double & inclined) const
{
  std::size_t size = cv_refs.size();
  std::vector<Eigen::Vector2d> refs;
  std::vector<Eigen::Vector2d> pts;
  for (std::size_t i = 0u; i < size; ++i) {
    refs.emplace_back(cv_refs[i].x, cv_refs[i].y);
    pts.emplace_back(cv_pts[i].x, cv_pts[i].y);
  }
  double cost = 0.;
  for (std::size_t i = 0u; i < size; ++i) {
    std::size_t p = (i + 1u) % size;
    // i - p 构成线段。过程：先移动起点，再补长度，再旋转
    Eigen::Vector2d ref_d = refs[p] - refs[i];  // 标准
    Eigen::Vector2d pt_d = pts[p] - pts[i];
    // 长度差代价 + 起点差代价(1 / 2)（0 度左右应该抛弃)
    double pixel_dis =  // dis 是指方差平面内到原点的距离
      (0.5 * ((refs[i] - pts[i]).norm() + (refs[p] - pts[p]).norm()) +
       std::fabs(ref_d.norm() - pt_d.norm())) /
      ref_d.norm();
    double angular_dis = ref_d.norm() * tools::get_abs_angle(ref_d, pt_d) / ref_d.norm();
    // 平方可能是为了配合 sin 和 cos
    // 弧度差代价（0 度左右占比应该大）
    double cost_i =
      tools::square(pixel_dis * std::sin(inclined)) +
      tools::square(angular_dis * std::cos(inclined)) * 2.0;  // DETECTOR_ERROR_PIXEL_BY_SLOPE
    // 重投影像素误差越大，越相信斜率
    cost += std::sqrt(cost_i);
  }
  return cost;
}

double Solver::armor_reprojection_error(
  const Armor & armor, double yaw, const double & inclined) const
{
  auto image_points = reproject_armor(armor.xyz_in_world, yaw, armor.type, armor.name);
  auto error = 0.0;
  for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
  // auto error = SJTU_cost(image_points, armor.points, inclined);

  return error;
}

// 世界坐标到像素坐标的转换
std::vector<cv::Point2f> Solver::world2pixel(const std::vector<cv::Point3f> & worldPoints)
{
  Eigen::Matrix3d R_world2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose();
  Eigen::Vector3d t_world2camera = -R_camera2gimbal_.transpose() * t_camera2gimbal_;

  cv::Mat rvec;
  cv::Mat tvec;
  cv::eigen2cv(R_world2camera, rvec);
  cv::eigen2cv(t_world2camera, tvec);

  std::vector<cv::Point3f> valid_world_points;
  for (const auto & world_point : worldPoints) {
    Eigen::Vector3d world_point_eigen(world_point.x, world_point.y, world_point.z);
    Eigen::Vector3d camera_point = R_world2camera * world_point_eigen + t_world2camera;

    if (camera_point.z() > 0) {
      valid_world_points.push_back(world_point);
    }
  }
  // 如果没有有效点，返回空vector
  if (valid_world_points.empty()) {
    return std::vector<cv::Point2f>();
  }
  std::vector<cv::Point2f> pixelPoints;
  cv::projectPoints(valid_world_points, rvec, tvec, camera_matrix_, distort_coeffs_, pixelPoints);
  return pixelPoints;
}
}  // namespace auto_aim