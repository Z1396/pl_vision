/**
 * @file img_tools.cpp
 * @brief 视觉工具模块 - 图像绘制工具函数实现
 * @details 封装OpenCV基础绘制接口，提供**单点、多点、浮点坐标点、文字**的便捷绘制函数，
 *          是机器视觉调试、结果可视化的核心工具——将检测/解算结果（如装甲板角点、目标坐标、参数信息）
 *          绘制到图像上，便于开发调试、效果验证与实时监控。
 * @note 核心依赖：OpenCV3/4 核心模块（cv::Mat、cv::Point、绘制接口等）
 * @note 设计原则：1. 重载适配不同数据类型（cv::Point/cv::Point2f）；2. 简化OpenCV原生接口参数；
 *                3. 原地修改图像（引用传递），无额外内存拷贝；4. 兼容彩色/灰度图像绘制。
 */
#include "img_tools.hpp"

// 工具模块命名空间，隔离工具函数，避免与业务代码命名冲突
namespace tools
{

/**
 * @brief 绘制单个实心点（圆形）
 * @param img 输入输出参数：待绘制的图像（cv::Mat& 引用传递，原地修改）
 * @param point 绘制位置：图像像素坐标点（cv::Point，整型坐标）
 * @param color 绘制颜色：BGR格式颜色值（cv::Scalar，如红色cv::Scalar(0,0,255)）
 * @param radius 圆形半径：像素单位，默认可设为2~5（根据图像分辨率调整）
 * @details 封装OpenCV cv::circle接口，固定绘制**实心圆**，简化参数传递，
 *          适用于绘制特征点、目标中心、角点等单个标记点。
 * @note 颜色格式：OpenCV默认BGR（蓝绿红），与RGB相反，绘制时需注意颜色顺序；
 * @note 原地修改：图像为引用传递，函数执行后原图像会被修改，无需返回新图像。
 */
void draw_point(cv::Mat & img, const cv::Point & point, const cv::Scalar & color, int radius)
{
  // cv::circle：OpenCV绘制圆形接口
  // 参数：图像、圆心、半径、颜色、线宽（-1表示实心填充）
  cv::circle(img, point, radius, color, -1);
}

/**
 * @brief 绘制多点轮廓（连接成闭合/开放曲线）
 * @param img 输入输出参数：待绘制的图像（cv::Mat& 引用传递，原地修改）
 * @param points 绘制点集：整型像素坐标点集合（std::vector<cv::Point>）
 * @param color 绘制颜色：BGR格式颜色值（cv::Scalar）
 * @param thickness 线条宽度：像素单位，默认1~3，值越大线条越粗
 * @details 封装OpenCV cv::drawContours接口，将点集转换为单轮廓格式，
 *          快速绘制点集的连接曲线，适用于绘制装甲板角点、目标轮廓、检测框等连续特征。
 * @note 轮廓特性：点集按传入顺序依次连接，最后一个点会与第一个点闭合（轮廓特性）；
 * @note 适用场景：绘制多边形轮廓（如装甲板四个角点）、特征点轨迹等。
 */
void draw_points(
  cv::Mat & img, const std::vector<cv::Point> & points, const cv::Scalar & color, int thickness)
{
  // 将单点集转换为cv::drawContours要求的轮廓格式（vector<vector<cv::Point>>）
  std::vector<std::vector<cv::Point>> contours = {points};
  
  // cv::drawContours：OpenCV绘制轮廓接口
  // 参数：图像、轮廓集、轮廓索引（-1表示绘制所有轮廓）、颜色、线条宽度
  cv::drawContours(img, contours, -1, color, thickness);
}

/**
 * @brief 绘制浮点型坐标点集（重载适配，自动转换为整型）
 * @param img 输入输出参数：待绘制的图像（cv::Mat& 引用传递，原地修改）
 * @param points 绘制点集：浮点型像素坐标点集合（std::vector<cv::Point2f>）
 * @param color 绘制颜色：BGR格式颜色值（cv::Scalar）
 * @param thickness 线条宽度：像素单位
 * @details 为浮点型坐标点集提供的**重载函数**，解决视觉解算中浮点坐标（如PnP结果、重投影点）
 *          无法直接绘制的问题，自动完成cv::Point2f → cv::Point的类型转换（浮点取整），
 *          调用底层整型点集绘制函数，保证接口一致性。
 * @note 类型转换：浮点坐标会被直接截断取整（如(100.9, 200.5)→(100,200)），适用于像素级绘制；
 * @note 接口统一：与整型点集绘制函数同名，根据参数类型自动匹配，简化调用代码。
 */
void draw_points(
  cv::Mat & img, const std::vector<cv::Point2f> & points, const cv::Scalar & color, int thickness)
{
  // 浮点型点集 → 整型点集转换：利用迭代器直接构造，简洁高效
  std::vector<cv::Point> int_points(points.begin(), points.end());
  
  // 调用底层整型点集绘制函数，复用绘制逻辑，避免代码冗余
  draw_points(img, int_points, color, thickness);
}

/**
 * @brief 在图像上绘制文字（标注参数、信息、结果）
 * @param img 输入输出参数：待绘制的图像（cv::Mat& 引用传递，原地修改）
 * @param text 绘制文字内容：任意字符串（如"Armor: big"、"Yaw: 0.5rad"）
 * @param point 文字绘制起始位置：像素坐标点（cv::Point，文字左下角为基准点）
 * @param color 文字颜色：BGR格式颜色值（cv::Scalar）
 * @param font_scale 字体缩放比例：默认1.0，值越大字体越大
 * @param thickness 文字线条宽度：像素单位，默认1~2，值越大文字越粗
 * @details 封装OpenCV cv::putText接口，使用常用的无衬线字体，简化文字绘制参数，
 *          适用于在图像上标注目标信息、解算参数、调试日志等文本内容。
 * @note 字体选择：cv::FONT_HERSHEY_SIMPLEX 为OpenCV默认无衬线字体，清晰易读，适配实时绘制；
 * @note 基准点：文字绘制位置为**左下角**，而非左上角，标注时需注意坐标调整；
 * @note 字符支持：默认仅支持ASCII字符（数字、字母、符号），不支持中文（需额外加载字体）。
 */
void draw_text(
  cv::Mat & img, const std::string & text, const cv::Point & point, const cv::Scalar & color,
  double font_scale, int thickness)
{
  // cv::putText：OpenCV绘制文字接口
  // 参数：图像、文字内容、起始点、字体类型、缩放比例、颜色、线条宽度
  cv::putText(img, text, point, cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness);
}

}  // namespace tools
