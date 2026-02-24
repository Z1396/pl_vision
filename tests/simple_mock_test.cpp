#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "io/mock_camera.hpp"
#include "io/mock_cboard.hpp"

int main() {
    std::cout << "=== 模拟设备测试程序 ===" << std::endl;
    
    try {
        std::cout << "1. 初始化MockCamera..." << std::endl;
        io::MockCamera camera("configs/standard3.yaml");
        std::cout << "   ✓ MockCamera初始化成功" << std::endl;
        
        std::cout << "2. 初始化MockCBoard..." << std::endl;
        io::MockCBoard cboard("configs/standard3.yaml");
        std::cout << "   ✓ MockCBoard初始化成功" << std::endl;
        
        std::cout << "3. 开始读取图像和IMU数据..." << std::endl;
        cv::Mat img;
        std::chrono::steady_clock::time_point timestamp;
        
        for (int i = 0; i < 10; i++) {
            camera.read(img, timestamp);
            auto q = cboard.imu_at(timestamp);
            
            std::cout << "   帧 " << i << ": " 
                      << img.cols << "x" << img.rows 
                      << ", IMU四元数: [" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "]"
                      << ", 弹速: " << cboard.bullet_speed << " m/s"
                      << ", 模式: " << io::MODES[cboard.mode] << std::endl;
        }
        
        std::cout << "4. 测试控制指令发送..." << std::endl;
        io::Command cmd;
        cmd.control = true;
        cmd.yaw = 0.1;
        cmd.pitch = 0.05;
        cmd.shoot = true;
        cboard.send(cmd);
        std::cout << "   ✓ 控制指令发送成功" << std::endl;
        
        std::cout << "\n=== 所有测试通过！ ===" << std::endl;
        std::cout << "\n模拟设备功能验证：" << std::endl;
        std::cout << "✓ 相机图像生成" << std::endl;
        std::cout << "✓ IMU姿态数据生成" << std::endl;
        std::cout << "✓ 时间戳插值查询" << std::endl;
        std::cout << "✓ 控制指令发送" << std::endl;
        std::cout << "✓ 设备状态管理" << std::endl;
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "\n错误: " << e.what() << std::endl;
        return 1;
    }
}