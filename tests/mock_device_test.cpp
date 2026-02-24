#include <iostream>
#include "io/mock_camera.hpp"
#include "io/mock_cboard.hpp"
#include <opencv2/opencv.hpp>

int main() {
    std::cout << "Testing MockCamera and MockCBoard..." << std::endl;
    
    try {
        std::cout << "Creating MockCamera..." << std::endl;
        io::MockCamera camera("configs/standard3.yaml");
        std::cout << "MockCamera created successfully!" << std::endl;
        
        std::cout << "Creating MockCBoard..." << std::endl;
        io::MockCBoard cboard("configs/standard3.yaml");
        std::cout << "MockCBoard created successfully!" << std::endl;
        
        cv::Mat img;
        std::chrono::steady_clock::time_point timestamp;
        
        std::cout << "Reading 5 frames from MockCamera..." << std::endl;
        for (int i = 0; i < 5; i++) {
            camera.read(img, timestamp);
            auto q = cboard.imu_at(timestamp);
            
            std::cout << "Frame " << i << ": " << img.cols << "x" << img.rows 
                      << ", IMU: [" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "]" << std::endl;
        }
        
        std::cout << "Test completed successfully!" << std::endl;
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}