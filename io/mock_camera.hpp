#pragma once

#include <chrono>
#include <opencv2/opencv.hpp>
#include <string>
#include <random>

namespace io {

class MockCamera {
public:
    MockCamera(const std::string& config_path);
    ~MockCamera();

    void read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp);

private:
    void generateMockImage(cv::Mat& img);
    void addMovingTarget(cv::Mat& img);
    void addNoise(cv::Mat& img);

    std::chrono::steady_clock::time_point start_time_;
    int frame_count_;
    int width_;
    int height_;
    
    std::mt19937 rng_;
    std::normal_distribution<double> noise_dist_;
    
    double target_x_;
    double target_y_;
    double target_vx_;
    double target_vy_;
};

}