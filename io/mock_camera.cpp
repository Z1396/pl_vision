#include "mock_camera.hpp"
#include <tools/logger.hpp>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace io {

MockCamera::MockCamera(const std::string& config_path)
    : start_time_(std::chrono::steady_clock::now())
    , frame_count_(0)
    , width_(1280)
    , height_(720)
    , rng_(std::random_device{}())
    , noise_dist_(0.0, 10.0)
    , target_x_(width_ * 0.3)
    , target_y_(height_ * 0.5)
    , target_vx_(2.0)
    , target_vy_(1.5) {
    
    std::ifstream config_file(config_path);
    if (!config_file.good()) {
        tools::logger()->warn("Config file not found or not readable: {}, using defaults", config_path);
    } else {
        try {
            YAML::Node config = YAML::LoadFile(config_path);
            if (config["camera"]) {
                if (config["camera"]["width"]) {
                    width_ = config["camera"]["width"].as<int>();
                }
                if (config["camera"]["height"]) {
                    height_ = config["camera"]["height"].as<int>();
                }
            }
        } catch (const YAML::Exception& e) {
            tools::logger()->warn("YAML parsing error in config file: {}, using defaults", e.what());
        } catch (const std::exception& e) {
            tools::logger()->warn("Failed to load camera config: {}, using defaults", e.what());
        }
    }
    
    tools::logger()->info("MockCamera initialized: {}x{}", width_, height_);
}

MockCamera::~MockCamera() {
    tools::logger()->info("MockCamera destroyed. Total frames: {}", frame_count_);
}

void MockCamera::read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) {
    timestamp = std::chrono::steady_clock::now();
    
    img.create(height_, width_, CV_8UC3);
    generateMockImage(img);
    addMovingTarget(img);
    addNoise(img);
    
    frame_count_++;
}

void MockCamera::generateMockImage(cv::Mat& img) {
    img.setTo(cv::Scalar(20, 20, 30));
    
    for (int y = 0; y < img.rows; y++) {
        for (int x = 0; x < img.cols; x++) {
            double noise_x = noise_dist_(rng_);
            double noise_y = noise_dist_(rng_);
            double noise_z = noise_dist_(rng_);
            
            img.at<cv::Vec3b>(y, x)[0] = cv::saturate_cast<uchar>(20 + noise_z);
            img.at<cv::Vec3b>(y, x)[1] = cv::saturate_cast<uchar>(20 + noise_y);
            img.at<cv::Vec3b>(y, x)[2] = cv::saturate_cast<uchar>(30 + noise_x);
        }
    }
    
    cv::circle(img, cv::Point(width_ / 2, height_ / 2), 5, cv::Scalar(0, 255, 0), -1);
    cv::line(img, cv::Point(width_ / 2 - 20, height_ / 2), cv::Point(width_ / 2 + 20, height_ / 2), cv::Scalar(0, 255, 0), 2);
    cv::line(img, cv::Point(width_ / 2, height_ / 2 - 20), cv::Point(width_ / 2, height_ / 2 + 20), cv::Scalar(0, 255, 0), 2);
    
    cv::rectangle(img, cv::Rect(50, 50, 100, 50), cv::Scalar(255, 0, 0), 2);
    cv::rectangle(img, cv::Rect(width_ - 150, height_ - 100, 100, 50), cv::Scalar(0, 0, 255), 2);
}

void MockCamera::addMovingTarget(cv::Mat& img) {
    target_x_ += target_vx_;
    target_y_ += target_vy_;
    
    if (target_x_ < 100 || target_x_ > width_ - 100) {
        target_vx_ = -target_vx_;
    }
    if (target_y_ < 100 || target_y_ > height_ - 100) {
        target_vy_ = -target_vy_;
    }
    
    int armor_width = 60;
    int armor_height = 30;
    
    cv::Point top_left(target_x_ - armor_width / 2, target_y_ - armor_height / 2);
    cv::Point top_right(target_x_ + armor_width / 2, target_y_ - armor_height / 2);
    cv::Point bottom_right(target_x_ + armor_width / 2, target_y_ + armor_height / 2);
    cv::Point bottom_left(target_x_ - armor_width / 2, target_y_ + armor_height / 2);
    
    std::vector<cv::Point> armor_points = {top_left, top_right, bottom_right, bottom_left};
    cv::fillConvexPoly(img, armor_points, cv::Scalar(0, 200, 255));
    
    cv::circle(img, cv::Point(target_x_, target_y_), 5, cv::Scalar(255, 255, 255), -1);
    
    std::string info = fmt::format("Frame: {}", frame_count_);
    cv::putText(img, info, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    
    std::string pos_info = fmt::format("Target: ({:.1f}, {:.1f})", target_x_, target_y_);
    cv::putText(img, pos_info, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
}

void MockCamera::addNoise(cv::Mat& img) {
    cv::Mat noise(img.size(), img.type());
    cv::randn(noise, 0, 5);
    cv::add(img, noise, img);
}

}