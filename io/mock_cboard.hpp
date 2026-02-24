#pragma once

#include <Eigen/Geometry>
#include <chrono>
#include <string>
#include <vector>
#include <map>
#include "io/command.hpp"
#include "io/cboard.hpp"
#include "tools/logger.hpp"

namespace io {

class MockCBoard {
public:
    MockCBoard(const std::string& config_path);
    ~MockCBoard();

    Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);
    void send(Command command) const;

    double bullet_speed;
    Mode mode;
    ShootMode shoot_mode;
    double ft_angle;

private:
    void generateMockIMUData();
    Eigen::Quaterniond interpolateIMU(std::chrono::steady_clock::time_point timestamp);

    std::chrono::steady_clock::time_point start_time_;
    std::map<std::chrono::steady_clock::time_point, Eigen::Quaterniond> imu_data_;
    
    double current_yaw_;
    double current_pitch_;
    double current_roll_;
    
    double yaw_rate_;
    double pitch_rate_;
    double roll_rate_;
};

}