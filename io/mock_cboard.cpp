#include "mock_cboard.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <thread>

namespace io {

MockCBoard::MockCBoard(const std::string& config_path)
    : start_time_(std::chrono::steady_clock::now())
    , bullet_speed(27.0)
    , mode(Mode::auto_aim)
    , shoot_mode(ShootMode::both_shoot)
    , ft_angle(0.0)
    , current_yaw_(0.0)
    , current_pitch_(0.0)
    , current_roll_(0.0)
    , yaw_rate_(0.0)
    , pitch_rate_(0.0)
    , roll_rate_(0.0) {
    
    generateMockIMUData();
    
    tools::logger()->info("MockCBoard initialized");
    tools::logger()->info("  Bullet speed: {} m/s", bullet_speed);
    tools::logger()->info("  Mode: {}", MODES[mode]);
    tools::logger()->info("  Shoot mode: {}", SHOOT_MODES[shoot_mode]);
}

MockCBoard::~MockCBoard() {
    tools::logger()->info("MockCBoard destroyed");
}

Eigen::Quaterniond MockCBoard::imu_at(std::chrono::steady_clock::time_point timestamp) {
    return interpolateIMU(timestamp);
}

void MockCBoard::send(Command command) const {
    tools::logger()->debug("MockCBoard::send() - Control: {}, Yaw: {:.2f}, Pitch: {:.2f}, Shoot: {}",
                    command.control, command.yaw * 57.3, command.pitch * 57.3, command.shoot);
}

void MockCBoard::generateMockIMUData() {
    auto current_time = start_time_;
    auto end_time = start_time_ + std::chrono::minutes(10);
    
    double time_step = 0.01;
    
    while (current_time < end_time) {
        yaw_rate_ = 0.5 * std::sin(current_time.time_since_epoch().count() * 1e-9 * 0.5);
        pitch_rate_ = 0.3 * std::cos(current_time.time_since_epoch().count() * 1e-9 * 0.3);
        roll_rate_ = 0.2 * std::sin(current_time.time_since_epoch().count() * 1e-9 * 0.7);
        
        current_yaw_ += yaw_rate_ * time_step;
        current_pitch_ += pitch_rate_ * time_step;
        current_roll_ += roll_rate_ * time_step;
        
        Eigen::Quaterniond q = Eigen::AngleAxisd(current_yaw_, Eigen::Vector3d::UnitZ()) *
                             Eigen::AngleAxisd(current_pitch_, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(current_roll_, Eigen::Vector3d::UnitX());
        
        imu_data_[current_time] = q;
        
        current_time += std::chrono::microseconds(static_cast<int64_t>(time_step * 1e6));
    }
}

Eigen::Quaterniond MockCBoard::interpolateIMU(std::chrono::steady_clock::time_point timestamp) {
    if (imu_data_.empty()) {
        return Eigen::Quaterniond::Identity();
    }
    
    auto it = imu_data_.lower_bound(timestamp);
    
    if (it == imu_data_.begin()) {
        return it->second;
    }
    
    if (it == imu_data_.end()) {
        return (--it)->second;
    }
    
    auto next_it = it;
    auto prev_it = --it;
    
    auto prev_time = prev_it->first;
    auto next_time = next_it->first;
    auto prev_q = prev_it->second;
    auto next_q = next_it->second;
    
    double alpha = std::chrono::duration<double>(timestamp - prev_time).count() /
                  std::chrono::duration<double>(next_time - prev_time).count();
    
    return prev_q.slerp(alpha, next_q);
}

}