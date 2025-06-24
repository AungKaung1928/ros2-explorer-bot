#pragma once
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <random>

class NavigationController {
public:
    NavigationController();
    geometry_msgs::msg::Twist compute_velocity(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    
private:
    bool is_obstacle_ahead(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    double find_best_direction(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<> dis_;
    
    static constexpr double MIN_DISTANCE = 0.5;
    static constexpr double MAX_LINEAR_VEL = 0.3;
    static constexpr double MAX_ANGULAR_VEL = 0.8;
};