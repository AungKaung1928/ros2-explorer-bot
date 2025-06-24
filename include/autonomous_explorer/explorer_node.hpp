#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "slam_processor.hpp"
#include "navigation_controller.hpp"

class ExplorerNode : public rclcpp::Node {
public:
    ExplorerNode();
    
private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void explore_timer_callback();
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::TimerBase::SharedPtr explore_timer_;
    
    std::unique_ptr<SlamProcessor> slam_;
    std::unique_ptr<NavigationController> nav_;
    
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
};