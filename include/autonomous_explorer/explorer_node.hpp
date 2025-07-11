#ifndef AUTONOMOUS_EXPLORER__EXPLORER_NODE_HPP_
#define AUTONOMOUS_EXPLORER__EXPLORER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>
#include <algorithm>
#include <memory>

class ExplorerNode : public rclcpp::Node
{
public:
    ExplorerNode();

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void move_robot();
    
    // Publishers and subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::TimerBase::SharedPtr move_timer_;
    
    // Robot state variables
    bool obstacle_detected_;
    double min_distance_;
    double min_lane_width_;
    bool front_clear_;
    bool left_clear_;
    bool right_clear_;
    double left_lane_width_;
    double right_lane_width_;
    
    // Speed settings
    double normal_speed_;
    double turn_speed_;
    
    // Latest scan data
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
};

#endif  // AUTONOMOUS_EXPLORER__EXPLORER_NODE_HPP_