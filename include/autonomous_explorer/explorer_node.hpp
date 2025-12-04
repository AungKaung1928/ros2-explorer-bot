#ifndef AUTONOMOUS_EXPLORER__EXPLORER_NODE_HPP_
#define AUTONOMOUS_EXPLORER__EXPLORER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>
#include <algorithm>
#include <memory>

class ExplorerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    ExplorerNode();

    // Lifecycle callbacks
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State& previous_state);
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State& previous_state);
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State& previous_state);
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State& previous_state);
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State& previous_state);

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void move_robot();
    void declare_parameters();
    void load_parameters();
    
    // Publishers and subscribers
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::TimerBase::SharedPtr move_timer_;
    
    // Robot state variables
    bool obstacle_detected_;
    bool front_clear_;
    bool left_clear_;
    bool right_clear_;
    double left_lane_width_;
    double right_lane_width_;
    
    // Parameters (configurable from YAML)
    double min_distance_;
    double min_lane_width_;
    double normal_speed_;
    double turn_speed_;
    double control_frequency_;
    
    // Scan angle parameters
    int front_scan_start_;
    int front_scan_end_;
    int left_scan_start_;
    int left_scan_end_;
    int right_scan_start_;
    int right_scan_end_;
    int left_wide_start_;
    int left_wide_end_;
    int right_wide_start_;
    int right_wide_end_;
    
    // Latest scan data
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
};

#endif  // AUTONOMOUS_EXPLORER__EXPLORER_NODE_HPP_