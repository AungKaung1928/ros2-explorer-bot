#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>

class SlamProcessor {
public:
    SlamProcessor(rclcpp::Node* node);
    void process_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    nav_msgs::msg::OccupancyGrid get_map() const { return map_; }
    
private:
    void update_map(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    
    rclcpp::Node* node_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    nav_msgs::msg::OccupancyGrid map_;
    
    // Simple grid parameters
    static constexpr int MAP_SIZE = 200;
    static constexpr double RESOLUTION = 0.1;
};