#include "autonomous_explorer/slam_processor.hpp"

SlamProcessor::SlamProcessor(rclcpp::Node* node) : node_(node) {
    map_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
    
    // Initialize simple grid map
    map_.header.frame_id = "map";
    map_.info.resolution = RESOLUTION;
    map_.info.width = MAP_SIZE;
    map_.info.height = MAP_SIZE;
    map_.info.origin.position.x = -MAP_SIZE * RESOLUTION / 2;
    map_.info.origin.position.y = -MAP_SIZE * RESOLUTION / 2;
    map_.data.resize(MAP_SIZE * MAP_SIZE, -1); // Unknown
}

void SlamProcessor::process_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    update_map(scan);
    map_.header.stamp = node_->now();
    map_pub_->publish(map_);
}

void SlamProcessor::update_map(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Very simple mapping - mark obstacles
    int center_x = MAP_SIZE / 2;
    int center_y = MAP_SIZE / 2;
    
    for (size_t i = 0; i < scan->ranges.size(); i += 10) { // Subsample
        if (scan->ranges[i] < scan->range_max && scan->ranges[i] > scan->range_min) {
            double angle = scan->angle_min + i * scan->angle_increment;
            int x = center_x + static_cast<int>(scan->ranges[i] * cos(angle) / RESOLUTION);
            int y = center_y + static_cast<int>(scan->ranges[i] * sin(angle) / RESOLUTION);
            
            if (x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE) {
                map_.data[y * MAP_SIZE + x] = 100; // Obstacle
            }
        }
    }
}