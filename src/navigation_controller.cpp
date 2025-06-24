#include "autonomous_explorer/navigation_controller.hpp"
#include <algorithm>
#include <cmath>

NavigationController::NavigationController() : gen_(rd_()), dis_(-1.0, 1.0) {}

geometry_msgs::msg::Twist NavigationController::compute_velocity(
    const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    
    geometry_msgs::msg::Twist cmd;
    
    if (is_obstacle_ahead(scan)) {
        // Turn to avoid obstacle
        cmd.linear.x = 0.1;
        cmd.angular.z = find_best_direction(scan);
    } else {
        // Move forward and explore
        cmd.linear.x = MAX_LINEAR_VEL;
        cmd.angular.z = dis_(gen_) * 0.2; // Small random turn
    }
    
    return cmd;
}

bool NavigationController::is_obstacle_ahead(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    size_t front_start = scan->ranges.size() * 0.4;
    size_t front_end = scan->ranges.size() * 0.6;
    
    for (size_t i = front_start; i < front_end; ++i) {
        if (scan->ranges[i] < MIN_DISTANCE) {
            return true;
        }
    }
    return false;
}

double NavigationController::find_best_direction(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Simple: turn towards the side with more free space
    double left_space = 0, right_space = 0;
    
    for (size_t i = 0; i < scan->ranges.size() / 2; ++i) {
        if (scan->ranges[i] > MIN_DISTANCE) left_space += scan->ranges[i];
    }
    
    for (size_t i = scan->ranges.size() / 2; i < scan->ranges.size(); ++i) {
        if (scan->ranges[i] > MIN_DISTANCE) right_space += scan->ranges[i];
    }
    
    return (left_space > right_space) ? MAX_ANGULAR_VEL : -MAX_ANGULAR_VEL;
}