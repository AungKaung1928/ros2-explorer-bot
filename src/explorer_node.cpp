#include "autonomous_explorer/explorer_node.hpp"

ExplorerNode::ExplorerNode() : Node("explorer_node")
{
    // Initialize publishers and subscribers
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&ExplorerNode::laser_callback, this, std::placeholders::_1));
    
    // Timer to publish commands regularly (100ms = 10Hz)
    move_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ExplorerNode::move_robot, this));
    
    // Initialize robot state variables
    obstacle_detected_ = false;
    min_distance_ = 0.5;      // Stop if obstacle closer than 50cm
    min_lane_width_ = 0.8;    // Minimum lane width to enter (80cm)
    front_clear_ = true;
    left_clear_ = true;
    right_clear_ = true;
    left_lane_width_ = 0.0;
    right_lane_width_ = 0.0;
    
    // Speed settings - reduced for smoother movement
    normal_speed_ = 0.3;      // Reduced for smooth movement
    turn_speed_ = 0.3;        // Smooth turning speed
    
    latest_scan_ = nullptr;
    
    RCLCPP_INFO(get_logger(), "Simple Obstacle Avoider Started!");
}

void ExplorerNode::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    try {
        // Store latest scan for processing
        latest_scan_ = msg;
        
        // Get ranges (distances) from laser
        const auto& ranges = msg->ranges;
        int total_readings = ranges.size();
        
        if (total_readings == 0) {
            return;
        }
        
        // Check different directions with wider scanning
        // Front: indices around middle of scan
        std::vector<float> front_ranges;
        std::vector<float> left_ranges;
        std::vector<float> right_ranges;
        std::vector<float> left_wide_ranges;
        std::vector<float> right_wide_ranges;
        
        // Front 40 degrees (340-360 and 0-20 indices)
        for (int i = 340; i < 360; ++i) {
            if (i < total_readings) front_ranges.push_back(ranges[i]);
        }
        for (int i = 0; i < 20; ++i) {
            if (i < total_readings) front_ranges.push_back(ranges[i]);
        }
        
        // Left side (60-120 indices)
        for (int i = 60; i < 120; ++i) {
            if (i < total_readings) left_ranges.push_back(ranges[i]);
        }
        
        // Right side (240-300 indices)
        for (int i = 240; i < 300; ++i) {
            if (i < total_readings) right_ranges.push_back(ranges[i]);
        }
        
        // Extended scanning for lane width measurement
        // Wider left scan (45-135 indices)
        for (int i = 45; i < 135; ++i) {
            if (i < total_readings) left_wide_ranges.push_back(ranges[i]);
        }
        
        // Wider right scan (225-315 indices)
        for (int i = 225; i < 315; ++i) {
            if (i < total_readings) right_wide_ranges.push_back(ranges[i]);
        }
        
        // Filter out invalid readings (inf, nan, 0)
        auto filter_ranges = [](const std::vector<float>& input) {
            std::vector<float> filtered;
            for (float r : input) {
                if (std::isfinite(r) && r > 0.1 && r < 10.0) {
                    filtered.push_back(r);
                }
            }
            return filtered;
        };
        
        auto front_distances = filter_ranges(front_ranges);
        auto left_distances = filter_ranges(left_ranges);
        auto right_distances = filter_ranges(right_ranges);
        auto left_wide_distances = filter_ranges(left_wide_ranges);
        auto right_wide_distances = filter_ranges(right_wide_ranges);
        
        // Check if path is clear (no obstacles within min_distance)
        front_clear_ = front_distances.empty() || 
                      (*std::min_element(front_distances.begin(), front_distances.end()) > min_distance_);
        left_clear_ = left_distances.empty() || 
                     (*std::min_element(left_distances.begin(), left_distances.end()) > min_distance_);
        right_clear_ = right_distances.empty() || 
                      (*std::min_element(right_distances.begin(), right_distances.end()) > min_distance_);
        
        // Measure lane widths to determine if they're safe to enter
        left_lane_width_ = left_wide_distances.empty() ? 0.0 : 
                          *std::min_element(left_wide_distances.begin(), left_wide_distances.end());
        right_lane_width_ = right_wide_distances.empty() ? 0.0 : 
                           *std::min_element(right_wide_distances.begin(), right_wide_distances.end());
        
        // Overall obstacle detection
        obstacle_detected_ = !front_clear_;
        
    } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "Laser processing error: %s", e.what());
    }
}

void ExplorerNode::move_robot()
{
    // Don't move if we haven't received any laser data yet
    if (!latest_scan_) {
        return;
    }
    
    auto cmd = geometry_msgs::msg::Twist();
    
    try {
        if (front_clear_) {
            // Move forward at normal speed
            cmd.linear.x = normal_speed_;
            cmd.angular.z = 0.0;
            RCLCPP_INFO(get_logger(), "Moving forward - path clear");
            
        } else {
            // Obstacle detected - think about available options
            cmd.linear.x = 0.0;  // Stop forward motion
            
            // Check if lanes are wide enough and clear
            bool right_safe = right_clear_ && right_lane_width_ > min_lane_width_;
            bool left_safe = left_clear_ && left_lane_width_ > min_lane_width_;
            
            if (right_safe && left_safe) {
                // Both lanes are safe - choose the wider one
                if (right_lane_width_ >= left_lane_width_) {
                    cmd.angular.z = -turn_speed_;
                    RCLCPP_INFO(get_logger(), "Turning right - lane width: %.2fm", right_lane_width_);
                } else {
                    cmd.angular.z = turn_speed_;
                    RCLCPP_INFO(get_logger(), "Turning left - lane width: %.2fm", left_lane_width_);
                }
                
            } else if (right_safe) {
                // Only right lane is safe
                cmd.angular.z = -turn_speed_;
                RCLCPP_INFO(get_logger(), "Turning right - safe lane width: %.2fm", right_lane_width_);
                
            } else if (left_safe) {
                // Only left lane is safe
                cmd.angular.z = turn_speed_;
                RCLCPP_INFO(get_logger(), "Turning left - safe lane width: %.2fm", left_lane_width_);
                
            } else {
                // No safe lanes - continue rotating to find better option
                cmd.angular.z = turn_speed_;
                if (right_clear_ || left_clear_) {
                    RCLCPP_INFO(get_logger(), "Lanes too narrow - continuing to search for wider path");
                } else {
                    RCLCPP_INFO(get_logger(), "No clear path - rotating to find opening");
                }
            }
        }
        
        // Publish the command
        cmd_pub_->publish(cmd);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Movement error: %s", e.what());
        // Stop robot on error
        auto stop_cmd = geometry_msgs::msg::Twist();
        cmd_pub_->publish(stop_cmd);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<ExplorerNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}