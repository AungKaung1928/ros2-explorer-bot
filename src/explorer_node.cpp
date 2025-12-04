#include "autonomous_explorer/explorer_node.hpp"

ExplorerNode::ExplorerNode() 
    : rclcpp_lifecycle::LifecycleNode("explorer_node")
{
    RCLCPP_INFO(get_logger(), "Explorer Node Created");
    
    // Declare all parameters with defaults
    declare_parameters();
}

void ExplorerNode::declare_parameters()
{
    // Safety parameters
    declare_parameter("min_distance", 0.5);
    declare_parameter("min_lane_width", 0.8);
    
    // Speed settings
    declare_parameter("normal_speed", 0.3);
    declare_parameter("turn_speed", 0.3);
    declare_parameter("control_frequency", 10.0);
    
    // Scan angles
    declare_parameter("front_scan_start", 340);
    declare_parameter("front_scan_end", 20);
    declare_parameter("left_scan_start", 60);
    declare_parameter("left_scan_end", 120);
    declare_parameter("right_scan_start", 240);
    declare_parameter("right_scan_end", 300);
    declare_parameter("left_wide_start", 45);
    declare_parameter("left_wide_end", 135);
    declare_parameter("right_wide_start", 225);
    declare_parameter("right_wide_end", 315);
}

void ExplorerNode::load_parameters()
{
    min_distance_ = get_parameter("min_distance").as_double();
    min_lane_width_ = get_parameter("min_lane_width").as_double();
    normal_speed_ = get_parameter("normal_speed").as_double();
    turn_speed_ = get_parameter("turn_speed").as_double();
    control_frequency_ = get_parameter("control_frequency").as_double();
    
    front_scan_start_ = get_parameter("front_scan_start").as_int();
    front_scan_end_ = get_parameter("front_scan_end").as_int();
    left_scan_start_ = get_parameter("left_scan_start").as_int();
    left_scan_end_ = get_parameter("left_scan_end").as_int();
    right_scan_start_ = get_parameter("right_scan_start").as_int();
    right_scan_end_ = get_parameter("right_scan_end").as_int();
    left_wide_start_ = get_parameter("left_wide_start").as_int();
    left_wide_end_ = get_parameter("left_wide_end").as_int();
    right_wide_start_ = get_parameter("right_wide_start").as_int();
    right_wide_end_ = get_parameter("right_wide_end").as_int();
    
    RCLCPP_INFO(get_logger(), "Parameters loaded:");
    RCLCPP_INFO(get_logger(), "  min_distance: %.2f", min_distance_);
    RCLCPP_INFO(get_logger(), "  min_lane_width: %.2f", min_lane_width_);
    RCLCPP_INFO(get_logger(), "  normal_speed: %.2f", normal_speed_);
    RCLCPP_INFO(get_logger(), "  turn_speed: %.2f", turn_speed_);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ExplorerNode::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Configuring...");
    
    // Load parameters from YAML
    load_parameters();
    
    // Initialize state variables
    obstacle_detected_ = false;
    front_clear_ = true;
    left_clear_ = true;
    right_clear_ = true;
    left_lane_width_ = 0.0;
    right_lane_width_ = 0.0;
    latest_scan_ = nullptr;
    
    // Create publisher (inactive initially)
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    // Create subscriber
    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&ExplorerNode::laser_callback, this, std::placeholders::_1));
    
    // Create timer with configurable frequency
    int timer_ms = static_cast<int>(1000.0 / control_frequency_);
    move_timer_ = create_wall_timer(
        std::chrono::milliseconds(timer_ms),
        std::bind(&ExplorerNode::move_robot, this));
    
    RCLCPP_INFO(get_logger(), "Configuration complete");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ExplorerNode::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Activating...");
    
    // Activate publisher
    cmd_pub_->on_activate();
    
    RCLCPP_INFO(get_logger(), "Explorer Node Active - Ready to Navigate!");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ExplorerNode::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Deactivating...");
    
    // Stop robot
    auto stop_cmd = geometry_msgs::msg::Twist();
    cmd_pub_->publish(stop_cmd);
    
    // Deactivate publisher
    cmd_pub_->on_deactivate();
    
    RCLCPP_INFO(get_logger(), "Deactivated");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ExplorerNode::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Cleaning up...");
    
    // Reset timer and subscriptions
    move_timer_.reset();
    laser_sub_.reset();
    cmd_pub_.reset();
    
    RCLCPP_INFO(get_logger(), "Cleanup complete");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ExplorerNode::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Shutting down...");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void ExplorerNode::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    try {
        latest_scan_ = msg;
        const auto& ranges = msg->ranges;
        int total_readings = ranges.size();
        
        if (total_readings == 0) {
            return;
        }
        
        std::vector<float> front_ranges;
        std::vector<float> left_ranges;
        std::vector<float> right_ranges;
        std::vector<float> left_wide_ranges;
        std::vector<float> right_wide_ranges;
        
        // Front scanning using configurable angles
        for (int i = front_scan_start_; i < 360; ++i) {
            if (i < total_readings) front_ranges.push_back(ranges[i]);
        }
        for (int i = 0; i < front_scan_end_; ++i) {
            if (i < total_readings) front_ranges.push_back(ranges[i]);
        }
        
        // Left side
        for (int i = left_scan_start_; i < left_scan_end_; ++i) {
            if (i < total_readings) left_ranges.push_back(ranges[i]);
        }
        
        // Right side
        for (int i = right_scan_start_; i < right_scan_end_; ++i) {
            if (i < total_readings) right_ranges.push_back(ranges[i]);
        }
        
        // Wide scanning for lane width
        for (int i = left_wide_start_; i < left_wide_end_; ++i) {
            if (i < total_readings) left_wide_ranges.push_back(ranges[i]);
        }
        
        for (int i = right_wide_start_; i < right_wide_end_; ++i) {
            if (i < total_readings) right_wide_ranges.push_back(ranges[i]);
        }
        
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
        
        front_clear_ = front_distances.empty() || 
                      (*std::min_element(front_distances.begin(), front_distances.end()) > min_distance_);
        left_clear_ = left_distances.empty() || 
                     (*std::min_element(left_distances.begin(), left_distances.end()) > min_distance_);
        right_clear_ = right_distances.empty() || 
                      (*std::min_element(right_distances.begin(), right_distances.end()) > min_distance_);
        
        left_lane_width_ = left_wide_distances.empty() ? 0.0 : 
                          *std::min_element(left_wide_distances.begin(), left_wide_distances.end());
        right_lane_width_ = right_wide_distances.empty() ? 0.0 : 
                           *std::min_element(right_wide_distances.begin(), right_wide_distances.end());
        
        obstacle_detected_ = !front_clear_;
        
    } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "Laser processing error: %s", e.what());
    }
}

void ExplorerNode::move_robot()
{
    // Only move if node is active
    // State ID 3 = PRIMARY_STATE_ACTIVE in lifecycle state machine
    auto current_state = get_current_state();
    if (current_state.id() != 3) {
        return;
    }
    
    if (!latest_scan_) {
        return;
    }
    
    auto cmd = geometry_msgs::msg::Twist();
    
    try {
        if (front_clear_) {
            cmd.linear.x = normal_speed_;
            cmd.angular.z = 0.0;
            RCLCPP_INFO(get_logger(), "Moving forward - path clear");
            
        } else {
            cmd.linear.x = 0.0;
            
            bool right_safe = right_clear_ && right_lane_width_ > min_lane_width_;
            bool left_safe = left_clear_ && left_lane_width_ > min_lane_width_;
            
            if (right_safe && left_safe) {
                if (right_lane_width_ >= left_lane_width_) {
                    cmd.angular.z = -turn_speed_;
                    RCLCPP_INFO(get_logger(), "Turning right - lane width: %.2fm", right_lane_width_);
                } else {
                    cmd.angular.z = turn_speed_;
                    RCLCPP_INFO(get_logger(), "Turning left - lane width: %.2fm", left_lane_width_);
                }
                
            } else if (right_safe) {
                cmd.angular.z = -turn_speed_;
                RCLCPP_INFO(get_logger(), "Turning right - safe lane width: %.2fm", right_lane_width_);
                
            } else if (left_safe) {
                cmd.angular.z = turn_speed_;
                RCLCPP_INFO(get_logger(), "Turning left - safe lane width: %.2fm", left_lane_width_);
                
            } else {
                cmd.angular.z = turn_speed_;
                if (right_clear_ || left_clear_) {
                    RCLCPP_INFO(get_logger(), "Lanes too narrow - continuing to search");
                } else {
                    RCLCPP_INFO(get_logger(), "No clear path - rotating");
                }
            }
        }
        
        cmd_pub_->publish(cmd);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Movement error: %s", e.what());
        auto stop_cmd = geometry_msgs::msg::Twist();
        cmd_pub_->publish(stop_cmd);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<ExplorerNode>();
        
        // Executor for lifecycle management
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node->get_node_base_interface());
        
        // Automatic state transitions
        node->configure();
        node->activate();
        
        executor.spin();
        
        // Cleanup on shutdown
        node->deactivate();
        node->cleanup();
        node->shutdown();
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}