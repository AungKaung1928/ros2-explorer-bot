#include "autonomous_explorer/explorer_node.hpp"

ExplorerNode::ExplorerNode() : Node("explorer_node") {
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&ExplorerNode::laser_callback, this, std::placeholders::_1));
    
    explore_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ExplorerNode::explore_timer_callback, this));
    
    slam_ = std::make_unique<SlamProcessor>(this);
    nav_ = std::make_unique<NavigationController>();
    
    RCLCPP_INFO(get_logger(), "Explorer node started");
}

void ExplorerNode::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    latest_scan_ = msg;
    slam_->process_scan(msg);
}

void ExplorerNode::explore_timer_callback() {
    if (!latest_scan_) return;
    
    auto cmd = nav_->compute_velocity(latest_scan_);
    cmd_pub_->publish(cmd);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExplorerNode>());
    rclcpp::shutdown();
    return 0;
}