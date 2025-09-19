#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  // load ROS2 yaml parameters
  processParameters();

  // Subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map",
    10,
    std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1)
  );

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point",
    10,
    std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1)
  );

  // Subscribe to odometry from /odom/filtered
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered",
    10,
    std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1)
  );

  // Publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // Timer to check goal/timeout status periodically (500 ms)
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&PlannerNode::timerCallback, this)
  );

  // planner_.initPlanner(smoothing_factor_, iterations_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
