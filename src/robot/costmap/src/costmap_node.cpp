#include <chrono>
#include <memory>
#include <functional>  // for std::bind / placeholders

#include "costmap_node.hpp"

CostmapNode::CostmapNode()
: Node("costmap")
, costmap_core_(robot::CostmapCore(this->get_logger()))
{
  // === Initialize OccupancyGrid ===
  const double resolution = 0.1;   // m/cell
  const unsigned int width  = 300; // cols
  const unsigned int height = 300; // rows

  costmap_msg_.header.frame_id = "map";
  costmap_msg_.info.resolution = resolution;
  costmap_msg_.info.width = width;
  costmap_msg_.info.height = height;
  costmap_msg_.info.origin.position.x = 0.0;
  costmap_msg_.info.origin.position.y = 0.0;
  costmap_msg_.info.origin.orientation.w = 1.0;
  costmap_msg_.data.assign(width * height, 0);  // 0 = free; use -1 for unknown if you prefer

  // === Publishers / Subscriptions / Timer ===
  string_pub_  = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 1);

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar",
    rclcpp::SensorDataQoS(), // safer QoS for sensors
    std::bind(&CostmapNode::lidar_callback, this, std::placeholders::_1)
  );

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&CostmapNode::publishMessage, this)
  );
}

void CostmapNode::publishMessage() {
  // example string publish
  std_msgs::msg::String message;
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);

  // publish the costmap
  costmap_msg_.header.stamp = now();
  costmap_pub_->publish(costmap_msg_);
}

void CostmapNode::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(),
              "Scan: %zu ranges (angle_min=%.3f, angle_max=%.3f, inc=%.3f)",
              msg->ranges.size(), msg->angle_min, msg->angle_max, msg->angle_increment);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
