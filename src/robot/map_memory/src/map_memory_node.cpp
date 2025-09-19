#include <cmath>
#include <functional>
#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
: Node("map_memory"),
  map_memory_(robot::MapMemoryCore(this->get_logger()))
{
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10,
    std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).reliable().transient_local());

  timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&MapMemoryNode::updateMap, this));

  initializeGlobalMap();
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_costmap_ = *msg;
  costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  robot_yaw_ = quaternionToYaw(msg->pose.pose.orientation);
  has_odom_ = true;

  if (!map_initialized_) {
    global_map_.header.frame_id = "sim_world";
    global_map_.header.stamp = this->now();
    map_pub_->publish(global_map_);
    map_initialized_ = true;
  }
}


void MapMemoryNode::updateMap()
{
  if (costmap_updated_ && has_odom_ && map_initialized_) {
    integrateCostmap();
    global_map_.header.frame_id = "sim_world";
    global_map_.header.stamp = this->now();
    map_pub_->publish(global_map_);
    costmap_updated_ = false;
  }
}

void MapMemoryNode::integrateCostmap()
{
  if (!has_odom_) return;

  const int costmap_width = static_cast<int>(latest_costmap_.info.width);
  const int costmap_height = static_cast<int>(latest_costmap_.info.height);
  const double costmap_resolution = latest_costmap_.info.resolution;
  
  const int global_width = static_cast<int>(global_map_.info.width);
  const int global_height = static_cast<int>(global_map_.info.height);
  const double global_resolution = global_map_.info.resolution;
  
  if (std::abs(costmap_resolution - global_resolution) > 1e-6) return;

  for (int cy = 0; cy < costmap_height; ++cy) {
    for (int cx = 0; cx < costmap_width; ++cx) {
      const int costmap_idx = cy * costmap_width + cx;
      const int8_t cost_value = latest_costmap_.data[costmap_idx];
      
      if (cost_value <= 0) continue;
      
      const double local_x = latest_costmap_.info.origin.position.x + cx * costmap_resolution;
      const double local_y = latest_costmap_.info.origin.position.y + cy * costmap_resolution;
      
      const double cos_yaw = std::cos(robot_yaw_);
      const double sin_yaw = std::sin(robot_yaw_);
      
      const double world_x = robot_x_ + (cos_yaw * local_x - sin_yaw * local_y);
      const double world_y = robot_y_ + (sin_yaw * local_x + cos_yaw * local_y);
      
      const double global_origin_x = global_map_.info.origin.position.x;
      const double global_origin_y = global_map_.info.origin.position.y;
      
      const int gx = static_cast<int>((world_x - global_origin_x) / global_resolution);
      const int gy = static_cast<int>((world_y - global_origin_y) / global_resolution);
      
      if (gx >= 0 && gx < global_width && gy >= 0 && gy < global_height) {
        const int global_idx = gy * global_width + gx;
        const int8_t current_val = global_map_.data[global_idx];
        const int current = (current_val < 0) ? 0 : current_val;
        global_map_.data[global_idx] = static_cast<int8_t>(std::max(current, static_cast<int>(cost_value)));
      }
    }
  }
}


void MapMemoryNode::initializeGlobalMap()
{
  const double resolution = 0.1;
  const unsigned int width = 600;
  const unsigned int height = 600;
  
  global_map_.header.frame_id = "sim_world";
  global_map_.header.stamp = this->now();
  
  global_map_.info.resolution = resolution;
  global_map_.info.width = width;
  global_map_.info.height = height;
  
  global_map_.info.origin.position.x = -30.0;
  global_map_.info.origin.position.y = -30.0;
  global_map_.info.origin.position.z = 0.0;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.info.origin.orientation.x = 0.0;
  global_map_.info.origin.orientation.y = 0.0;
  global_map_.info.origin.orientation.z = 0.0;
  
  global_map_.data.assign(width * height, -1);
}



double MapMemoryNode::quaternionToYaw(const geometry_msgs::msg::Quaternion& quat)
{
  // Convert quaternion to yaw angle using atan2
  const double siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y);
  const double cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}