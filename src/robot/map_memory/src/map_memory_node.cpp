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

  // Use transient_local so new viewers (Foxglove/RViz) get the latest map immediately
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).reliable().transient_local());

  timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_costmap_ = *msg;
  costmap_updated_ = true;

  if (global_map_.info.width == 0) {
    global_map_ = latest_costmap_;

  }
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const double x = msg->pose.pose.position.x;
  const double y = msg->pose.pose.position.y;

  const double dx = x - last_x_;
  const double dy = y - last_y_;
  const double distance = std::hypot(dx, dy);

  if (distance >= distance_threshold_) {
    last_x_ = x;
    last_y_ = y;
    should_update_map_ = true;
  }
}

void MapMemoryNode::updateMap()
{
  if (should_update_map_ && costmap_updated_) {
    integrateCostmap();
    global_map_.header.stamp = now();
    map_pub_->publish(global_map_);
    should_update_map_ = false;
    costmap_updated_ = false;
  }
}

void MapMemoryNode::integrateCostmap()
{
  //TODO: TRNASOFRM TAKEN FROM ODOM PMO PMO PMO
  if (global_map_.info.width != latest_costmap_.info.width ||
      global_map_.info.height != latest_costmap_.info.height ||
      global_map_.info.resolution != latest_costmap_.info.resolution) {
    RCLCPP_WARN(get_logger(), "Costmap and global_map dimensions/resolution differ; skipping merge.");
    return;
  }

  const size_t N = latest_costmap_.data.size();
  if (global_map_.data.size() != N) global_map_.data.resize(N, -1);

  for (size_t i = 0; i < N; ++i) {
    const int8_t a = global_map_.data[i];
    const int8_t b = latest_costmap_.data[i];
    // Unknown handling: treat -1 (unknown) as 0 baseline when taking max
    const int va = (a < 0) ? 0 : a;
    const int vb = (b < 0) ? 0 : b;
    global_map_.data[i] = static_cast<int8_t>(std::max(va, vb));
  }

  global_map_.header.frame_id = latest_costmap_.header.frame_id;
  global_map_.info.origin     = latest_costmap_.info.origin;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}