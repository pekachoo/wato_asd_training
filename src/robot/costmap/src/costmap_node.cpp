#include "costmap_node.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

// ===== Constructor =====
CostmapNode::CostmapNode()
: Node("costmap"),
  costmap_(robot::CostmapCore(this->get_logger()))
{
  // ---- Costmap setup (tweak as needed) ----
  const double resolution = 0.1;    // meters per cell
  const unsigned int width  = 300;  // columns
  const unsigned int height = 300;  // rows

  costmap_msg_.header.frame_id = "base_link"; // temporary default; overwritten per-scan to lidar frame
  costmap_msg_.info.resolution = resolution;
  costmap_msg_.info.width      = width;
  costmap_msg_.info.height     = height;

  // Center the grid around the robot (lower-left is at -W/2, -H/2)
  const double Wm = resolution * static_cast<double>(width);
  const double Hm = resolution * static_cast<double>(height);
  costmap_msg_.info.origin.position.x = -0.5 * Wm;
  costmap_msg_.info.origin.position.y = -0.5 * Hm;
  costmap_msg_.info.origin.position.z = 0.0;
  costmap_msg_.info.origin.orientation.w = 1.0;

  costmap_msg_.data.assign(width * height, 0); // 0 = free

  // ---- ROS I/O ----
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 1);
  string_pub_  = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 10, std::bind(&CostmapNode::lidar_callback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&CostmapNode::publishMessage, this));
}

// ===== Simple timer publisher =====
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

// ===== Inflation with linear decay =====
// cost = max_cost * (1 - dist / inflation_radius), clamped, only within radius
void CostmapNode::inflateObstacles(nav_msgs::msg::OccupancyGrid &grid,
                                   double inflation_radius,
                                   int max_cost) {
  const double res = grid.info.resolution;
  const int W = static_cast<int>(grid.info.width);
  const int H = static_cast<int>(grid.info.height);
  if (W <= 0 || H <= 0 || res <= 0.0 || inflation_radius <= 0.0) return;

  const int cell_radius = static_cast<int>(std::ceil(inflation_radius / res));

  // snapshot to avoid self-amplifying during pass
  const std::vector<int8_t> original = grid.data;

  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      if (original[y * W + x] < 100) continue;  // inflate from only true obstacles

      for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
        for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
          const int nx = x + dx;
          const int ny = y + dy;
          if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;

          const double dist = std::hypot(dx * res, dy * res);
          if (dist > inflation_radius) continue;

          const int cost = static_cast<int>(std::round(max_cost * (1.0 - dist / inflation_radius)));
          const int idx  = ny * W + nx;
          if (cost > grid.data[idx]) {
            grid.data[idx] = static_cast<int8_t>(std::min(cost, max_cost));
          }
        }
      }
    }
  }
}

// ===== Polar (range, angle) -> grid indices (robot-centered, no TF) =====
// Assumes scan frame == costmap_msg_.header.frame_id. Grid is centered at (0,0).
void CostmapNode::convertToGrid(double range, double angle, int &gx, int &gy) {
  const double x = range * std::cos(angle); // meters, in scan frame
  const double y = range * std::sin(angle);

  const double res = costmap_msg_.info.resolution;
  const double x0  = costmap_msg_.info.origin.position.x;  // lower-left (robot-centered)
  const double y0  = costmap_msg_.info.origin.position.y;

  gx = static_cast<int>(std::floor((x - x0) / res));
  gy = static_cast<int>(std::floor((y - y0) / res));
}

// ===== Mark a cell as occupied (100) if in bounds =====
void CostmapNode::markObstacle(int gx, int gy) {
  const int W = static_cast<int>(costmap_msg_.info.width);
  const int H = static_cast<int>(costmap_msg_.info.height);
  if (gx < 0 || gx >= W || gy < 0 || gy >= H) return;

  const int idx = gy * W + gx;
  costmap_msg_.data[idx] = static_cast<int8_t>(std::max<int>(costmap_msg_.data[idx], 100));
}

// ===== LiDAR callback =====
void CostmapNode::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // Make the grid live in the same frame/timestamp as the scan to avoid "swimming"
  costmap_msg_.header.frame_id = msg->header.frame_id; // e.g., "lidar_link" or "base_link"
  costmap_msg_.header.stamp    = msg->header.stamp;

  // Clear grid each scan (simple). Remove for persistence/decay if desired.
  std::fill(costmap_msg_.data.begin(), costmap_msg_.data.end(), 0);

  // Rasterize scan points (robot-centered)
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
    const double range = msg->ranges[i];

    if (!std::isfinite(range)) continue;
    if (range <= msg->range_min || range >= msg->range_max) continue;

    int gx = 0, gy = 0;
    convertToGrid(range, angle, gx, gy);
    markObstacle(gx, gy);
  }

  // Inflate obstacles
  inflateObstacles(costmap_msg_, /*inflation_radius=*/0.3, /*max_cost=*/100);

  // Publish
  costmap_pub_->publish(costmap_msg_);
}

// (Optional) main in this translation unit; remove if you have a separate main TU.
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}