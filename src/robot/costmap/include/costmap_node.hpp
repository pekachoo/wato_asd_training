#ifndef COSTMAP_NODEHPP
#define COSTMAP_NODEHPP

#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "costmap_core.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();

    // Place callback function here
    void publishMessage();
    void inflateObstacles(nav_msgs::msg::OccupancyGrid & grid,
                      double inflation_radius,
                      int max_cost);
    void convertToGrid(double range, double angle, int &gx, int &gy);
    void markObstacle(int gx, int gy);

  private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    nav_msgs::msg::OccupancyGrid costmap_msg_;
    robot::CostmapCore costmap_;
    // Place these constructs here
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

};

#endif