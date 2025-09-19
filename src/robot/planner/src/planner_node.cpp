#include "planner_node.hpp"

#include <chrono>
#include <cmath>

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  processParameters();

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

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered",
    10,
    std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1)
  );

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&PlannerNode::timerCallback, this)
  );

  // planner_.initPlanner(smoothing_factor_, iterations_);
}

void PlannerNode::processParameters() {
  // TODO: declare/get params here later
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg){

}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr  goal_msg){

}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg){

}

void PlannerNode::timerCallback(){

}

void PlannerNode::pubPath(){
  
}

void PlannerNode::resetGoal(){

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
