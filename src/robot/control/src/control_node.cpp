#include "control_node.hpp"

ControlNode::ControlNode() 
  : Node("control"), 
    control_(robot::ControlCore(this->get_logger())),
    lookahead_distance_(1.0),
    goal_tolerance_(0.2),
    linear_speed_(0.5),
    max_angular_velocity_(1.0)
{
  // Subscribers and Publishers
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });

  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Timer
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::controlLoop() {
  if (!current_path_) {
    return;
  }

  if (current_path_->poses.empty()) {
    geometry_msgs::msg::Twist stop_cmd;
    cmd_pub_->publish(stop_cmd);
    return;
  }

  const auto& goal_pose = current_path_->poses.back();
  double distance_to_goal = computeDistance(
    robot_odom_->pose.pose.position, 
    goal_pose.pose.position);
    
  if (distance_to_goal < goal_tolerance_) {
    geometry_msgs::msg::Twist stop_cmd;
    cmd_pub_->publish(stop_cmd);
    return;
  }

  auto lookahead_point = findLookaheadPoint();
  if (!lookahead_point) {
    geometry_msgs::msg::Twist stop_cmd;
    cmd_pub_->publish(stop_cmd);
    return;
  }
  auto cmd_vel = computeVelocity(*lookahead_point);
  cmd_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  const auto& robot_pos = robot_odom_->pose.pose.position;
  
  size_t closest_idx = 0;
  double min_distance = std::numeric_limits<double>::max();
  
  for (size_t i = 0; i < current_path_->poses.size(); ++i) {
    double dist = computeDistance(robot_pos, current_path_->poses[i].pose.position);
    if (dist < min_distance) {
      min_distance = dist;
      closest_idx = i;
    }
  }

  for (size_t i = closest_idx; i < current_path_->poses.size(); ++i) {
    double dist = computeDistance(robot_pos, current_path_->poses[i].pose.position);
    if (dist >= lookahead_distance_) {
      return current_path_->poses[i]; // Returns the one at the radius 
    }
  }

  return current_path_->poses.back();
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped& target) {
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear_speed_;

  const auto& robot_pos = robot_odom_->pose.pose.position;
  const auto& robot_orient = robot_odom_->pose.pose.orientation;
  double dx = target.pose.position.x - robot_pos.x;
  double dy = target.pose.position.y - robot_pos.y;
  
  // Calculate heading
  double goal_yaw = std::atan2(dy, dx);
  double current_yaw = extractYaw(robot_orient);
  double dyaw = goal_yaw - current_yaw;
  while (dyaw > M_PI) dyaw -= 2 * M_PI;
  while (dyaw < -M_PI) dyaw += 2 * M_PI;
  
  
  double k_angular = 2.0; 
  cmd_vel.angular.z = k_angular * dyaw;
  
  if (cmd_vel.angular.z > max_angular_velocity_) {
    cmd_vel.angular.z = max_angular_velocity_;
  } else if (cmd_vel.angular.z < -max_angular_velocity_) {
    cmd_vel.angular.z = -max_angular_velocity_;
  }

  return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion& quat) {
  // Convert quaternion to yaw angle
  double siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y);
  double cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}