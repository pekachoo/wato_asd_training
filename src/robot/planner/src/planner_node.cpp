#include "planner_node.hpp"

#include <chrono>
#include <cmath>
#include <queue> 
#include <unordered_map> 
#include <vector> 
#include <limits>

struct CellIndex
{
  int x;
  int y;
 
  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}
 
  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }
 
  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};
 
// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};
 
// Structure representing a node in the A* open set
struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h
 
  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};
 
// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
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

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg){
  current_map_ = *map_msg;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
      planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr  goal_msg){
  goal_ = *goal_msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg){
  robot_pose_ = odom_msg->pose.pose;
}

void PlannerNode::timerCallback(){
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;
    } else {
      RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
      planPath();
    }
  }
}

bool PlannerNode::goalReached() {
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < 0.5; // Threshold for reaching the goal
}

void PlannerNode::planPath() {
  if (!goal_received_ || current_map_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "🗺️ Creating path from robot position: (%.3f, %.3f)", 
              robot_pose_.position.x, robot_pose_.position.y);

  // --- Short-hands ---
  const auto& grid = current_map_;
  const double res = grid.info.resolution;
  const double origin_x = grid.info.origin.position.x;
  const double origin_y = grid.info.origin.position.y;
  const int W = static_cast<int>(grid.info.width);
  const int H = static_cast<int>(grid.info.height);

  // --- Helpers (lambdas) ---
  auto worldToCell = [&](double wx, double wy) -> std::optional<CellIndex> {
    int gx = static_cast<int>(std::floor((wx - origin_x) / res));
    int gy = static_cast<int>(std::floor((wy - origin_y) / res));
    if (gx < 0 || gy < 0 || gx >= W || gy >= H) return std::nullopt;
    return CellIndex(gx, gy);
  };

  auto cellToWorld = [&](const CellIndex& c) -> std::pair<double,double> {
    // cell center
    return {origin_x + (c.x + 0.5) * res, origin_y + (c.y + 0.5) * res};
  };

  auto inBounds = [&](const CellIndex& c) {
    return (c.x >= 0 && c.y >= 0 && c.x < W && c.y < H);
  };

  auto isFree = [&](const CellIndex& c) {
    const int idx = c.y * W + c.x;
    if (idx < 0 || idx >= static_cast<int>(grid.data.size())) return false;
    const int8_t v = grid.data[idx];
    // Treat unknown (-1) and >50 as not traversable; tweak if you want to allow unknown
    return (v >= 0 && v <= 50);
  };

  auto h = [&](const CellIndex& a, const CellIndex& b) {
    // Euclidean heuristic (admissible)
    const double dx = static_cast<double>(a.x - b.x);
    const double dy = static_cast<double>(a.y - b.y);
    return std::sqrt(dx*dx + dy*dy);
  };

  // --- Start/Goal in grid space ---
  auto maybeStart = worldToCell(robot_pose_.position.x, robot_pose_.position.y);
  auto maybeGoal  = worldToCell(goal_.point.x,                  goal_.point.y);
  if (!maybeStart || !maybeGoal) {
    RCLCPP_WARN(this->get_logger(), "Start or Goal lies outside the map.");
    return;
  }
  CellIndex start = *maybeStart;
  CellIndex goal  = *maybeGoal;

  if (!inBounds(start) || !inBounds(goal) || !isFree(start) || !isFree(goal)) {
    RCLCPP_WARN(this->get_logger(), "Start/Goal is blocked or out of bounds.");
    return;
  }

  // --- A* containers ---
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

  g_score[start] = 0.0;
  open.emplace(start, h(start, goal));

  auto pushOrImprove = [&](const CellIndex& current, const CellIndex& nb, double tentative_g) {
    auto it = g_score.find(nb);
    if (it == g_score.end() || tentative_g < it->second) {
      g_score[nb] = tentative_g;
      came_from[nb] = current;
      const double f = tentative_g + h(nb, goal);
      open.emplace(nb, f);
    }
  };

  // 8-connected moves
  const int DX[8] = {-1,-1,-1, 0, 0, 1, 1, 1};
  const int DY[8] = {-1, 0, 1,-1, 1,-1, 0, 1};
  const double COST[8] = {
    std::sqrt(2.0), 1.0, std::sqrt(2.0),
    1.0,             1.0,
    std::sqrt(2.0), 1.0, std::sqrt(2.0)
  };

  // --- Main A* loop ---
  bool found = false;
  std::unordered_map<CellIndex, bool, CellIndexHash> closed;

  while (!open.empty()) {
    CellIndex cur = open.top().index;
    open.pop();

    if (closed[cur]) continue;       // skip if already expanded
    closed[cur] = true;

    if (cur == goal) {               // reached!
      found = true;
      break;
    }

    for (int k = 0; k < 8; ++k) {
      CellIndex nb(cur.x + DX[k], cur.y + DY[k]);
      if (!inBounds(nb) || !isFree(nb) || closed[nb]) continue;

      // Optional: simple corner-cutting prevention for diagonals
      if (DX[k] != 0 && DY[k] != 0) {
        CellIndex a(cur.x + DX[k], cur.y);
        CellIndex b(cur.x,          cur.y + DY[k]);
        if (!isFree(a) || !isFree(b)) continue;
      }

      const double tentative_g = g_score[cur] + COST[k];
      pushOrImprove(cur, nb, tentative_g);
    }
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "map";

  if (!found) {
    RCLCPP_WARN(this->get_logger(), "A*: No path found.");
    path_pub_->publish(path); // publish empty to clear old
    return;
  }

  // --- Reconstruct path (goal -> start) ---
  std::vector<CellIndex> cells;
  for (CellIndex c = goal; c != start; c = came_from[c]) {
    cells.push_back(c);
  }
  cells.push_back(start);
  std::reverse(cells.begin(), cells.end());

  // --- Convert to PoseStamped sequence (center of each cell) ---
  path.poses.reserve(cells.size());
  for (const auto& c : cells) {
    auto [wx, wy] = cellToWorld(c);
    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    p.pose.position.x = wx;
    p.pose.position.y = wy;
    p.pose.position.z = 0.0;
    p.pose.orientation.w = 1.0; // no heading here
    path.poses.push_back(std::move(p));
  }

  path_pub_->publish(path);
  RCLCPP_INFO(this->get_logger(), "A*: Path with %zu waypoints published.", path.poses.size());
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
