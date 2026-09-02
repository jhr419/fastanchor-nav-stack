#include "astar_global_planner_node.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<astar_planner::AStarGlobalPlannerNode>();
  node->start();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
