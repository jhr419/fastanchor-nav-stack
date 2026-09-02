#include <exception>
#include <memory>

#include "genisom_l1_control/manager_node.hpp"
#include "genisom_l1_control/process_supervisor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

int run_manager_node(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<genisom_l1_control::ManagerNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("genisom_control"), "节点启动失败: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

}  // 匿名命名空间

int main(int argc, char * argv[])
{
  return genisom_l1_control::run_with_shutdown_supervisor(argc, argv, run_manager_node);
}
