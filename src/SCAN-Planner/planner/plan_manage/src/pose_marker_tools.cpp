#include <memory>
#include <exception>
#include <string>
#include <utility>

#include <QIcon>
#include <QString>
#include <QTimer>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/tool_manager.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace scan_planner
{

class PublishPoseMarkerTool : public rviz_common::Tool
{
public:
  PublishPoseMarkerTool(
    std::string service_name, QString display_name, std::string icon_name,
    QString status_message)
  : service_name_(std::move(service_name)),
    display_name_(std::move(display_name)),
    icon_name_(std::move(icon_name)),
    status_message_(std::move(status_message))
  {
  }

  void onInitialize() override
  {
    setName(display_name_);
    setDescription(status_message_);

    try {
      const auto share_directory = ament_index_cpp::get_package_share_directory("scan_planner");
      setIcon(QIcon(QString::fromStdString(share_directory + "/icons/" + icon_name_)));
    } catch (const std::exception &) {
      // The text and tooltip still make the toolbar tool usable without an icon.
    }

    const auto node_abstraction = context_->getRosNodeAbstraction().lock();
    if (!node_abstraction) {
      setStatus("RViz ROS node is unavailable");
      return;
    }
    node_ = node_abstraction->get_raw_node();
    client_ = node_->create_client<std_srvs::srv::Trigger>(service_name_);
  }

  void activate() override
  {
    if (!client_) {
      setStatus("发布服务尚未初始化");
    } else if (!client_->service_is_ready()) {
      setStatus(QString("服务不可用: %1").arg(QString::fromStdString(service_name_)));
      RCLCPP_WARN(node_->get_logger(), "Pose marker service is unavailable: %s", service_name_.c_str());
    } else {
      client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      setStatus(status_message_);
    }

    // A toolbar click is a one-shot publish action. Return to Interact so the
    // user can immediately drag either marker again.
    QTimer::singleShot(0, [this]() {returnToInteractTool();});
  }

  void deactivate() override {}

private:
  void returnToInteractTool()
  {
    auto * tool_manager = context_->getToolManager();
    if (!tool_manager) {
      return;
    }

    for (int index = 0; index < tool_manager->numTools(); ++index) {
      auto * tool = tool_manager->getTool(index);
      if (tool && tool->getClassId() == "rviz_default_plugins/Interact") {
        tool_manager->setCurrentTool(tool);
        return;
      }
    }

    if (tool_manager->getDefaultTool()) {
      tool_manager->setCurrentTool(tool_manager->getDefaultTool());
    }
  }

  std::string service_name_;
  QString display_name_;
  std::string icon_name_;
  QString status_message_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
};

class PublishInitialPoseTool : public PublishPoseMarkerTool
{
public:
  PublishInitialPoseTool()
  : PublishPoseMarkerTool(
      "/scan_planner_pose_markers/publish_initial_pose", "发布定位",
      "publish_initial_pose.svg", "已请求发布定位初始位姿")
  {
  }
};

class PublishGoalPoseTool : public PublishPoseMarkerTool
{
public:
  PublishGoalPoseTool()
  : PublishPoseMarkerTool(
      "/scan_planner_pose_markers/publish_goal_pose", "发布目标",
      "publish_goal_pose.svg", "已请求发布 SCAN-Planner 目标位姿")
  {
  }
};

}  // namespace scan_planner

PLUGINLIB_EXPORT_CLASS(scan_planner::PublishInitialPoseTool, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(scan_planner::PublishGoalPoseTool, rviz_common::Tool)
