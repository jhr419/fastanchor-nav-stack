#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace scan_planner
{

class PoseMarkerServer : public rclcpp::Node
{
public:
  PoseMarkerServer()
  : Node("pose_marker_server")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    marker_namespace_ =
      declare_parameter<std::string>("marker_namespace", "/scan_planner_pose_markers");
    initial_pose_topic_ = declare_parameter<std::string>("initial_pose_topic", "/initialpose");
    goal_pose_topic_ =
      declare_parameter<std::string>("goal_pose_topic", "/move_base_simple/goal");
    reference_path_topic_ =
      declare_parameter<std::string>("reference_path_topic", "/initial_path");
    publish_reference_path_ = declare_parameter<bool>("publish_reference_path", true);

    initial_pose_.orientation.w = 1.0;
    goal_pose_.position.x = declare_parameter<double>("default_goal_x", 1.0);
    goal_pose_.orientation.w = 1.0;

    const auto pose_qos = rclcpp::QoS(1).transient_local().reliable();
    initial_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initial_pose_topic_, pose_qos);
    goal_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      goal_pose_topic_, pose_qos);
    if (publish_reference_path_) {
      reference_path_pub_ = create_publisher<nav_msgs::msg::Path>(
        reference_path_topic_, pose_qos);
    }

    initial_pose_service_ = create_service<std_srvs::srv::Trigger>(
      "/scan_planner_pose_markers/publish_initial_pose",
      std::bind(
        &PoseMarkerServer::publishInitialPose, this, std::placeholders::_1,
        std::placeholders::_2));
    goal_pose_service_ = create_service<std_srvs::srv::Trigger>(
      "/scan_planner_pose_markers/publish_goal_pose",
      std::bind(
        &PoseMarkerServer::publishGoalPose, this, std::placeholders::_1,
        std::placeholders::_2));

    marker_server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
      marker_namespace_, this);
    insertPoseMarker(
      "initial_pose", "定位初始位姿", initial_pose_, 0.10F, 0.55F, 1.0F);
    insertPoseMarker(
      "goal_pose", "SCAN-Planner 目标位姿", goal_pose_, 0.15F, 0.95F, 0.25F);
    marker_server_->applyChanges();

    RCLCPP_INFO(
      get_logger(),
      "Pose markers ready in frame '%s'. Move them with RViz Interact, then use the toolbar "
      "publish buttons.",
      frame_id_.c_str());
  }

private:
  void insertPoseMarker(
    const std::string & name, const std::string & description,
    const geometry_msgs::msg::Pose & pose, float red, float green, float blue)
  {
    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = frame_id_;
    marker.name = name;
    marker.description = description;
    marker.scale = 1.2;
    marker.pose = pose;

    visualization_msgs::msg::InteractiveMarkerControl visual_control;
    visual_control.name = name + "_visual";
    visual_control.always_visible = true;

    visualization_msgs::msg::Marker arrow;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.scale.x = 0.85;
    arrow.scale.y = 0.18;
    arrow.scale.z = 0.18;
    arrow.color.r = red;
    arrow.color.g = green;
    arrow.color.b = blue;
    arrow.color.a = 0.95F;
    arrow.pose.position.z = 0.08;
    arrow.pose.orientation.w = 1.0;
    visual_control.markers.push_back(arrow);

    visualization_msgs::msg::Marker base;
    base.type = visualization_msgs::msg::Marker::CYLINDER;
    base.scale.x = 0.42;
    base.scale.y = 0.42;
    base.scale.z = 0.06;
    base.color.r = red;
    base.color.g = green;
    base.color.b = blue;
    base.color.a = 0.65F;
    base.pose.position.z = 0.03;
    base.pose.orientation.w = 1.0;
    visual_control.markers.push_back(base);
    marker.controls.push_back(visual_control);

    // InteractiveMarker controls operate around their local X axis. Rotate
    // that axis onto world X/Y/Z to expose a complete 6-DOF manipulator.
    constexpr double kHalfSqrtTwo = 0.7071067811865476;
    const auto add_axis_controls =
      [&marker, &name](
      const std::string & axis_name, const geometry_msgs::msg::Quaternion & orientation)
      {
        visualization_msgs::msg::InteractiveMarkerControl move_control;
        move_control.name = name + "_move_" + axis_name;
        move_control.orientation = orientation;
        move_control.orientation_mode =
          visualization_msgs::msg::InteractiveMarkerControl::FIXED;
        move_control.interaction_mode =
          visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        marker.controls.push_back(move_control);

        visualization_msgs::msg::InteractiveMarkerControl rotate_control;
        rotate_control.name = name + "_rotate_" + axis_name;
        rotate_control.orientation = orientation;
        rotate_control.orientation_mode =
          visualization_msgs::msg::InteractiveMarkerControl::FIXED;
        rotate_control.interaction_mode =
          visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        marker.controls.push_back(rotate_control);
      };

    geometry_msgs::msg::Quaternion x_axis;
    x_axis.w = 1.0;
    add_axis_controls("x_roll", x_axis);

    geometry_msgs::msg::Quaternion y_axis;
    y_axis.w = kHalfSqrtTwo;
    y_axis.z = kHalfSqrtTwo;
    add_axis_controls("y_pitch", y_axis);

    geometry_msgs::msg::Quaternion z_axis;
    z_axis.w = kHalfSqrtTwo;
    z_axis.y = -kHalfSqrtTwo;
    add_axis_controls("z_yaw", z_axis);

    marker_server_->insert(
      marker,
      std::bind(&PoseMarkerServer::processFeedback, this, std::placeholders::_1));
  }

  void processFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
  {
    if (!feedback) {
      return;
    }
    if (
      feedback->event_type != visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE &&
      feedback->event_type != visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP)
    {
      return;
    }

    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (feedback->marker_name == "initial_pose") {
      initial_pose_ = feedback->pose;
    } else if (feedback->marker_name == "goal_pose") {
      goal_pose_ = feedback->pose;
    }
  }

  void publishInitialPose(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    geometry_msgs::msg::Pose pose;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      pose = initial_pose_;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped message;
    message.header.frame_id = frame_id_;
    message.header.stamp = now();
    message.pose.pose = pose;
    message.pose.covariance[0] = 0.25;
    message.pose.covariance[7] = 0.25;
    message.pose.covariance[35] = 0.06853891909122467;
    initial_pose_pub_->publish(message);

    response->success = true;
    response->message = "Published initial pose on " + initial_pose_topic_;
    RCLCPP_INFO(
      get_logger(), "Published initial pose [%.3f, %.3f] in %s", pose.position.x,
      pose.position.y, frame_id_.c_str());
  }

  void publishGoalPose(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    geometry_msgs::msg::Pose pose;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      pose = goal_pose_;
    }

    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = frame_id_;
    goal.header.stamp = now();
    goal.pose = pose;
    goal_pose_pub_->publish(goal);

    if (reference_path_pub_) {
      // Standalone mode 3 consumes a Path. Integrated navigation disables this
      // direct path so the goal must pass through the global planner.
      nav_msgs::msg::Path path;
      path.header = goal.header;
      path.poses.push_back(goal);
      reference_path_pub_->publish(path);
    }

    response->success = true;
    response->message = "Published goal on " + goal_pose_topic_;
    if (reference_path_pub_) {
      response->message += " and " + reference_path_topic_;
    }
    RCLCPP_INFO(
      get_logger(), "Published goal pose [%.3f, %.3f] in %s", pose.position.x,
      pose.position.y, frame_id_.c_str());
  }

  std::string frame_id_;
  std::string marker_namespace_;
  std::string initial_pose_topic_;
  std::string goal_pose_topic_;
  std::string reference_path_topic_;
  bool publish_reference_path_{true};

  std::mutex pose_mutex_;
  geometry_msgs::msg::Pose initial_pose_;
  geometry_msgs::msg::Pose goal_pose_;

  std::unique_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr initial_pose_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr goal_pose_service_;
};

}  // namespace scan_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::PoseMarkerServer>());
  rclcpp::shutdown();
  return 0;
}
