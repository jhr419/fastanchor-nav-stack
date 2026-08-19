#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"

#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"


class GlobalPathWindowNode : public rclcpp::Node
{
public:
  GlobalPathWindowNode()
  : Node("global_path_window_node"),
    tf_buffer_(this->get_clock())
  {
    global_path_topic_ = this->declare_parameter<std::string>(
      "global_path_topic", "/planned_path");

    local_path_topic_ = this->declare_parameter<std::string>(
      "local_path_topic", "/scan_planner/initial_path");

    robot_frame_ = this->declare_parameter<std::string>(
      "robot_frame", "base_link");

    /*
     * IMPORTANT:
     * Window size is no longer configured independently here.
     *
     * The node polls SCAN-Planner's live ROS2 parameters:
     *
     *   /scan_planner_node:
     *     grid_map.sliding_map_size_x
     *     grid_map.sliding_map_size_y
     *
     * Thus SCAN-Planner remains the single source of truth.
     */
    scan_planner_node_name_ = this->declare_parameter<std::string>(
      "scan_planner_node_name", "/scan_planner_node");

    sliding_map_size_x_parameter_ = this->declare_parameter<std::string>(
      "sliding_map_size_x_parameter", "grid_map.sliding_map_size_x");

    sliding_map_size_y_parameter_ = this->declare_parameter<std::string>(
      "sliding_map_size_y_parameter", "grid_map.sliding_map_size_y");

    parameter_sync_rate_hz_ = this->declare_parameter<double>(
      "parameter_sync_rate_hz", 1.0);

    robot_aligned_window_ = this->declare_parameter<bool>(
      "robot_aligned_window", false);

    update_rate_ = this->declare_parameter<double>(
      "update_rate", 20.0);

    search_backtrack_points_ = this->declare_parameter<int>(
      "search_backtrack_points", 5);

    search_forward_points_ = this->declare_parameter<int>(
      "search_forward_points", 1000);

    reacquire_distance_ = this->declare_parameter<double>(
      "reacquire_distance", 3.0);

    if (update_rate_ <= 0.0) {
      throw std::runtime_error("update_rate must be > 0");
    }

    if (parameter_sync_rate_hz_ <= 0.0) {
      throw std::runtime_error("parameter_sync_rate_hz must be > 0");
    }

    std::string param_service = scan_planner_node_name_;
    if (param_service.empty()) {
      throw std::runtime_error("scan_planner_node_name cannot be empty");
    }

    if (param_service.front() != '/') {
      param_service = "/" + param_service;
    }
    while (param_service.size() > 1 && param_service.back() == '/') {
      param_service.pop_back();
    }
    param_service += "/get_parameters";

    parameter_client_ =
      this->create_client<rcl_interfaces::srv::GetParameters>(
        param_service);

    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

    global_path_sub_ =
      this->create_subscription<nav_msgs::msg::Path>(
        global_path_topic_,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        std::bind(
          &GlobalPathWindowNode::globalPathCallback,
          this,
          std::placeholders::_1));

    local_path_pub_ =
      this->create_publisher<nav_msgs::msg::Path>(
        local_path_topic_,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    const auto path_period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / update_rate_));

    update_timer_ = this->create_wall_timer(
      path_period,
      std::bind(
        &GlobalPathWindowNode::updateLocalPath,
        this));

    const auto parameter_period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / parameter_sync_rate_hz_));

    parameter_sync_timer_ = this->create_wall_timer(
      parameter_period,
      std::bind(
        &GlobalPathWindowNode::syncSlidingWindowParameters,
        this));

    RCLCPP_INFO(this->get_logger(), "Global path window node started");
    RCLCPP_INFO(
      this->get_logger(),
      "  path: %s -> %s",
      global_path_topic_.c_str(),
      local_path_topic_.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "  robot_frame: %s",
      robot_frame_.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "  SCAN parameter source: %s",
      scan_planner_node_name_.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "  window parameters: %s / %s",
      sliding_map_size_x_parameter_.c_str(),
      sliding_map_size_y_parameter_.c_str());
  }


private:
  static bool parameterValueAsDouble(
    const rcl_interfaces::msg::ParameterValue & value,
    double & output)
  {
    if (value.type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
      output = value.double_value;
      return true;
    }

    if (value.type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
      output = static_cast<double>(value.integer_value);
      return true;
    }

    return false;
  }


  void syncSlidingWindowParameters()
  {
    if (!parameter_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Waiting for %s parameter service...",
        scan_planner_node_name_.c_str());
      return;
    }

    bool expected = false;
    if (!parameter_request_in_flight_.compare_exchange_strong(expected, true)) {
      return;
    }

    auto request =
      std::make_shared<rcl_interfaces::srv::GetParameters::Request>();

    request->names = {
      sliding_map_size_x_parameter_,
      sliding_map_size_y_parameter_
    };

    parameter_client_->async_send_request(
      request,
      [this](
        rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future)
      {
        parameter_request_in_flight_.store(false);

        rcl_interfaces::srv::GetParameters::Response::SharedPtr response;
        try {
          response = future.get();
        }
        catch (const std::exception & ex) {
          RCLCPP_WARN(
            this->get_logger(),
            "Failed to read SCAN window parameters: %s",
            ex.what());
          return;
        }

        if (!response || response->values.size() != 2) {
          RCLCPP_WARN(
            this->get_logger(),
            "SCAN parameter response does not contain two values");
          return;
        }

        double x = 0.0;
        double y = 0.0;

        if (!parameterValueAsDouble(response->values[0], x) ||
            !parameterValueAsDouble(response->values[1], y))
        {
          RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "SCAN window parameters must be numeric: %s / %s",
            sliding_map_size_x_parameter_.c_str(),
            sliding_map_size_y_parameter_.c_str());
          return;
        }

        if (x <= 0.0 || y <= 0.0) {
          RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "Invalid SCAN sliding window size: %.3f x %.3f",
            x,
            y);
          return;
        }

        bool changed = false;

        {
          std::lock_guard<std::mutex> lock(window_mutex_);

          changed =
            !have_window_parameters_ ||
            std::abs(x - window_size_x_) > 1e-9 ||
            std::abs(y - window_size_y_) > 1e-9;

          window_size_x_ = x;
          window_size_y_ = y;
          have_window_parameters_ = true;
        }

        if (changed) {
          RCLCPP_INFO(
            this->get_logger(),
            "Synchronized SCAN sliding window: %.3f x %.3f m",
            x,
            y);
        }
      });
  }


  uint64_t computePathSignature(
    const nav_msgs::msg::Path & path) const
  {
    uint64_t hash = 1469598103934665603ULL;

    auto mix = [&hash](int64_t value) {
      hash ^= static_cast<uint64_t>(value);
      hash *= 1099511628211ULL;
    };

    mix(static_cast<int64_t>(path.poses.size()));

    for (const auto & pose : path.poses) {
      const auto & p = pose.pose.position;

      mix(static_cast<int64_t>(std::llround(p.x * 1000.0)));
      mix(static_cast<int64_t>(std::llround(p.y * 1000.0)));
      mix(static_cast<int64_t>(std::llround(p.z * 1000.0)));
    }

    return hash;
  }


  void globalPathCallback(
    const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    const uint64_t new_signature = computePathSignature(*msg);

    std::lock_guard<std::mutex> lock(path_mutex_);

    const bool changed =
      !have_global_path_ ||
      new_signature != path_signature_;

    global_path_ = *msg;
    path_signature_ = new_signature;
    have_global_path_ = true;

    if (changed) {
      last_nearest_index_ = 0;
      have_progress_ = false;

      RCLCPP_INFO(
        this->get_logger(),
        "Received new global path: %zu points, frame=%s",
        msg->poses.size(),
        msg->header.frame_id.c_str());
    }
  }


  bool insideWindow(
    const geometry_msgs::msg::Point & point,
    double robot_x,
    double robot_y,
    double robot_yaw,
    double window_size_x,
    double window_size_y) const
  {
    const double dx = point.x - robot_x;
    const double dy = point.y - robot_y;

    double window_x = dx;
    double window_y = dy;

    if (robot_aligned_window_) {
      const double c = std::cos(robot_yaw);
      const double s = std::sin(robot_yaw);

      window_x = c * dx + s * dy;
      window_y = -s * dx + c * dy;
    }

    constexpr double eps = 1e-6;

    return
      std::abs(window_x) <= window_size_x * 0.5 + eps &&
      std::abs(window_y) <= window_size_y * 0.5 + eps;
  }


  size_t findNearestIndex(
    const nav_msgs::msg::Path & path,
    double robot_x,
    double robot_y,
    size_t previous_index,
    bool have_progress,
    bool & full_reacquired)
  {
    full_reacquired = false;

    if (path.poses.empty()) {
      return 0;
    }

    size_t begin = 0;
    size_t end = path.poses.size();

    if (have_progress) {
      const size_t backtrack =
        static_cast<size_t>(std::max(0, search_backtrack_points_));

      const size_t forward =
        static_cast<size_t>(std::max(1, search_forward_points_));

      begin =
        previous_index > backtrack
        ? previous_index - backtrack
        : 0;

      end =
        std::min(
          path.poses.size(),
          previous_index + forward + 1);
    }

    auto search_range =
      [&](size_t start, size_t finish, double & best_distance_sq) -> size_t
      {
        size_t best_index = start;
        best_distance_sq = std::numeric_limits<double>::max();

        for (size_t i = start; i < finish; ++i) {
          const auto & p = path.poses[i].pose.position;

          const double dx = p.x - robot_x;
          const double dy = p.y - robot_y;
          const double d2 = dx * dx + dy * dy;

          if (d2 < best_distance_sq) {
            best_distance_sq = d2;
            best_index = i;
          }
        }

        return best_index;
      };

    double best_distance_sq = 0.0;

    size_t best_index =
      search_range(begin, end, best_distance_sq);

    if (
      have_progress &&
      std::sqrt(best_distance_sq) > reacquire_distance_)
    {
      best_index =
        search_range(
          0,
          path.poses.size(),
          best_distance_sq);

      full_reacquired = true;
    }

    if (
      have_progress &&
      !full_reacquired &&
      best_index < previous_index)
    {
      best_index = previous_index;
    }

    return best_index;
  }


  void updateLocalPath()
  {
    double window_size_x = 0.0;
    double window_size_y = 0.0;

    {
      std::lock_guard<std::mutex> lock(window_mutex_);

      if (!have_window_parameters_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          3000,
          "No SCAN sliding-window parameters yet; local path publication is waiting.");
        return;
      }

      window_size_x = window_size_x_;
      window_size_y = window_size_y_;
    }

    nav_msgs::msg::Path path;
    uint64_t working_signature = 0;
    size_t previous_index = 0;
    bool have_progress = false;

    {
      std::lock_guard<std::mutex> lock(path_mutex_);

      if (!have_global_path_ || global_path_.poses.empty()) {
        return;
      }

      path = global_path_;
      working_signature = path_signature_;
      previous_index = last_nearest_index_;
      have_progress = have_progress_;
    }

    const std::string path_frame = path.header.frame_id;

    if (path_frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "Global path header.frame_id is empty.");
      return;
    }

    geometry_msgs::msg::TransformStamped tf;

    try {
      tf = tf_buffer_.lookupTransform(
        path_frame,
        robot_frame_,
        tf2::TimePointZero);
    }
    catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Cannot get TF %s <- %s: %s",
        path_frame.c_str(),
        robot_frame_.c_str(),
        ex.what());
      return;
    }

    const double robot_x = tf.transform.translation.x;
    const double robot_y = tf.transform.translation.y;
    const double robot_yaw = tf2::getYaw(tf.transform.rotation);

    bool full_reacquired = false;

    const size_t nearest_index =
      findNearestIndex(
        path,
        robot_x,
        robot_y,
        previous_index,
        have_progress,
        full_reacquired);

    nav_msgs::msg::Path local_path;
    local_path.header.stamp = this->now();
    local_path.header.frame_id = path_frame;

    bool entered_window = false;

    for (size_t i = nearest_index; i < path.poses.size(); ++i) {
      const auto & point = path.poses[i].pose.position;

      const bool inside =
        insideWindow(
          point,
          robot_x,
          robot_y,
          robot_yaw,
          window_size_x,
          window_size_y);

      if (inside) {
        entered_window = true;

        auto pose = path.poses[i];
        pose.header.frame_id = path_frame;
        pose.header.stamp = local_path.header.stamp;

        local_path.poses.push_back(std::move(pose));
      }
      else if (entered_window) {
        break;
      }
    }

    {
      std::lock_guard<std::mutex> lock(path_mutex_);

      if (
        have_global_path_ &&
        path_signature_ == working_signature)
      {
        last_nearest_index_ = nearest_index;
        have_progress_ = true;
      }
    }

    local_path_pub_->publish(local_path);

    if (full_reacquired) {
      RCLCPP_WARN(
        this->get_logger(),
        "Path progress reacquired globally at index %zu.",
        nearest_index);
    }
  }


private:
  std::string global_path_topic_;
  std::string local_path_topic_;
  std::string robot_frame_;

  std::string scan_planner_node_name_;
  std::string sliding_map_size_x_parameter_;
  std::string sliding_map_size_y_parameter_;

  double parameter_sync_rate_hz_{1.0};
  bool robot_aligned_window_{false};
  double update_rate_{20.0};

  int search_backtrack_points_{5};
  int search_forward_points_{1000};
  double reacquire_distance_{3.0};

  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr
    parameter_client_;

  std::atomic<bool> parameter_request_in_flight_{false};

  rclcpp::TimerBase::SharedPtr parameter_sync_timer_;
  rclcpp::TimerBase::SharedPtr update_timer_;

  std::mutex window_mutex_;
  bool have_window_parameters_{false};
  double window_size_x_{0.0};
  double window_size_y_{0.0};

  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr
    global_path_sub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
    local_path_pub_;

  std::mutex path_mutex_;
  nav_msgs::msg::Path global_path_;

  bool have_global_path_{false};
  uint64_t path_signature_{0};
  size_t last_nearest_index_{0};
  bool have_progress_{false};
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GlobalPathWindowNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
