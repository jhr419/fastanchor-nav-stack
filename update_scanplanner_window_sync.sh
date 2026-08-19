#!/usr/bin/env bash
set -eo pipefail

WS="${1:-$HOME/workspace/fastanchor-nav-stack}"

echo "============================================================"
echo "SCAN-Planner window parameter sync migration"
echo "Workspace: $WS"
echo "============================================================"

if [ ! -d "$WS" ]; then
  echo "[ERROR] Workspace not found: $WS"
  exit 1
fi

find_pkg_dir() {
  local pkg_name="$1"
  python3 - "$WS" "$pkg_name" <<'PY'
import sys
from pathlib import Path
root = Path(sys.argv[1]).resolve()
pkg = sys.argv[2]
skip = {"build", "install", "log", ".git"}

for package_xml in root.rglob("package.xml"):
    if any(part in skip for part in package_xml.parts):
        continue
    if any(part.startswith(".scan_window_sync_backup_") for part in package_xml.parts):
        continue
    try:
        text = package_xml.read_text(errors="ignore")
    except Exception:
        continue
    if f"<name>{pkg}</name>" in text:
        print(package_xml.parent)
        break
PY
}

SCAN_PKG="$(find_pkg_dir scan_planner)"
NAV_PKG="$(find_pkg_dir navigation_bringup)"

if [ -z "$SCAN_PKG" ]; then
  echo "[ERROR] scan_planner package not found."
  exit 1
fi
if [ -z "$NAV_PKG" ]; then
  echo "[ERROR] navigation_bringup package not found."
  exit 1
fi

CPP="$SCAN_PKG/src/global_path_window_node.cpp"
CMAKE="$SCAN_PKG/CMakeLists.txt"
PKG_XML="$SCAN_PKG/package.xml"

RUN_WINDOWED="$SCAN_PKG/launch/run_windowed.launch.py"
if [ ! -f "$RUN_WINDOWED" ]; then
  RUN_WINDOWED="$(find "$SCAN_PKG" -type f -name run_windowed.launch.py | head -n1 || true)"
fi

NAV_LAUNCH="$NAV_PKG/launch/navigation_system.launch.py"
if [ ! -f "$NAV_LAUNCH" ]; then
  NAV_LAUNCH="$(find "$NAV_PKG" -type f -name navigation_system.launch.py | head -n1 || true)"
fi

for f in "$CPP" "$CMAKE" "$PKG_XML" "$RUN_WINDOWED" "$NAV_LAUNCH"; do
  if [ -z "$f" ] || [ ! -f "$f" ]; then
    echo "[ERROR] Required file not found: $f"
    exit 1
  fi
done

STAMP="$(date +%Y%m%d_%H%M%S)"
BACKUP_DIR="$WS/.scan_window_sync_backup_$STAMP"
mkdir -p "$BACKUP_DIR"

cp "$CPP" "$BACKUP_DIR/global_path_window_node.cpp"
cp "$CMAKE" "$BACKUP_DIR/CMakeLists.txt"
cp "$PKG_XML" "$BACKUP_DIR/package.xml"
cp "$RUN_WINDOWED" "$BACKUP_DIR/run_windowed.launch.py"
cp "$NAV_LAUNCH" "$BACKUP_DIR/navigation_system.launch.py"

echo
echo "[1/6] Existing SCAN window parameters"
echo "------------------------------------------------------------"
grep -R -n \
  --include='*.yaml' --include='*.yml' \
  -E 'sliding_map_size_x|sliding_map_size_y' \
  "$SCAN_PKG" || true

echo
echo "[2/6] Replace global_path_window_node"
echo "------------------------------------------------------------"

cat > "$CPP" <<'CPP'
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
CPP

echo "[OK] Replaced: $CPP"

echo
echo "[3/6] Ensure rcl_interfaces build dependency"
echo "------------------------------------------------------------"

python3 - "$CMAKE" "$PKG_XML" <<'PY'
import re
import sys
from pathlib import Path

cmake = Path(sys.argv[1])
pkgxml = Path(sys.argv[2])

text = cmake.read_text()

if not re.search(r'find_package\s*\(\s*rcl_interfaces\b', text):
    # Insert next to rclcpp if possible; otherwise before generated block.
    m = re.search(r'find_package\s*\(\s*rclcpp\s+REQUIRED\s*\)', text)
    if m:
        text = text[:m.end()] + "\nfind_package(rcl_interfaces REQUIRED)" + text[m.end():]
    else:
        marker = "# BEGIN GLOBAL_PATH_WINDOW_NODE"
        if marker in text:
            text = text.replace(marker, "find_package(rcl_interfaces REQUIRED)\n\n" + marker, 1)
        else:
            raise SystemExit("[ERROR] Cannot place find_package(rcl_interfaces REQUIRED)")

# Add rcl_interfaces to the ament_target_dependencies block for this executable.
pattern = re.compile(
    r'(ament_target_dependencies\s*\(\s*global_path_window_node\b)(.*?)(\n\s*\))',
    re.S,
)
m = pattern.search(text)
if not m:
    raise SystemExit("[ERROR] Cannot find ament_target_dependencies(global_path_window_node ...)")

body = m.group(2)
if not re.search(r'\brcl_interfaces\b', body):
    body = body.rstrip() + "\n  rcl_interfaces"
    text = text[:m.start()] + m.group(1) + body + m.group(3) + text[m.end():]

cmake.write_text(text)

xml = pkgxml.read_text()
dep = "rcl_interfaces"
if not any(
    token in xml
    for token in (
        f"<depend>{dep}</depend>",
        f"<build_depend>{dep}</build_depend>",
        f"<exec_depend>{dep}</exec_depend>",
    )
):
    insert = f"  <depend>{dep}</depend>\n"
    if "<export>" in xml:
        xml = xml.replace("  <export>", insert + "\n  <export>", 1)
    else:
        xml = xml.replace("</package>", insert + "</package>", 1)
    pkgxml.write_text(xml)

print("[OK] rcl_interfaces dependency is present.")
PY

echo
echo "[4/6] Remove duplicate window-size launch arguments"
echo "------------------------------------------------------------"

python3 - "$RUN_WINDOWED" "$NAV_LAUNCH" <<'PY'
import re
import sys
from pathlib import Path

run_path = Path(sys.argv[1])
nav_path = Path(sys.argv[2])


def remove_balanced_call(text: str, function_name: str, arg_name: str) -> str:
    pos = 0
    while True:
        start = text.find(function_name + "(", pos)
        if start < 0:
            return text

        # Determine whether this call declares the target launch argument.
        probe = text[start:start + 300]
        if not re.search(
            rf'{re.escape(function_name)}\(\s*["\']{re.escape(arg_name)}["\']',
            probe,
            re.S,
        ):
            pos = start + len(function_name) + 1
            continue

        depth = 0
        quote = None
        escape = False
        i = start

        while i < len(text):
            ch = text[i]

            if quote:
                if escape:
                    escape = False
                elif ch == "\\":
                    escape = True
                elif ch == quote:
                    quote = None
            else:
                if ch in ("'", '"'):
                    quote = ch
                elif ch == "(":
                    depth += 1
                elif ch == ")":
                    depth -= 1
                    if depth == 0:
                        end = i + 1

                        # Include an immediately following comma.
                        j = end
                        while j < len(text) and text[j] in " \t":
                            j += 1
                        if j < len(text) and text[j] == ",":
                            j += 1

                        # Remove only indentation on the same line before call.
                        line_start = text.rfind("\n", 0, start) + 1
                        prefix = text[line_start:start]
                        if prefix.strip() == "":
                            start_remove = line_start
                        else:
                            start_remove = start

                        # Include one trailing newline.
                        while j < len(text) and text[j] in " \t":
                            j += 1
                        if j < len(text) and text[j] == "\n":
                            j += 1

                        return text[:start_remove] + text[j:]
            i += 1

        raise RuntimeError(f"Unbalanced {function_name} call for {arg_name}")


def remove_variable(text: str, name: str) -> str:
    return re.sub(
        rf'^[ \t]*{re.escape(name)}\s*=\s*LaunchConfiguration\([^\n]*\)\s*\n',
        '',
        text,
        flags=re.M,
    )


def remove_simple_dict_entry(text: str, key: str, value_name: str) -> str:
    return re.sub(
        rf'^[ \t]*["\']{re.escape(key)}["\']\s*:\s*{re.escape(value_name)}\s*,?\s*\n',
        '',
        text,
        flags=re.M,
    )


def remove_parameter_value_entry(text: str, key: str) -> str:
    # Generated form:
    # "window_size_x": ParameterValue(
    #     path_window_size_x,
    #     value_type=float,
    # ),
    lines = text.splitlines(keepends=True)
    out = []
    i = 0

    while i < len(lines):
        if re.search(
            rf'["\']{re.escape(key)}["\']\s*:\s*ParameterValue\s*\(',
            lines[i],
        ):
            depth = lines[i].count("(") - lines[i].count(")")
            i += 1

            while i < len(lines) and depth > 0:
                depth += lines[i].count("(") - lines[i].count(")")
                i += 1

            continue

        out.append(lines[i])
        i += 1

    return "".join(out)


run = run_path.read_text()

for name in ("path_window_size_x", "path_window_size_y"):
    run = remove_variable(run, name)
    run = remove_balanced_call(run, "DeclareLaunchArgument", name)

run = remove_parameter_value_entry(run, "window_size_x")
run = remove_parameter_value_entry(run, "window_size_y")

# Also handle possible single-line dictionary variants.
run = re.sub(
    r'^[ \t]*["\']window_size_[xy]["\']\s*:\s*[^,\n]+,?\s*\n',
    '',
    run,
    flags=re.M,
)

run_path.write_text(run)

nav = nav_path.read_text()

# Always make the full navigation bringup use the windowed SCAN launch.
nav = nav.replace(
    '"run.launch.py"',
    '"run_windowed.launch.py"',
)
nav = nav.replace(
    "'run.launch.py'",
    "'run_windowed.launch.py'",
)

for name in ("path_window_size_x", "path_window_size_y"):
    nav = remove_variable(nav, name)
    nav = remove_balanced_call(nav, "DeclareLaunchArgument", name)
    nav = remove_simple_dict_entry(nav, name, name)

# Ensure the SCAN include receives the cropped-path output topic.
# Hard-code only the interface topic here; the window dimensions themselves
# are NOT duplicated and are read live from /scan_planner_node.
if not re.search(
    r'^[ \t]*["\']scan_initial_path_topic["\']\s*:',
    nav,
    re.M,
):
    pattern = re.compile(
        r'(^[ \t]*["\']global_path_topic["\']\s*:\s*global_path_topic\s*,\s*$)',
        re.M,
    )
    m = pattern.search(nav)
    if not m:
        raise SystemExit(
            "[ERROR] Cannot find SCAN include global_path_topic entry "
            "in navigation_system.launch.py"
        )
    indent = re.match(r'^[ \t]*', m.group(1)).group(0)
    insertion = (
        m.group(1)
        + "\n"
        + indent
        + '"scan_initial_path_topic": "/scan_planner/initial_path",'
    )
    nav = nav[:m.start()] + insertion + nav[m.end():]

# Ensure the cropper uses the same robot frame selected by navigation_bringup.
if not re.search(
    r'^[ \t]*["\']path_cropper_robot_frame["\']\s*:',
    nav,
    re.M,
):
    pattern = re.compile(
        r'(^[ \t]*["\']scan_initial_path_topic["\']\s*:\s*[^,\n]+,\s*$)',
        re.M,
    )
    m = pattern.search(nav)
    if not m:
        raise SystemExit(
            "[ERROR] Cannot insert path_cropper_robot_frame into SCAN include"
        )
    indent = re.match(r'^[ \t]*', m.group(1)).group(0)
    insertion = (
        m.group(1)
        + "\n"
        + indent
        + '"path_cropper_robot_frame": base_frame,'
    )
    nav = nav[:m.start()] + insertion + nav[m.end():]

nav_path.write_text(nav)

print("[OK] Removed duplicated path_window_size_x/y launch configuration.")
print("[OK] navigation_system.launch.py uses run_windowed.launch.py.")
print("[OK] cropper robot frame follows navigation_bringup base_frame.")
PY

python3 -m py_compile "$RUN_WINDOWED"
python3 -m py_compile "$NAV_LAUNCH"

echo "[OK] Launch Python syntax checks passed."

echo
echo "[5/6] Build"
echo "------------------------------------------------------------"

cd "$WS"

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "[ERROR] ROS2 Humble setup not found: /opt/ros/humble/setup.bash"
  exit 1
fi

# Intentionally do not use `set -u`; ROS setup scripts may reference unset vars.
source /opt/ros/humble/setup.bash

if [ -f "$WS/install/setup.bash" ]; then
  source "$WS/install/setup.bash"
fi

colcon build \
  --symlink-install \
  --packages-select scan_planner navigation_bringup

source "$WS/install/setup.bash"

echo
echo "[6/6] Verify installation"
echo "------------------------------------------------------------"

if ros2 pkg executables scan_planner | grep -q 'global_path_window_node'; then
  echo "[OK] scan_planner/global_path_window_node is installed."
else
  echo "[ERROR] global_path_window_node is not visible."
  exit 1
fi

if ros2 launch navigation_bringup navigation_system.launch.py --show-args >/tmp/navigation_system_show_args.txt 2>&1; then
  echo "[OK] navigation_system.launch.py can be parsed."
else
  echo "[WARN] ros2 launch --show-args reported a problem:"
  cat /tmp/navigation_system_show_args.txt
  exit 1
fi

if grep -qE 'path_window_size_x|path_window_size_y' /tmp/navigation_system_show_args.txt; then
  echo "[ERROR] Old duplicated window-size launch arguments still exist."
  exit 1
else
  echo "[OK] No duplicated path_window_size_x/y launch arguments remain."
fi

echo
echo "============================================================"
echo "DONE"
echo "============================================================"
echo "Backup:"
echo "  $BACKUP_DIR"
echo
echo "The cropper now reads live parameters from:"
echo "  /scan_planner_node -> grid_map.sliding_map_size_x"
echo "  /scan_planner_node -> grid_map.sliding_map_size_y"
echo
echo "Start the full navigation system with the SAME command:"
echo
echo "  cd $WS"
echo "  source /opt/ros/humble/setup.bash"
echo "  source install/setup.bash"
echo "  ros2 launch navigation_bringup navigation_system.launch.py \\"
echo '    map_pcd_path:=$PWD/maps/map_preprocessed2.pcd \'
echo "    local_target_distance:=3.0 \\"
echo "    lidar_model:=mid360"
echo
echo "After startup verify:"
echo
echo "  ros2 param get /scan_planner_node grid_map.sliding_map_size_x"
echo "  ros2 param get /scan_planner_node grid_map.sliding_map_size_y"
echo "  ros2 topic info /scan_planner/initial_path"
echo "  ros2 topic hz /scan_planner/initial_path"
echo
echo "Expected cropper log for your current config:"
echo "  Synchronized SCAN sliding window: 10.000 x 10.000 m"
