#pragma once

#include <octomap/OcTree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <sstream>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/time.h"
#if __has_include("tf2_geometry_msgs/tf2_geometry_msgs.hpp")
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#endif
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace astar_planner {

constexpr double kInf = std::numeric_limits<double>::infinity();

struct GridKey {
  int x{0}, y{0}, z{0};
  bool operator==(const GridKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
  bool operator!=(const GridKey& other) const {
    return !(*this == other);
  }
};

struct GridKeyHash {
  std::size_t operator()(const GridKey& key) const {
    std::size_t seed = 0;
    auto combine = [&seed](int value) {
      seed ^= std::hash<int>{}(value) + 0x9e3779b97f4a7c15ULL + (seed << 6) +
              (seed >> 2);
    };
    combine(key.x);
    combine(key.y);
    combine(key.z);
    return seed;
  }
};

struct XyKey {
  int x{0}, y{0};
  bool operator==(const XyKey& other) const {
    return x == other.x && y == other.y;
  }
};

struct XyKeyHash {
  std::size_t operator()(const XyKey& key) const {
    std::size_t seed = 0;
    auto combine = [&seed](int value) {
      seed ^= std::hash<int>{}(value) + 0x9e3779b97f4a7c15ULL + (seed << 6) +
              (seed >> 2);
    };
    combine(key.x);
    combine(key.y);
    return seed;
  }
};

struct PathMetrics {
  double length{0.0};
  double min_clearance{kInf};
  double avg_clearance{kInf};
};

struct TraversableSample {
  Eigen::Vector3d point;
  double cost{0.0};
};

struct TraversableLayer {
  double z{0.0};
  double cost{0.0};
};

rclcpp::QoS latchedQos();
std::string trim(const std::string& value);
std::string lower(const std::string& value);
bool isAbsolutePath(const std::string& path);
std::filesystem::path findWorkspaceRoot();
std::string resolveProjectPath(const std::string& raw_path);
double distance3d(const Eigen::Vector3d& a, const Eigen::Vector3d& b);
double pathLength(const std::vector<Eigen::Vector3d>& points);
geometry_msgs::msg::Point toPointMsg(const Eigen::Vector3d& point);
geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);
Eigen::Vector3d transformPoint(
    const geometry_msgs::msg::TransformStamped& transform,
    const Eigen::Vector3d& point);

class AStarPlannerAdapter {
 public:
  AStarPlannerAdapter() = default;
  ~AStarPlannerAdapter() = default;

  bool initialize(const rclcpp::Node::SharedPtr& node);
  bool plan(const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
            std::vector<Eigen::Vector3d>& path);

  bool isTransitionValid(const Eigen::Vector3d& from, const Eigen::Vector3d& to,
                         bool use_3d) const;
  bool validatePath(const std::vector<Eigen::Vector3d>& path) const;
  double queryObstacleDistance(const Eigen::Vector3d& p) const;
  double queryHardCollisionDistance(const Eigen::Vector3d& p) const;

  // 可通行性支持
  void setTraversableSupport(const std::vector<TraversableSample>& points);
  bool needsTraversableSupport() const;
  bool hasTraversableSupport() const;
  void updateActiveTraversableZRange(double start_z, double goal_z,
                                     bool use_3d);

  // 度量与状态
  PathMetrics computeMetrics(const std::vector<Eigen::Vector3d>& path) const;
  bool initialized() const { return initialized_; }
  const std::string& lastError() const { return last_error_; }
  const PathMetrics& lastMetrics() const { return last_metrics_; }
  std::size_t lastExpandedNodes() const { return last_expanded_nodes_; }
  double lastPlanningTimeSec() const { return last_planning_time_sec_; }
  const std::vector<Eigen::Vector3d>& expandedDebugPoints() const {
    return expanded_debug_points_;
  }
  const std::vector<Eigen::Vector3d>& closedDebugPoints() const {
    return closed_debug_points_;
  }
  double resolution() const { return resolution_; }
  double hardMinClearance() const { return hard_min_clearance_; }
  double preferredClearance() const { return preferred_clearance_; }

 private:
  // 参数读取
  void readParameters();
  template <typename T>
  T param(const std::string& name, const T& fallback) const;

  // 地图加载与处理
  void clearMap();
  bool loadOctomap(const std::string& path);
  bool loadPcd(const std::string& path);
  void addOccupiedPoint(const Eigen::Vector3d& point);
  void addOccupiedKey(const GridKey& key);
  void finalizeMap();

  // 坐标变换
  GridKey worldToKey(const Eigen::Vector3d& point) const;
  Eigen::Vector3d keyCenter(const GridKey& key) const;
  Eigen::Vector3d keyToWorld(const GridKey& key, const Eigen::Vector3d& start,
                             const Eigen::Vector3d& goal, bool use_3d) const;
  double terrainZAt(const XyKey& xy, double fallback) const;
  double traversableZAt(const XyKey& xy, double fallback) const;

  // 搜索与状态验证
  void updateSearchBounds(const GridKey& start, const GridKey& goal,
                          bool use_3d);
  bool withinSearchBounds(const GridKey& key, bool use_3d) const;
  bool isStateValid(const Eigen::Vector3d& point, const GridKey& key,
                    bool use_3d) const;
  std::string invalidStateReason(const std::string& label,
                                 const Eigen::Vector3d& point,
                                 const GridKey& key, bool use_3d) const;
  double requiredClearance() const;
  double hardCollisionClearance() const;

  // 可通行性查询
  bool hasTraversableSupportAt(const XyKey& xy) const;
  std::optional<double> traversableCostAt(const XyKey& xy) const;

  // 障碍物查询
  double queryObstacleDistance(const Eigen::Vector3d& p, bool use_3d) const;
  double queryHardCollisionDistance(const Eigen::Vector3d& p,
                                    bool use_3d) const;
  bool isHardObstacleColumn(const XyKey& xy, double path_z) const;

  // 邻居与代价
  const std::vector<GridKey>& neighborOffsets(bool use_3d) const;
  void rebuildNeighborOffsets();
  double heuristicCost(const GridKey& key, const GridKey& goal,
                       bool use_3d) const;
  template <typename Record>
  double transitionCost(const GridKey& current, const GridKey& neighbor,
                        const Eigen::Vector3d& neighbor_point,
                        const Record& current_record, const GridKey& goal,
                        bool use_3d) const;
  double clearanceCost(const Eigen::Vector3d& point, bool use_3d) const;
  double wallClearanceCost(const Eigen::Vector3d& point, bool use_3d) const;
  double tomogramCost(const GridKey& key, bool use_3d) const;

  // 路径重建
  template <typename Records>
  std::vector<Eigen::Vector3d> reconstructPath(const Records& records,
                                               const GridKey& start_key,
                                               GridKey final_key,
                                               const Eigen::Vector3d& start,
                                               const Eigen::Vector3d& goal,
                                               bool use_3d) const;

  // 调试
  void rememberDebugPoint(const GridKey& key, bool use_3d,
                          std::vector<Eigen::Vector3d>& points);

  // --- 成员变量 ---
  rclcpp::Node::SharedPtr node_;
  bool initialized_{false};
  std::string last_error_;
  std::string loaded_map_path_;
  PathMetrics last_metrics_;
  std::size_t last_expanded_nodes_{0};
  double last_planning_time_sec_{0.0};

  // 参数
  std::string map_frame_{"map"}, map_source_{"octomap"};
  std::string octomap_file_{"maps/map_preprocessed.bt"};
  std::string pcd_file_{"maps/map_preprocessed.pcd"};
  double resolution_{0.2};
  std::string planning_mode_{"2.5d"};
  bool unknown_as_occupied_{true};
  double robot_radius_{0.30}, safety_margin_{0.15}, inflation_radius_{0.45};
  double robot_height_min_{0.05}, robot_height_max_{0.70};
  double obstacle_min_relative_z_{0.20}, obstacle_max_relative_z_{1.40};
  bool terrain_following_enabled_{true};
  double default_path_z_{0.0};
  double heuristic_weight_{1.0};
  std::size_t max_iterations_{500000};
  double search_timeout_sec_{5.0};
  int neighbor_mode_{26};
  bool allow_diagonal_{true};
  bool clearance_cost_enabled_{true};
  double hard_min_clearance_{0.35}, preferred_clearance_{0.75},
      clearance_weight_{2.0};
  double path_length_weight_{1.0}, smoothness_weight_{1.0};
  double search_bounds_padding_{2.0};
  bool search_bounds_use_full_map_{false};
  int max_debug_marker_points_{20000};
  int pcd_voxel_min_points_{1};
  int map_distance_max_points_{250000};
  bool require_traversable_support_{true};
  bool enforce_obstacle_clearance_{false};
  double hard_collision_clearance_{0.05};
  int hard_collision_min_column_points_{2};
  double hard_collision_min_vertical_span_{0.20};
  bool wall_clearance_cost_enabled_{true};
  double wall_preferred_clearance_{0.60}, wall_clearance_weight_{10.0};
  double wall_clearance_power_{2.0};
  double tomogram_traversable_cost_threshold_{50.0};
  bool tomogram_z_aware_support_{true};
  double tomogram_support_z_tolerance_{0.50};
  bool tomogram_cost_enabled_{true};
  double tomogram_cost_normalizer_{20.0}, tomogram_cost_weight_{8.0};
  double tomogram_cost_power_{1.5};
  double traversable_support_radius_{0.25}, traversable_neighbor_radius_{0.30};
  int traversable_min_neighbors_{3};

  // 地图数据
  std::unordered_set<GridKey, GridKeyHash> occupied_;
  std::unordered_map<XyKey, std::vector<double>, XyKeyHash> occupied_by_xy_;
  std::vector<Eigen::Vector3d> occupied_centers_;
  std::unordered_map<XyKey, double, XyKeyHash> terrain_min_z_;
  GridKey map_min_, map_max_, search_min_, search_max_;

  // 搜索数据
  std::vector<Eigen::Vector3d> expanded_debug_points_, closed_debug_points_;
  std::vector<GridKey> neighbor_offsets_2d_, neighbor_offsets_3d_;

  // 可通行性数据
  std::unordered_set<XyKey, XyKeyHash> traversable_xy_;
  std::unordered_map<XyKey, double, XyKeyHash> traversable_cost_;
  std::unordered_map<XyKey, std::vector<TraversableLayer>, XyKeyHash>
      traversable_layers_;
  bool traversable_support_loaded_{false};
  bool active_traversable_z_range_enabled_{false};
  double active_traversable_min_z_{-kInf}, active_traversable_max_z_{kInf};
};

class AStarGlobalPlannerNode : public rclcpp::Node {
 public:
  AStarGlobalPlannerNode();
  ~AStarGlobalPlannerNode() = default;

  void start();

  bool planPath(const Eigen::Vector3d& start, const Eigen::Vector3d& goal);

 private:
  // 初始化
  void declareParameters();
  void readParameters();
  void createRosInterfaces();

  // 回调函数
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onStartPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onTomogram(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg,
                  bool force_default_z, const std::string& source);
  void onGoalPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg);

  // 规划逻辑
  bool shouldAcceptGoalRequest(const Eigen::Vector3d& goal,
                               const std::string& source);
  bool planToGoal(const Eigen::Vector3d& goal);
  void maybeReplanFromStartUpdate(const std::string& trigger);
  bool getCurrentStart(Eigen::Vector3d& start);
  void applyStartZOverride(Eigen::Vector3d& start) const;
  bool transformToMap(const Eigen::Vector3d& input, const std::string& frame_id,
                      Eigen::Vector3d& output);

  // 路径后处理
  std::vector<Eigen::Vector3d> postprocessPath(
      const std::vector<Eigen::Vector3d>& raw_path);
  static std::vector<Eigen::Vector3d> removeDuplicates(
      const std::vector<Eigen::Vector3d>& points);
  static std::vector<Eigen::Vector3d> resamplePath(
      const std::vector<Eigen::Vector3d>& points, double resolution);
  std::vector<Eigen::Vector3d> smoothZProfile(
      const std::vector<Eigen::Vector3d>& points) const;
  void enforceZRateLimits(std::vector<Eigen::Vector3d>& points,
                          const std::vector<Eigen::Vector3d>& reference) const;
  double allowedZStep(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const;
  static double maxZStep(const std::vector<Eigen::Vector3d>& points);
  static std::vector<Eigen::Vector3d> smoothPath(
      const std::vector<Eigen::Vector3d>& points, int iterations);

  // 发布与调试
  void publishPath(const std::vector<Eigen::Vector3d>& points);
  void publishDebug(const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
                    const std::vector<Eigen::Vector3d>& path,
                    const PathMetrics& metrics,
                    const std::string& failure_reason);
  void publishDebugMarkers(const std::vector<Eigen::Vector3d>& path);
  visualization_msgs::msg::Marker makePointMarker(
      const std::vector<Eigen::Vector3d>& points,
      const builtin_interfaces::msg::Time& stamp, const std::string& ns, int id,
      double scale, const std_msgs::msg::ColorRGBA& color) const;
  visualization_msgs::msg::MarkerArray makeClearanceMarker(
      const std::vector<Eigen::Vector3d>& path,
      const builtin_interfaces::msg::Time& stamp) const;
  static std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a);
  std::string statusFromFailure(const std::string& error) const;
  void setStatus(const std::string& status);
  void publishStatus();

  // --- 成员变量 ---
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  AStarPlannerAdapter adapter_;

  // 参数
  std::string map_frame_{"map"}, odom_frame_{"odom"}, base_frame_{"base_link"};
  std::string start_source_{"tf"},
      start_pose_topic_{"/astar_global_planner/start_pose"};
  std::string odom_topic_{"/odom"};
  double tf_lookup_timeout_{0.25};
  bool start_z_override_enabled_{false};
  double start_z_override_{0.0};
  bool replan_on_start_update_{true};
  double start_replan_check_frequency_{2.0};
  double start_replan_min_distance_{0.30}, start_replan_min_interval_sec_{1.0};
  double start_replan_goal_tolerance_{0.35};
  double goal_replan_min_distance_{0.10}, goal_replan_min_z_distance_{0.20};
  double goal_replan_min_interval_sec_{0.75};
  std::string goal_pose_topic_{"/goal_pose_3d"},
      goal_point_topic_{"/goal_point_3d"};
  std::string rviz_2d_goal_topic_{"/goal_pose"};
  double default_goal_z_{0.0}, default_path_z_{0.0};
  std::string tomogram_topic_{"/tomogram"};
  double tomogram_traversable_cost_threshold_{50.0};
  bool tomogram_z_aware_support_{true}, tomogram_filter_by_z_{true};
  double tomogram_support_z_tolerance_{0.50};

  // 后处理参数
  bool postprocess_enabled_{true}, enable_path_resampling_{true};
  double path_resample_resolution_{0.2};
  bool enable_z_smoothing_{true};
  int z_smoothing_iterations_{30};
  double z_smoothing_alpha_{0.45};
  double z_max_step_{0.12}, z_max_slope_{0.80};
  bool enable_path_smoothing_{true};
  int smoothing_iterations_{20};
  bool remove_duplicate_points_{true}, validate_after_smoothing_{true};

  // 话题与发布
  std::string publish_path_topic_{"/planned_path"};
  std::string publish_alias_path_topic_{"/path"};
  std::string publish_marker_topic_{"/planned_path_marker"};
  std::string status_topic_{"/astar_global_planner/status"};
  std::string debug_topic_{"/astar_global_planner/debug"};
  std::string expanded_nodes_marker_topic_{"/astar_expanded_nodes_marker"};
  std::string closed_set_marker_topic_{"/astar_closed_set_marker"};
  std::string clearance_marker_topic_{"/astar_clearance_marker"};
  bool publish_debug_markers_{true};
  double marker_line_width_{0.08};

  // 状态
  bool planning_in_progress_{false};
  std::string status_{"IDLE"};
  std::optional<nav_msgs::msg::Odometry> last_odom_;
  std::optional<geometry_msgs::msg::PoseStamped> last_start_pose_;
  std::optional<Eigen::Vector3d> pending_goal_;
  std::optional<Eigen::Vector3d> active_goal_;
  std::optional<Eigen::Vector3d> last_planned_start_;
  std::optional<Eigen::Vector3d> last_goal_request_;
  rclcpp::Time last_start_replan_attempt_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_goal_request_time_{0, 0, RCL_ROS_TIME};

  // ROS 2 接口
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr alias_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      expanded_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      closed_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      clearance_marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      start_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr tomogram_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      goal_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      rviz_goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      goal_point_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr start_replan_timer_;
};

}  // namespace astar_planner
