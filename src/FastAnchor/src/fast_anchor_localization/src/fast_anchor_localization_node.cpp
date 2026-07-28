#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "fast_anchor_interfaces/msg/icp_result.hpp"
#include "fast_anchor_interfaces/msg/localization_status.hpp"
#include "fast_anchor_interfaces/srv/relocalize.hpp"
#include "fast_anchor_interfaces/srv/reset_localization.hpp"

namespace
{

using PointType = pcl::PointXYZI;

Eigen::Affine3d poseToAffine(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Translation3d translation(
    pose.position.x,
    pose.position.y,
    pose.position.z);
  Eigen::Quaterniond rotation(
    pose.orientation.w,
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z);
  return translation * rotation.normalized();
}

geometry_msgs::msg::Pose affineToPose(const Eigen::Affine3d & affine)
{
  geometry_msgs::msg::Pose pose;
  const Eigen::Vector3d t = affine.translation();
  const Eigen::Quaterniond q(affine.rotation());
  pose.position.x = t.x();
  pose.position.y = t.y();
  pose.position.z = t.z();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

geometry_msgs::msg::TransformStamped affineToTransform(
  const builtin_interfaces::msg::Time & stamp,
  const std::string & parent_frame,
  const std::string & child_frame,
  const Eigen::Affine3d & affine)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = parent_frame;
  transform.child_frame_id = child_frame;
  const auto pose = affineToPose(affine);
  transform.transform.translation.x = pose.position.x;
  transform.transform.translation.y = pose.position.y;
  transform.transform.translation.z = pose.position.z;
  transform.transform.rotation = pose.orientation;
  return transform;
}

Eigen::Affine3d makeTransform(
  const std::vector<double> & xyz,
  const std::vector<double> & rpy)
{
  const double x = xyz.size() > 0 ? xyz[0] : 0.0;
  const double y = xyz.size() > 1 ? xyz[1] : 0.0;
  const double z = xyz.size() > 2 ? xyz[2] : 0.0;
  const double roll = rpy.size() > 0 ? rpy[0] : 0.0;
  const double pitch = rpy.size() > 1 ? rpy[1] : 0.0;
  const double yaw = rpy.size() > 2 ? rpy[2] : 0.0;
  return Eigen::Translation3d(x, y, z) *
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
}

void downsample(
  pcl::PointCloud<PointType>::Ptr cloud,
  const double leaf_size)
{
  if (!cloud || cloud->empty() || leaf_size <= 0.0) {
    return;
  }

  pcl::VoxelGrid<PointType> voxel;
  voxel.setLeafSize(
    static_cast<float>(leaf_size),
    static_cast<float>(leaf_size),
    static_cast<float>(leaf_size));
  voxel.setInputCloud(cloud);
  pcl::PointCloud<PointType> filtered;
  voxel.filter(filtered);
  *cloud = filtered;
}

}  // namespace

class FastAnchorLocalizationNode : public rclcpp::Node
{
public:
  FastAnchorLocalizationNode()
  : Node("fast_anchor_localization_node")
  {
    map_pcd_path_ = declare_parameter<std::string>("map_pcd_path", "");
    icp_map_pcd_path_ = declare_parameter<std::string>("icp_map_pcd_path", map_pcd_path_);
    visualization_map_pcd_path_ =
      declare_parameter<std::string>("visualization_map_pcd_path", map_pcd_path_);
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/cloud_registered_body");
    initialpose_topic_ = declare_parameter<std::string>("initialpose_topic", "/initialpose");
    reset_service_name_ = declare_parameter<std::string>("reset_service_name", "reset_localization");
    fastlio_reset_service_ =
      declare_parameter<std::string>("fastlio_reset_service", "/fastlio_reset");
    fastlio_reset_timeout_s_ = declare_parameter<double>("fastlio_reset_timeout_s", 1.0);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "camera_init");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    publish_base_tf_ = declare_parameter<bool>("publish_base_tf", true);
    odom_coincident_with_base_ = declare_parameter<bool>("odom_coincident_with_base", false);
    publish_map_cloud_ = declare_parameter<bool>("publish_map_cloud", true);

    pose_topic_ = declare_parameter<std::string>("pose_topic", "icp_pose");
    odom_output_topic_ = declare_parameter<std::string>("odom_output_topic", "icp_odom");
    path_topic_ = declare_parameter<std::string>("path_topic", "icp_path");
    status_topic_ = declare_parameter<std::string>("status_topic", "icp_status");
    icp_result_topic_ = declare_parameter<std::string>("icp_result_topic", "icp_result");
    aligned_cloud_topic_ = declare_parameter<std::string>("aligned_cloud_topic", "icp_aligned_cloud");
    icp_map_topic_ = declare_parameter<std::string>("icp_map_topic", "icp_map");
    visualization_map_topic_ =
      declare_parameter<std::string>("visualization_map_topic", "icp_visualization_map");
    relocalize_service_name_ =
      declare_parameter<std::string>("relocalize_service_name", "fast_anchor_relocalize");
    reset_localization_service_name_ =
      declare_parameter<std::string>("reset_localization_service_name", "fast_anchor_reset_localization");
    max_path_size_ = declare_parameter<int>("max_path_size", 10000);

    min_range_ = declare_parameter<double>("min_range", 0.5);
    max_range_ = declare_parameter<double>("max_range", 60.0);
    scan_leaf_size_ = declare_parameter<double>("scan_leaf_size", 0.25);
    map_leaf_size_ = declare_parameter<double>("map_leaf_size", 0.25);
    min_scan_points_ = declare_parameter<int>("min_scan_points", 120);
    source_non_ground_min_z_ = declare_parameter<double>("source_non_ground_min_z", -0.2);
    source_non_ground_filter_en_ = declare_parameter<bool>("source_non_ground_filter_en", true);
    max_correspondence_distance_ = declare_parameter<double>("max_correspondence_distance", 1.5);
    transformation_epsilon_ = declare_parameter<double>("transformation_epsilon", 0.01);
    euclidean_fitness_epsilon_ = declare_parameter<double>("euclidean_fitness_epsilon", 0.01);
    max_iterations_ = declare_parameter<int>("max_iterations", 40);
    fitness_score_threshold_ = declare_parameter<double>("fitness_score_threshold", 1.0);
    relocalization_interval_s_ = declare_parameter<double>("relocalization_interval_s", 0.2);
    aligned_cloud_interval_s_ = declare_parameter<double>("aligned_cloud_interval_s", 0.0);
    path_publish_interval_s_ = declare_parameter<double>("path_publish_interval_s", 0.0);
    use_initial_pose_param_ = declare_parameter<bool>("use_initial_pose_param", false);

    const auto base_to_body_xyz =
      declare_parameter<std::vector<double>>("base_to_body_xyz", {0.0, 0.0, 0.0});
    const auto base_to_body_rpy =
      declare_parameter<std::vector<double>>("base_to_body_rpy", {0.0, 0.0, 0.0});
    const auto initial_pose_xyz =
      declare_parameter<std::vector<double>>("initial_pose_xyz", {0.0, 0.0, 0.0});
    const auto initial_pose_rpy =
      declare_parameter<std::vector<double>>("initial_pose_rpy", {0.0, 0.0, 0.0});

    map_frame_ = declare_parameter<std::string>("frames.map_frame", map_frame_);
    odom_frame_ = declare_parameter<std::string>("frames.odom_frame", odom_frame_);
    base_frame_ = declare_parameter<std::string>("frames.base_frame", base_frame_);
    lidar_frame_ = declare_parameter<std::string>("frames.lidar_frame", "livox_frame");

    odom_topic_ = declare_parameter<std::string>("topics.fast_lio_odom", odom_topic_);
    scan_topic_ = declare_parameter<std::string>("topics.fast_lio_cloud", scan_topic_);
    initialpose_topic_ = declare_parameter<std::string>("topics.initial_pose", initialpose_topic_);
    pose_topic_ = declare_parameter<std::string>("topics.output_pose", pose_topic_);
    odom_output_topic_ = declare_parameter<std::string>("topics.output_odom", odom_output_topic_);
    path_topic_ = declare_parameter<std::string>("topics.output_path", path_topic_);
    status_topic_ = declare_parameter<std::string>("topics.output_status", status_topic_);
    icp_result_topic_ = declare_parameter<std::string>("topics.icp_result", icp_result_topic_);
    aligned_cloud_topic_ = declare_parameter<std::string>("topics.aligned_cloud", aligned_cloud_topic_);
    icp_map_topic_ = declare_parameter<std::string>("topics.local_map", icp_map_topic_);
    visualization_map_topic_ = declare_parameter<std::string>("topics.global_map", visualization_map_topic_);

    map_pcd_path_ = declare_parameter<std::string>("map.pcd_path", map_pcd_path_);
    icp_map_pcd_path_ = declare_parameter<std::string>("map.icp_pcd_path", icp_map_pcd_path_);
    visualization_map_pcd_path_ =
      declare_parameter<std::string>("map.visualization_pcd_path", visualization_map_pcd_path_);
    map_leaf_size_ = declare_parameter<double>("map.voxel_leaf_size", map_leaf_size_);
    publish_map_cloud_ = declare_parameter<bool>("map.publish_global_map", publish_map_cloud_);

    scan_leaf_size_ = declare_parameter<double>("cloud_preprocess.voxel_leaf_size", scan_leaf_size_);
    min_range_ = declare_parameter<double>("cloud_preprocess.min_range", min_range_);
    max_range_ = declare_parameter<double>("cloud_preprocess.max_range", max_range_);
    source_non_ground_filter_en_ =
      declare_parameter<bool>("cloud_preprocess.enable_height_filter", source_non_ground_filter_en_);
    source_non_ground_min_z_ =
      declare_parameter<double>("cloud_preprocess.min_z", source_non_ground_min_z_);

    max_iterations_ = declare_parameter<int>("icp.max_iterations", max_iterations_);
    max_correspondence_distance_ =
      declare_parameter<double>("icp.max_correspondence_distance", max_correspondence_distance_);
    transformation_epsilon_ =
      declare_parameter<double>("icp.transformation_epsilon", transformation_epsilon_);
    euclidean_fitness_epsilon_ =
      declare_parameter<double>("icp.euclidean_fitness_epsilon", euclidean_fitness_epsilon_);
    fitness_score_threshold_ =
      declare_parameter<double>("icp.fitness_score_threshold", fitness_score_threshold_);
    relocalization_interval_s_ =
      declare_parameter<double>("icp.relocalization_interval_s", relocalization_interval_s_);
    aligned_cloud_interval_s_ =
      declare_parameter<double>("output.aligned_cloud_interval_s", aligned_cloud_interval_s_);
    path_publish_interval_s_ =
      declare_parameter<double>("output.path_publish_interval_s", path_publish_interval_s_);

    base_to_body_ = makeTransform(base_to_body_xyz, base_to_body_rpy);
    body_to_base_ = base_to_body_.inverse();
    initial_map_to_base_ = makeTransform(initial_pose_xyz, initial_pose_rpy);

    // 如果icp_map_pcd_path_和visualization_map_pcd_path_没有单独设置，就使用map_pcd_path_。这样用户只需要提供一个地图文件即可满足ICP和可视化的需求。
    if (icp_map_pcd_path_.empty()) {
      icp_map_pcd_path_ = map_pcd_path_;
    }
    if (visualization_map_pcd_path_.empty()) {
      visualization_map_pcd_path_ = map_pcd_path_;
    }

    icp_map_cloud_ = loadMapCloud(icp_map_pcd_path_);
    if (!icp_map_cloud_ || icp_map_cloud_->empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to load ICP target map PCD: '%s'. Set icp_map_pcd_path to the non-ground map.",
        icp_map_pcd_path_.c_str());
    } else {
      downsample(icp_map_cloud_, map_leaf_size_);
      icp_.setInputTarget(icp_map_cloud_);
      RCLCPP_INFO(
        get_logger(),
        "Loaded ICP target map: %s points=%zu leaf=%.3f",
        icp_map_pcd_path_.c_str(), icp_map_cloud_->size(), map_leaf_size_);
    }

    if (visualization_map_pcd_path_ == icp_map_pcd_path_) {
      visualization_map_cloud_ = icp_map_cloud_;
      RCLCPP_INFO(get_logger(), "Reusing the ICP map for visualization; no duplicate PCD copy was loaded.");
    } else {
      visualization_map_cloud_ = loadMapCloud(visualization_map_pcd_path_);
      if (!visualization_map_cloud_ || visualization_map_cloud_->empty()) {
        RCLCPP_WARN(
          get_logger(),
          "Visualization map PCD is empty: '%s'. Falling back to ICP target map for display.",
          visualization_map_pcd_path_.c_str());
        visualization_map_cloud_ = icp_map_cloud_;
      } else {
        downsample(visualization_map_cloud_, map_leaf_size_);
        RCLCPP_INFO(
          get_logger(),
          "Loaded visualization map: %s points=%zu leaf=%.3f",
          visualization_map_pcd_path_.c_str(), visualization_map_cloud_->size(), map_leaf_size_);
      }
    }

    icp_.setMaximumIterations(max_iterations_);
    icp_.setMaxCorrespondenceDistance(max_correspondence_distance_);
    icp_.setTransformationEpsilon(transformation_epsilon_);
    icp_.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 50,
      std::bind(&FastAnchorLocalizationNode::odomCallback, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FastAnchorLocalizationNode::scanCallback, this, std::placeholders::_1));
    initialpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initialpose_topic_, 5,
      std::bind(&FastAnchorLocalizationNode::initialPoseCallback, this, std::placeholders::_1));

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic_, 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_output_topic_, 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
    status_pub_ =
      create_publisher<fast_anchor_interfaces::msg::LocalizationStatus>(status_topic_, 10);
    icp_result_pub_ =
      create_publisher<fast_anchor_interfaces::msg::IcpResult>(icp_result_topic_, 10);
    aligned_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(aligned_cloud_topic_, 2);
    icp_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      icp_map_topic_, rclcpp::QoS(1).transient_local().reliable());
    visualization_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      visualization_map_topic_, rclcpp::QoS(1).transient_local().reliable());
    reset_srv_ = create_service<std_srvs::srv::Trigger>(
      reset_service_name_,
      std::bind(
        &FastAnchorLocalizationNode::resetCallback, this,
        std::placeholders::_1, std::placeholders::_2));
    relocalize_srv_ = create_service<fast_anchor_interfaces::srv::Relocalize>(
      relocalize_service_name_,
      std::bind(
        &FastAnchorLocalizationNode::relocalizeCallback, this,
        std::placeholders::_1, std::placeholders::_2));
    reset_localization_srv_ = create_service<fast_anchor_interfaces::srv::ResetLocalization>(
      reset_localization_service_name_,
      std::bind(
        &FastAnchorLocalizationNode::resetLocalizationCallback, this,
        std::placeholders::_1, std::placeholders::_2));
    fastlio_reset_client_ = create_client<std_srvs::srv::Trigger>(fastlio_reset_service_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    if (publish_map_cloud_) {
      map_publish_timer_ = create_wall_timer(
        std::chrono::milliseconds(250),
        std::bind(&FastAnchorLocalizationNode::publishMapCloudOnce, this));
    }

    RCLCPP_INFO(
      get_logger(),
      "ICP localization waiting for odom=%s scan=%s initialpose=%s",
      odom_topic_.c_str(), scan_topic_.c_str(), initialpose_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Publishing TF %s -> %s -> %s, odom_coincident_with_base=%s",
      map_frame_.c_str(),
      odom_frame_.c_str(),
      base_frame_.c_str(),
      odom_coincident_with_base_ ? "true" : "false");
    RCLCPP_INFO(
      get_logger(),
      "Localization reset service ready: %s, FAST-LIO reset target: %s",
      reset_service_name_.c_str(),
      fastlio_reset_service_.c_str());
    publishStatus(
      now(), fast_anchor_interfaces::msg::LocalizationStatus::UNINITIALIZED,
      "waiting for map, odometry, and initial pose", false, -1.0);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_odom_stamp_ = msg->header.stamp;
    source_odom_frame_ = msg->header.frame_id;
    if (!source_odom_frame_.empty() && source_odom_frame_ != odom_frame_) {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "Odometry header frame is '%s', but TF odom_frame parameter is '%s'. Publishing TF with parameter frame name.",
        source_odom_frame_.c_str(),
        odom_frame_.c_str());
    }
    latest_odom_to_body_ = poseToAffine(msg->pose.pose);
    latest_odom_to_base_ = latest_odom_to_body_ * body_to_base_;
    if (!have_odom_) {
      RCLCPP_INFO(get_logger(), "Received first odometry from %s.", odom_topic_.c_str());
    }
    have_odom_ = true;

    if (!have_initial_pose_ && use_initial_pose_param_) {
      map_to_odom_ = initial_map_to_base_ * latest_odom_to_base_.inverse();
      have_initial_pose_ = true;
      RCLCPP_INFO(get_logger(), "Initialized map->odom from initial_pose_* parameters.");
    } else if (!have_initial_pose_ && have_pending_initial_pose_) {
      applyInitialPoseLocked(pending_initial_map_to_base_);
      RCLCPP_INFO(get_logger(), "Applied cached initial pose after odometry became available.");
    }
    publishStatus(
      msg->header.stamp,
      have_initial_pose_ ?
      fast_anchor_interfaces::msg::LocalizationStatus::TRACKING :
      fast_anchor_interfaces::msg::LocalizationStatus::INITIALIZING,
      have_initial_pose_ ? "odometry ready" : "waiting for initial pose",
      last_icp_converged_, last_icp_fitness_score_);
  }

  pcl::PointCloud<PointType>::Ptr loadMapCloud(const std::string & path) const
  {
    if (path.empty()) {
      return pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    }

    // Existing maps may come from FAST-LIO, GLIM, or CloudCompare and often
    // do not expose a standard "intensity" field. ICP only needs xyz.
    pcl::PointCloud<pcl::PointXYZ> xyz_cloud;
    if (pcl::io::loadPCDFile(path, xyz_cloud) < 0) {
      return pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    }

    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    cloud->reserve(xyz_cloud.size());
    for (const auto & p_xyz : xyz_cloud.points) {
      if (!std::isfinite(p_xyz.x) || !std::isfinite(p_xyz.y) || !std::isfinite(p_xyz.z)) {
        continue;
      }
      PointType point;
      point.x = p_xyz.x;
      point.y = p_xyz.y;
      point.z = p_xyz.z;
      point.intensity = 0.0F;
      cloud->push_back(point);
    }
    return cloud;
  }

  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const Eigen::Affine3d map_to_base = poseToAffine(msg->pose.pose);
    if (!have_odom_) {
      pending_initial_map_to_base_ = map_to_base;
      have_pending_initial_pose_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Cached initial pose in frame '%s'; waiting for odometry on %s before starting ICP.",
        msg->header.frame_id.c_str(), odom_topic_.c_str());
      return;
    }

    applyInitialPoseLocked(map_to_base);
    RCLCPP_INFO(
      get_logger(),
      "Accepted initial pose in frame '%s'; ICP relocalization is enabled.",
      msg->header.frame_id.c_str());
  }

  void applyInitialPoseLocked(const Eigen::Affine3d & map_to_base)
  {
    map_to_odom_ = map_to_base * latest_odom_to_base_.inverse();
    have_initial_pose_ = true;
    have_pending_initial_pose_ = false;
  }

  void resetCallback(
    const std_srvs::srv::Trigger::Request::ConstSharedPtr,
    const std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    resetLocalizationState();
    const bool fastlio_reset_requested = requestFastlioReset();

    response->success = fastlio_reset_requested;
    response->message = fastlio_reset_requested ?
      "Localization state was reset and FAST-LIO reset was requested. Select a new initial pose on /initialpose." :
      "Localization state was reset, but FAST-LIO reset service was unavailable. Select a new initial pose on /initialpose after FAST-LIO is reset.";

    if (fastlio_reset_requested) {
      RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    }
  }

  void relocalizeCallback(
    const fast_anchor_interfaces::srv::Relocalize::Request::ConstSharedPtr request,
    const fast_anchor_interfaces::srv::Relocalize::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const Eigen::Affine3d map_to_base = poseToAffine(request->initial_pose.pose.pose);
    if (!have_odom_) {
      pending_initial_map_to_base_ = map_to_base;
      have_pending_initial_pose_ = true;
      response->success = true;
      response->message = "Initial pose cached; waiting for FAST-LIO odometry.";
      response->fitness_score = last_icp_fitness_score_;
      publishStatus(
        request->initial_pose.header.stamp,
        fast_anchor_interfaces::msg::LocalizationStatus::WAITING_FOR_LIO,
        response->message, false, last_icp_fitness_score_);
      return;
    }

    applyInitialPoseLocked(map_to_base);
    response->success = true;
    response->message = "Initial pose accepted; ICP relocalization is enabled.";
    response->fitness_score = last_icp_fitness_score_;
    publishStatus(
      request->initial_pose.header.stamp,
      fast_anchor_interfaces::msg::LocalizationStatus::RELOCALIZING,
      response->message, last_icp_converged_, last_icp_fitness_score_);
  }

  void resetLocalizationCallback(
    const fast_anchor_interfaces::srv::ResetLocalization::Request::ConstSharedPtr request,
    const fast_anchor_interfaces::srv::ResetLocalization::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    resetLocalizationStateLocked();
    if (request->reset_to_initial_pose && use_initial_pose_param_ && have_odom_) {
      map_to_odom_ = initial_map_to_base_ * latest_odom_to_base_.inverse();
      have_initial_pose_ = true;
      response->message = "Localization reset to initial_pose_* parameters.";
    } else {
      response->message = "Localization state reset; publish /initialpose before ICP resumes.";
    }
    response->success = true;
    publishStatus(
      now(), fast_anchor_interfaces::msg::LocalizationStatus::UNINITIALIZED,
      response->message, false, -1.0);
  }

  void resetLocalizationState()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    resetLocalizationStateLocked();
  }

  void resetLocalizationStateLocked()
  {
    map_to_odom_ = Eigen::Affine3d::Identity();
    pending_initial_map_to_base_ = Eigen::Affine3d::Identity();
    have_initial_pose_ = false;
    have_pending_initial_pose_ = false;
    last_icp_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_aligned_cloud_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_path_publish_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_icp_fitness_score_ = -1.0;
    last_icp_converged_ = false;
    path_msg_.poses.clear();
  }

  bool requestFastlioReset()
  {
    if (!fastlio_reset_client_) {
      return false;
    }

    const auto timeout = std::chrono::duration<double>(fastlio_reset_timeout_s_);
    if (!fastlio_reset_client_->wait_for_service(
        std::chrono::duration_cast<std::chrono::nanoseconds>(timeout)))
    {
      return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    fastlio_reset_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        const auto response = future.get();
        if (response && response->success) {
          RCLCPP_INFO(get_logger(), "FAST-LIO reset completed: %s", response->message.c_str());
        } else {
          RCLCPP_WARN(
            get_logger(),
            "FAST-LIO reset failed: %s",
            response ? response->message.c_str() : "no response");
        }
      });
    return true;
  }

  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_scan_) {
      RCLCPP_INFO(get_logger(), "Received first scan from %s.", scan_topic_.c_str());
      have_scan_ = true;
    }
    if (!icp_map_cloud_ || icp_map_cloud_->empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "ICP map is empty; cannot localize.");
      publishStatus(
        msg->header.stamp, fast_anchor_interfaces::msg::LocalizationStatus::WAITING_FOR_MAP,
        "ICP map is empty", false, last_icp_fitness_score_);
      return;
    }
    if (!have_odom_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Waiting for odometry on %s.", odom_topic_.c_str());
      publishStatus(
        msg->header.stamp, fast_anchor_interfaces::msg::LocalizationStatus::WAITING_FOR_LIO,
        "waiting for FAST-LIO odometry", false, last_icp_fitness_score_);
      return;
    }
    if (!have_initial_pose_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Waiting for initial pose on %s.", initialpose_topic_.c_str());
      publishStatus(
        msg->header.stamp, fast_anchor_interfaces::msg::LocalizationStatus::INITIALIZING,
        "waiting for initial pose", false, last_icp_fitness_score_);
      return;
    }

    const Eigen::Affine3d guess_map_to_base = map_to_odom_ * latest_odom_to_base_;
    const rclcpp::Time stamp(msg->header.stamp);
    const double since_last_icp = (stamp - last_icp_stamp_).seconds();
    const double since_last_aligned = (stamp - last_aligned_cloud_stamp_).seconds();
    const bool icp_due = last_icp_stamp_.nanoseconds() == 0 || since_last_icp < 0.0 ||
      since_last_icp >= relocalization_interval_s_;
    const bool aligned_cloud_due = aligned_cloud_interval_s_ <= 0.0 ||
      last_aligned_cloud_stamp_.nanoseconds() == 0 || since_last_aligned < 0.0 ||
      since_last_aligned >= aligned_cloud_interval_s_;
    if (!icp_due && !aligned_cloud_due) {
      return;
    }

    auto source_base = makeSourceCloud(*msg);
    if (static_cast<int>(source_base->size()) < min_scan_points_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Filtered scan has too few points for ICP: %zu.", source_base->size());
      publishStatus(
        msg->header.stamp, fast_anchor_interfaces::msg::LocalizationStatus::LOST,
        "filtered scan has too few points for ICP", false, last_icp_fitness_score_);
      return;
    }

    if (!icp_due) {
      // ICP intentionally runs at a lower rate than the LiDAR. For intermediate
      // scans, apply the latest map->odom correction to the current (not cached)
      // cloud so aligned_cloud follows the sensor input rate.
      pcl::PointCloud<PointType> aligned;
      pcl::transformPointCloud(
        *source_base, aligned, guess_map_to_base.matrix().cast<float>());
      publishTfAndPose(msg->header.stamp, guess_map_to_base, -1.0);
      publishAlignedCloud(aligned, msg->header.stamp);
      last_aligned_cloud_stamp_ = stamp;
      publishStatus(
        msg->header.stamp, fast_anchor_interfaces::msg::LocalizationStatus::TRACKING,
        "tracking with latest map->odom correction", last_icp_converged_, last_icp_fitness_score_);
      return;
    }
    last_icp_stamp_ = stamp;

    pcl::PointCloud<PointType> aligned;
    icp_.setInputSource(source_base);
    icp_.align(aligned, guess_map_to_base.matrix().cast<float>());

    const double fitness = icp_.getFitnessScore();
    if (!icp_.hasConverged() || fitness > fitness_score_threshold_) {
      last_icp_fitness_score_ = fitness;
      last_icp_converged_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "ICP rejected: converged=%s fitness=%.4f threshold=%.4f",
        icp_.hasConverged() ? "true" : "false",
        fitness, fitness_score_threshold_);
      publishTfAndPose(msg->header.stamp, guess_map_to_base, fitness);
      publishAlignedCloud(aligned, msg->header.stamp);
      last_aligned_cloud_stamp_ = stamp;
      publishIcpResult(msg->header.stamp, false, fitness, guess_map_to_base);
      publishStatus(
        msg->header.stamp, fast_anchor_interfaces::msg::LocalizationStatus::LOST,
        "ICP rejected by convergence or fitness threshold", false, fitness);
      return;
    }

    const Eigen::Affine3f refined_f(icp_.getFinalTransformation());
    const Eigen::Affine3d refined_map_to_base(refined_f.matrix().cast<double>());
    map_to_odom_ = refined_map_to_base * latest_odom_to_base_.inverse();
    last_icp_fitness_score_ = fitness;
    last_icp_converged_ = true;
    publishTfAndPose(msg->header.stamp, refined_map_to_base, fitness);
    publishAlignedCloud(aligned, msg->header.stamp);
    last_aligned_cloud_stamp_ = stamp;
    publishIcpResult(msg->header.stamp, true, fitness, refined_map_to_base);
    publishStatus(
      msg->header.stamp, fast_anchor_interfaces::msg::LocalizationStatus::TRACKING,
      "ICP accepted", true, fitness);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "ICP accepted: fitness=%.4f source_points=%zu", fitness, source_base->size());
  }

  pcl::PointCloud<PointType>::Ptr makeSourceCloud(const sensor_msgs::msg::PointCloud2 & msg) const
  {
    pcl::PointCloud<PointType>::Ptr cloud_body(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(msg, *cloud_body);

    pcl::PointCloud<PointType>::Ptr cloud_base(new pcl::PointCloud<PointType>());
    cloud_base->reserve(cloud_body->size());
    for (const auto & p_body : cloud_body->points) {
      if (!std::isfinite(p_body.x) || !std::isfinite(p_body.y) || !std::isfinite(p_body.z)) {
        continue;
      }
      const double range = std::sqrt(
        p_body.x * p_body.x + p_body.y * p_body.y + p_body.z * p_body.z);
      if (range < min_range_ || range > max_range_) {
        continue;
      }

      const Eigen::Vector3d base_xyz =
        body_to_base_ * Eigen::Vector3d(p_body.x, p_body.y, p_body.z);
      if (source_non_ground_filter_en_ && base_xyz.z() < source_non_ground_min_z_) {
        continue;
      }
      PointType p_base;
      p_base.x = static_cast<float>(base_xyz.x());
      p_base.y = static_cast<float>(base_xyz.y());
      p_base.z = static_cast<float>(base_xyz.z());
      p_base.intensity = p_body.intensity;
      cloud_base->push_back(p_base);
    }

    downsample(cloud_base, scan_leaf_size_);
    return cloud_base;
  }

  void publishTfAndPose(
    const builtin_interfaces::msg::Time & stamp,
    const Eigen::Affine3d & map_to_base,
    const double fitness)
  {
    if (publish_tf_) {
      const auto transform = affineToTransform(
        stamp, map_frame_, odom_frame_, odom_coincident_with_base_ ? map_to_base : map_to_odom_);
      tf_broadcaster_->sendTransform(transform);
    }

    if (publish_base_tf_) {
      const auto odom_to_base =
        odom_coincident_with_base_ ? Eigen::Affine3d::Identity() : latest_odom_to_base_;
      const auto transform = affineToTransform(stamp, odom_frame_, base_frame_, odom_to_base);
      tf_broadcaster_->sendTransform(transform);
    }

    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose.pose = affineToPose(map_to_base);
    const double covariance = fitness >= 0.0 ? std::max(0.001, fitness) : 0.25;
    pose_msg.pose.covariance[0] = covariance;
    pose_msg.pose.covariance[7] = covariance;
    pose_msg.pose.covariance[14] = covariance;
    pose_msg.pose.covariance[35] = covariance;
    pose_pub_->publish(pose_msg);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header = pose_msg.header;
    odom_msg.child_frame_id = base_frame_;
    odom_msg.pose = pose_msg.pose;
    odom_pub_->publish(odom_msg);

    geometry_msgs::msg::PoseStamped path_pose;
    path_pose.header = pose_msg.header;
    path_pose.pose = pose_msg.pose.pose;
    path_msg_.header.stamp = stamp;
    path_msg_.header.frame_id = map_frame_;
    path_msg_.poses.push_back(path_pose);
    if (max_path_size_ > 0 && static_cast<int>(path_msg_.poses.size()) > max_path_size_) {
      const auto erase_count = path_msg_.poses.size() - static_cast<size_t>(max_path_size_);
      path_msg_.poses.erase(path_msg_.poses.begin(), path_msg_.poses.begin() + erase_count);
    }
    const rclcpp::Time path_stamp(stamp);
    const double since_last_path = (path_stamp - last_path_publish_stamp_).seconds();
    const bool path_due = path_publish_interval_s_ <= 0.0 ||
      last_path_publish_stamp_.nanoseconds() == 0 || since_last_path < 0.0 ||
      since_last_path >= path_publish_interval_s_;
    if (path_due && path_pub_->get_subscription_count() > 0) {
      path_pub_->publish(path_msg_);
      last_path_publish_stamp_ = path_stamp;
    }
  }

  void publishIcpResult(
    const builtin_interfaces::msg::Time & stamp,
    const bool converged,
    const double fitness,
    const Eigen::Affine3d & corrected_map_to_base) const
  {
    fast_anchor_interfaces::msg::IcpResult result_msg;
    result_msg.header.stamp = stamp;
    result_msg.header.frame_id = map_frame_;
    result_msg.converged = converged;
    result_msg.fitness_score = fitness;

    result_msg.corrected_pose.header = result_msg.header;
    result_msg.corrected_pose.pose.pose = affineToPose(corrected_map_to_base);
    const double covariance = fitness >= 0.0 ? std::max(0.001, fitness) : 0.25;
    result_msg.corrected_pose.pose.covariance[0] = covariance;
    result_msg.corrected_pose.pose.covariance[7] = covariance;
    result_msg.corrected_pose.pose.covariance[14] = covariance;
    result_msg.corrected_pose.pose.covariance[35] = covariance;
    result_msg.map_to_odom = affineToTransform(
      stamp, map_frame_, odom_frame_,
      odom_coincident_with_base_ ? corrected_map_to_base : map_to_odom_);
    icp_result_pub_->publish(result_msg);
  }

  void publishStatus(
    const builtin_interfaces::msg::Time & stamp,
    const uint8_t state,
    const std::string & state_text,
    const bool icp_converged,
    const double fitness) const
  {
    fast_anchor_interfaces::msg::LocalizationStatus status_msg;
    status_msg.header.stamp = stamp;
    status_msg.header.frame_id = map_frame_;
    status_msg.state = state;
    status_msg.state_text = state_text;
    status_msg.initialized = have_initial_pose_;
    status_msg.tracking =
      have_odom_ && have_initial_pose_ && icp_map_cloud_ && !icp_map_cloud_->empty();
    status_msg.icp_converged = icp_converged;
    status_msg.icp_fitness_score = fitness;
    status_msg.translation_error = 0.0;
    status_msg.rotation_error_deg = 0.0;
    status_pub_->publish(status_msg);
  }

  void publishAlignedCloud(
    const pcl::PointCloud<PointType> & aligned,
    const builtin_interfaces::msg::Time & stamp) const
  {
    if (aligned_pub_->get_subscription_count() == 0) {
      return;
    }
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(aligned, cloud_msg);
    cloud_msg.header.stamp = stamp;
    cloud_msg.header.frame_id = map_frame_;
    aligned_pub_->publish(cloud_msg);
  }

  void publishMapCloudOnce()
  {
    const bool publish_visualization_map = !visualization_map_published_ &&
      visualization_map_pub_->get_subscription_count() > 0;
    const bool publish_icp_map = !icp_map_published_ &&
      icp_map_pub_->get_subscription_count() > 0;
    if (!publish_visualization_map && !publish_icp_map) {
      return;
    }
    if (publish_visualization_map && visualization_map_cloud_ &&
      !visualization_map_cloud_->empty())
    {
      sensor_msgs::msg::PointCloud2 cloud_msg;
      pcl::toROSMsg(*visualization_map_cloud_, cloud_msg);
      cloud_msg.header.stamp = now();
      cloud_msg.header.frame_id = map_frame_;
      visualization_map_pub_->publish(cloud_msg);
      visualization_map_published_ = true;
    }

    if (publish_icp_map && icp_map_cloud_ && !icp_map_cloud_->empty()) {
      sensor_msgs::msg::PointCloud2 cloud_msg;
      pcl::toROSMsg(*icp_map_cloud_, cloud_msg);
      cloud_msg.header.stamp = now();
      cloud_msg.header.frame_id = map_frame_;
      icp_map_pub_->publish(cloud_msg);
      icp_map_published_ = true;
    }
    if (visualization_map_published_ && icp_map_published_ && map_publish_timer_) {
      map_publish_timer_->cancel();
    }
  }

  std::mutex mutex_;
  std::string map_pcd_path_;
  std::string icp_map_pcd_path_;
  std::string visualization_map_pcd_path_;
  std::string odom_topic_;
  std::string scan_topic_;
  std::string initialpose_topic_;
  std::string reset_service_name_;
  std::string relocalize_service_name_;
  std::string reset_localization_service_name_;
  std::string fastlio_reset_service_;
  std::string source_odom_frame_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string lidar_frame_;
  std::string pose_topic_;
  std::string odom_output_topic_;
  std::string path_topic_;
  std::string status_topic_;
  std::string icp_result_topic_;
  std::string aligned_cloud_topic_;
  std::string icp_map_topic_;
  std::string visualization_map_topic_;
  bool publish_tf_ = true;
  bool publish_base_tf_ = true;
  bool odom_coincident_with_base_ = false;
  bool publish_map_cloud_ = true;
  bool visualization_map_published_ = false;
  bool icp_map_published_ = false;
  bool use_initial_pose_param_ = false;
  int max_path_size_ = 10000;

  double min_range_ = 0.5;
  double max_range_ = 60.0;
  double scan_leaf_size_ = 0.25;
  double map_leaf_size_ = 0.25;
  int min_scan_points_ = 120;
  double source_non_ground_min_z_ = -0.2;
  bool source_non_ground_filter_en_ = true;
  double max_correspondence_distance_ = 1.5;
  double transformation_epsilon_ = 0.01;
  double euclidean_fitness_epsilon_ = 0.01;
  int max_iterations_ = 40;
  double fitness_score_threshold_ = 1.0;
  double relocalization_interval_s_ = 0.2;
  double aligned_cloud_interval_s_ = 0.0;
  double path_publish_interval_s_ = 0.0;
  double fastlio_reset_timeout_s_ = 1.0;

  Eigen::Affine3d base_to_body_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d body_to_base_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d initial_map_to_base_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d pending_initial_map_to_base_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d latest_odom_to_body_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d latest_odom_to_base_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d map_to_odom_ = Eigen::Affine3d::Identity();
  builtin_interfaces::msg::Time latest_odom_stamp_;
  rclcpp::Time last_icp_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_aligned_cloud_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_path_publish_stamp_{0, 0, RCL_ROS_TIME};
  double last_icp_fitness_score_ = -1.0;
  bool last_icp_converged_ = false;
  bool have_odom_ = false;
  bool have_scan_ = false;
  bool have_initial_pose_ = false;
  bool have_pending_initial_pose_ = false;
  nav_msgs::msg::Path path_msg_;

  pcl::PointCloud<PointType>::Ptr icp_map_cloud_;
  pcl::PointCloud<PointType>::Ptr visualization_map_cloud_;
  pcl::IterativeClosestPoint<PointType, PointType> icp_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::Service<fast_anchor_interfaces::srv::Relocalize>::SharedPtr relocalize_srv_;
  rclcpp::Service<fast_anchor_interfaces::srv::ResetLocalization>::SharedPtr reset_localization_srv_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr fastlio_reset_client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<fast_anchor_interfaces::msg::LocalizationStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<fast_anchor_interfaces::msg::IcpResult>::SharedPtr icp_result_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr icp_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr visualization_map_pub_;
  rclcpp::TimerBase::SharedPtr map_publish_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastAnchorLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
