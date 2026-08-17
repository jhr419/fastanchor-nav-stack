// yifan_lio_adapter: convert yifanLIO output into the FAST-LIO-compatible
// FastAnchor LIO interface.
//
//   yifanLIO:
//     /LIO/odom_imu    -> nav_msgs/Odometry   (world -> IMU)
//     /LIO/clouds_lidar -> PointCloud2        (undistorted scan in lidar frame)
//
//   adapter output (FAST-LIO compatible, consumed unchanged by FastAnchor ICP):
//     /Odometry           -> nav_msgs/Odometry   (camera_init -> body, body == IMU)
//     /cloud_registered_body -> PointCloud2      (undistorted scan in body/IMU frame)
//     TF camera_init -> body (mirrors FAST-LIO)
//
// The adapter does NOT re-integrate IMU or re-estimate poses; it only performs
// topic renaming, frame normalization and the lidar->IMU extrinsic transform.
// The extrinsic is read from the same yifanLIO yaml file the LIO node loads, so
// there is a single source of truth and no duplicated extrinsic parameters.

#include <string>

#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>

namespace
{

using PointType = pcl::PointXYZI;

struct LioConfig
{
  std::string odom_imu_topic = "/LIO/odom_imu";
  std::string clouds_lidar_topic = "/LIO/clouds_lidar";
  Eigen::Matrix3d imu_R_lidar = Eigen::Matrix3d::Identity();
  Eigen::Vector3d imu_t_lidar = Eigen::Vector3d::Zero();
};

bool readString(const YAML::Node & root, const std::string & key, std::string & out)
{
  const YAML::Node node = root[key];
  if (!node || !node.IsScalar()) {
    return false;
  }
  out = node.as<std::string>();
  return true;
}

bool readVector3(const YAML::Node & root, const std::string & key, Eigen::Vector3d & out)
{
  const YAML::Node node = root[key];
  if (!node || !node.IsSequence() || node.size() != 3) {
    return false;
  }
  out.x() = node[0].as<double>();
  out.y() = node[1].as<double>();
  out.z() = node[2].as<double>();
  return true;
}

bool readMatrix3(const YAML::Node & root, const std::string & key, Eigen::Matrix3d & out)
{
  const YAML::Node node = root[key];
  if (!node || !node.IsSequence() || node.size() != 3) {
    return false;
  }
  for (int i = 0; i < 3; ++i) {
    if (!node[i].IsSequence() || node[i].size() != 3) {
      return false;
    }
    for (int j = 0; j < 3; ++j) {
      out(i, j) = node[i][j].as<double>();
    }
  }
  return true;
}

}  // namespace

class YifanLioAdapterNode : public rclcpp::Node
{
public:
  YifanLioAdapterNode()
  : Node("yifan_lio_adapter_node")
  {
    const std::string lio_yaml_dir = declare_parameter<std::string>(
      "lio_yaml_dir", "");
    const std::string lio_root_config = declare_parameter<std::string>(
      "lio_root_config", "root_localization.yaml");
    output_odom_topic_ = declare_parameter<std::string>(
      "output_odom_topic", "/Odometry");
    output_cloud_topic_ = declare_parameter<std::string>(
      "output_cloud_topic", "/cloud_registered_body");
    world_frame_ = declare_parameter<std::string>("world_frame", "camera_init");
    body_frame_ = declare_parameter<std::string>("body_frame", "body");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    std::string yaml_dir = lio_yaml_dir;
    if (yaml_dir.empty()) {
      try {
        yaml_dir = ament_index_cpp::get_package_share_directory("lio") + "/yaml";
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          get_logger(),
          "Cannot locate lio/yaml directory (package 'lio' not found): %s. "
          "Set lio_yaml_dir explicitly.",
          e.what());
        throw;
      }
    }

    const std::string root_path = yaml_dir + "/" + lio_root_config;
    YAML::Node root_node;
    try {
      root_node = YAML::LoadFile(root_path);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to load root config '%s': %s", root_path.c_str(), e.what());
      throw;
    }

    std::string lio_yaml;
    if (!readString(root_node, "lio_yaml", lio_yaml)) {
      RCLCPP_ERROR(get_logger(), "Root config '%s' has no 'lio_yaml' key.", root_path.c_str());
      throw;
    }

    const std::string lio_path = yaml_dir + "/" + lio_yaml;
    YAML::Node lio_node_yaml;
    try {
      lio_node_yaml = YAML::LoadFile(lio_path);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to load lio config '%s': %s", lio_path.c_str(), e.what());
      throw;
    }

    LioConfig cfg;
    readString(lio_node_yaml["topic_pub"], "odom_imu_topic_name", cfg.odom_imu_topic);
    readString(lio_node_yaml["topic_pub"], "clouds_lidar_topic_name", cfg.clouds_lidar_topic);

    if (!readVector3(lio_node_yaml["offset"], "imu_t_lidar", cfg.imu_t_lidar) ||
      !readMatrix3(lio_node_yaml["offset"], "imu_R_lidar", cfg.imu_R_lidar))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to read offset.imu_t_lidar / offset.imu_R_lidar from '%s'.",
        lio_path.c_str());
      throw;
    }
    imu_R_lidar_ = cfg.imu_R_lidar;
    imu_t_lidar_ = cfg.imu_t_lidar;

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      cfg.odom_imu_topic, rclcpp::QoS(10),
      std::bind(&YifanLioAdapterNode::odomCallback, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cfg.clouds_lidar_topic, rclcpp::SensorDataQoS(),
      std::bind(&YifanLioAdapterNode::cloudCallback, this, std::placeholders::_1));

    // FAST-LIO publishes /Odometry and /cloud_registered_body with the default
    // (reliable, depth 20) QoS. Mirror that so the FastAnchor ICP subscriber
    // sees exactly the same contract in both backends.
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 20);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, 20);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(
      get_logger(),
      "[FastAnchor] LIO backend: yifanlio (adapter active)");
    RCLCPP_INFO(
      get_logger(),
      "yifanLIO config: %s  odom_in=%s cloud_in=%s  odom_out=%s cloud_out=%s",
      lio_path.c_str(), cfg.odom_imu_topic.c_str(), cfg.clouds_lidar_topic.c_str(),
      output_odom_topic_.c_str(), output_cloud_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "lidar->IMU extrinsic  t=[%.4f %.4f %.4f]  R(identity=%s)",
      imu_t_lidar_.x(), imu_t_lidar_.y(), imu_t_lidar_.z(),
      imu_R_lidar_.isApprox(Eigen::Matrix3d::Identity(), 1e-9) ? "true" : "false");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    nav_msgs::msg::Odometry out = *msg;
    out.header.frame_id = world_frame_;
    out.child_frame_id = body_frame_;
    odom_pub_->publish(out);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped trans;
      trans.header.stamp = msg->header.stamp;
      trans.header.frame_id = world_frame_;
      trans.child_frame_id = body_frame_;
      trans.transform.translation.x = msg->pose.pose.position.x;
      trans.transform.translation.y = msg->pose.pose.position.y;
      trans.transform.translation.z = msg->pose.pose.position.z;
      trans.transform.rotation = msg->pose.pose.orientation;
      tf_broadcaster_->sendTransform(trans);
    }
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<PointType>::Ptr cloud_lidar(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*msg, *cloud_lidar);

    pcl::PointCloud<PointType>::Ptr cloud_imu(new pcl::PointCloud<PointType>());
    cloud_imu->reserve(cloud_lidar->size());
    cloud_imu->is_dense = cloud_lidar->is_dense;
    for (const auto & p : cloud_lidar->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      const Eigen::Vector3d p_lidar(p.x, p.y, p.z);
      // p_imu = imu_R_lidar * p_lidar + imu_t_lidar (yifanLIO convention).
      const Eigen::Vector3d p_imu = imu_R_lidar_ * p_lidar + imu_t_lidar_;
      PointType q;
      q.x = static_cast<float>(p_imu.x());
      q.y = static_cast<float>(p_imu.y());
      q.z = static_cast<float>(p_imu.z());
      q.intensity = p.intensity;
      cloud_imu->push_back(q);
    }

    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(*cloud_imu, out);
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = body_frame_;
    cloud_pub_->publish(out);
  }

  std::string output_odom_topic_;
  std::string output_cloud_topic_;
  std::string world_frame_;
  std::string body_frame_;
  bool publish_tf_ = true;
  Eigen::Matrix3d imu_R_lidar_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d imu_t_lidar_ = Eigen::Vector3d::Zero();

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<YifanLioAdapterNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("yifan_lio_adapter_node"), "Adapter failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
