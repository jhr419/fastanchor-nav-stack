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
// timestamp pairing, frame normalization and rigid transforms.
// The extrinsic is read from the same yifanLIO yaml file the LIO node loads, so
// there is a single source of truth and no duplicated extrinsic parameters.

#include <cstdint>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

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

Eigen::Matrix3d rpyToRotation(const std::vector<double> & rpy)
{
  const double roll = rpy.at(0);
  const double pitch = rpy.at(1);
  const double yaw = rpy.at(2);
  return (
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

int64_t stampNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL +
         static_cast<int64_t>(stamp.nanosec);
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
    const auto imu_to_common_body_rpy = declare_parameter<std::vector<double>>(
      "imu_to_common_body_rpy", {0.0, 0.0, 0.0});
    max_sync_queue_size_ = declare_parameter<int>("max_sync_queue_size", 2000);
    if (imu_to_common_body_rpy.size() != 3) {
      throw std::runtime_error("imu_to_common_body_rpy must contain [roll, pitch, yaw]");
    }
    if (max_sync_queue_size_ < 2) {
      throw std::runtime_error("max_sync_queue_size must be at least 2");
    }
    common_body_R_imu_ = rpyToRotation(imu_to_common_body_rpy);

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
    RCLCPP_INFO(
      get_logger(),
      "IMU->common-body coordinate rotation rpy=[%.6f %.6f %.6f] rad; "
      "high-rate odometry is preserved and clouds require an exact-stamp posterior pose",
      imu_to_common_body_rpy[0], imu_to_common_body_rpy[1], imu_to_common_body_rpy[2]);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    nav_msgs::msg::Odometry out;
    try {
      out = transformOdometry(*msg);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "Drop invalid yifanLIO odometry: %s", e.what());
      return;
    }
    odom_pub_->publish(out);
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped trans;
      trans.header = out.header;
      trans.child_frame_id = body_frame_;
      trans.transform.translation.x = out.pose.pose.position.x;
      trans.transform.translation.y = out.pose.pose.position.y;
      trans.transform.translation.z = out.pose.pose.position.z;
      trans.transform.rotation = out.pose.pose.orientation;
      tf_broadcaster_->sendTransform(trans);
    }

    const int64_t stamp_ns = stampNanoseconds(msg->header.stamp);
    odom_queue_[stamp_ns] = msg;
    publishPairIfReady(stamp_ns);
    trimQueue(odom_queue_, "odometry");
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const int64_t stamp_ns = stampNanoseconds(msg->header.stamp);
    cloud_queue_[stamp_ns] = msg;
    publishPairIfReady(stamp_ns);
    trimQueue(cloud_queue_, "cloud");
  }

  void publishPairIfReady(const int64_t stamp_ns)
  {
    const auto odom_it = odom_queue_.find(stamp_ns);
    const auto cloud_it = cloud_queue_.find(stamp_ns);
    if (odom_it == odom_queue_.end() || cloud_it == cloud_queue_.end()) {
      return;
    }

    const auto odom_msg = odom_it->second;
    const auto cloud_msg = cloud_it->second;
    odom_queue_.erase(odom_it);
    cloud_queue_.erase(cloud_it);

    const nav_msgs::msg::Odometry odom_out = transformOdometry(*odom_msg);

    pcl::PointCloud<PointType>::Ptr cloud_lidar(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*cloud_msg, *cloud_lidar);

    pcl::PointCloud<PointType>::Ptr cloud_body(new pcl::PointCloud<PointType>());
    cloud_body->reserve(cloud_lidar->size());
    cloud_body->is_dense = cloud_lidar->is_dense;
    for (const auto & p : cloud_lidar->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      const Eigen::Vector3d p_lidar(p.x, p.y, p.z);
      // p_imu = imu_R_lidar * p_lidar + imu_t_lidar (yifanLIO convention).
      const Eigen::Vector3d p_imu = imu_R_lidar_ * p_lidar + imu_t_lidar_;
      const Eigen::Vector3d p_body = common_body_R_imu_ * p_imu;
      PointType q;
      q.x = static_cast<float>(p_body.x());
      q.y = static_cast<float>(p_body.y());
      q.z = static_cast<float>(p_body.z());
      q.intensity = p.intensity;
      cloud_body->push_back(q);
    }

    sensor_msgs::msg::PointCloud2 cloud_out;
    pcl::toROSMsg(*cloud_body, cloud_out);
    cloud_out.header.stamp = cloud_msg->header.stamp;
    cloud_out.header.frame_id = body_frame_;

    // The exact-stamp posterior odometry has already been forwarded by
    // odomCallback. Publish the paired cloud only after that pose has arrived;
    // FastAnchor selects the same-stamp pose from its common odometry buffer.
    cloud_pub_->publish(cloud_out);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "[LIO INPUT] backend=yifanlio paired_stamp=%d.%09u points=%zu "
      "odom_xyz=[%.3f %.3f %.3f] frame=%s",
      cloud_out.header.stamp.sec, cloud_out.header.stamp.nanosec, cloud_body->size(),
      odom_out.pose.pose.position.x, odom_out.pose.pose.position.y,
      odom_out.pose.pose.position.z, cloud_out.header.frame_id.c_str());
  }

  nav_msgs::msg::Odometry transformOdometry(const nav_msgs::msg::Odometry & msg) const
  {
    // yifanLIO estimates T_world_imu. FastLIO's configured input pre-rotation
    // defines the common virtual body coordinates by p_body = R_body_imu p_imu.
    // Therefore T_world_body = T_world_imu * T_imu_body, where
    // T_imu_body = inverse(T_body_imu). This is a real pose conversion, not a
    // frame_id rename.
    const auto & pose = msg.pose.pose;
    Eigen::Quaterniond world_q_imu(
      pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    if (!std::isfinite(world_q_imu.norm()) || world_q_imu.norm() < 1e-9) {
      throw std::runtime_error("yifanLIO odometry contains an invalid quaternion");
    }
    world_q_imu.normalize();
    Eigen::Affine3d world_T_imu = Eigen::Affine3d::Identity();
    world_T_imu.linear() = world_q_imu.toRotationMatrix();
    world_T_imu.translation() = Eigen::Vector3d(
      pose.position.x, pose.position.y, pose.position.z);
    Eigen::Affine3d body_T_imu = Eigen::Affine3d::Identity();
    body_T_imu.linear() = common_body_R_imu_;
    const Eigen::Affine3d world_T_body = world_T_imu * body_T_imu.inverse();

    nav_msgs::msg::Odometry odom_out = msg;
    odom_out.header.frame_id = world_frame_;
    odom_out.child_frame_id = body_frame_;
    odom_out.pose.pose.position.x = world_T_body.translation().x();
    odom_out.pose.pose.position.y = world_T_body.translation().y();
    odom_out.pose.pose.position.z = world_T_body.translation().z();
    const Eigen::Quaterniond world_q_body(world_T_body.rotation());
    odom_out.pose.pose.orientation.x = world_q_body.x();
    odom_out.pose.pose.orientation.y = world_q_body.y();
    odom_out.pose.pose.orientation.z = world_q_body.z();
    odom_out.pose.pose.orientation.w = world_q_body.w();
    return odom_out;
  }

  template<typename MessageT>
  void trimQueue(std::map<int64_t, std::shared_ptr<MessageT>> & queue, const char * queue_name)
  {
    while (static_cast<int>(queue.size()) > max_sync_queue_size_) {
      const auto dropped_stamp = queue.begin()->first;
      queue.erase(queue.begin());
      if (std::string(queue_name) == "cloud") {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Dropping unmatched yifanLIO cloud at %.9f; exact posterior odometry was not received.",
          static_cast<double>(dropped_stamp) * 1e-9);
      }
    }
  }

  std::string output_odom_topic_;
  std::string output_cloud_topic_;
  std::string world_frame_;
  std::string body_frame_;
  bool publish_tf_ = true;
  int max_sync_queue_size_ = 2000;
  Eigen::Matrix3d imu_R_lidar_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d imu_t_lidar_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d common_body_R_imu_ = Eigen::Matrix3d::Identity();
  std::map<int64_t, nav_msgs::msg::Odometry::SharedPtr> odom_queue_;
  std::map<int64_t, sensor_msgs::msg::PointCloud2::SharedPtr> cloud_queue_;

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
