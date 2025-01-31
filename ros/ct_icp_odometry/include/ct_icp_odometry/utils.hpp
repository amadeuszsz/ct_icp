#ifndef CT_ICP_ODOMETRY_UTILS_HPP
#define CT_ICP_ODOMETRY_UTILS_HPP

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/time_reference.hpp>

#include <rclcpp/publisher.hpp>
#include <rclcpp/time.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <yaml-cpp/yaml.h>

#include <SlamCore/types.h>
#include <SlamCore/config_utils.h>
#include "ct_icp_odometry/roscore/point_types.hpp"


namespace ct_icp
{

enum TIME_UNIT
{
  SECONDS,
  NANO_SECONDS,
  MILLI_SECONDS
};

inline TIME_UNIT TimeUnitFromNode(YAML::Node & node, const std::string & key)
{
  TIME_UNIT unit;
  slam::config::FindEnumOption(
    node, (int &) unit, key, {
      {"NANO_SECONDS", ct_icp::NANO_SECONDS},
      {"MILLI_SECONDS", ct_icp::MILLI_SECONDS},
      {"SECONDS", ct_icp::SECONDS}
    });
  return unit;
}

// Returns the multiplication factor to apply to values expressed in unit to convert them to seconds
inline double ToSecondsFactor(TIME_UNIT unit)
{
  switch (unit) {
    case SECONDS:
      return 1.;
    case MILLI_SECONDS:
      return 1.e-3;
    case NANO_SECONDS:
      return 1.e-9;
    default:
      return -1.;
  }
}

// The Frame Ids on which the published topics depend
const std::string main_frame_id = "map";
const std::string child_frame_id = "base_link";

typedef pcl::PointCloud<slam::XYZTPoint> CloudMessageT;

// Converts a ct_icp Point Cloud to a PCL CloudMessageT
inline std::shared_ptr<CloudMessageT> WPointsToROSPointCloud(
  const std::vector<slam::WPoint3D> & points,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  auto pc = std::make_shared<CloudMessageT>();
  pc->header.frame_id = frame_id;
  pc->header.stamp = pcl_conversions::toPCL(stamp);
  pc->reserve(points.size());
  slam::XYZTPoint new_pt;
  for (auto idx(0); idx < points.size(); idx++) {
    new_pt.getVector3fMap() = points[idx].world_point.cast<float>();
    new_pt.timestamp = points[idx].TimestampConst();
    pc->push_back(new_pt);
  }
  return pc;
}

// TODO (?)
// inline ros::Publisher
// RegisterPointCloudPublisher(ros::NodeHandle &handle, const std::string &topic, int queue_size = 2) {
//     return handle.advertise<CloudMessageT>(topic, queue_size, false);
// }

template<typename T>
inline void PublishPoints(
  rclcpp::Publisher<T> publisher, const std::vector<slam::WPoint3D> & points,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  auto pc = (WPointsToROSPointCloud(points, frame_id, stamp));
  sensor_msgs::msg::PointCloud2 pc_msg;
  pcl::toROSMsg(*pc, pc_msg);
  publisher.publish(pc_msg);
}

inline nav_msgs::msg::Odometry SlamPoseToROSOdometry(
  const slam::SE3 & pose, const rclcpp::Time & stamp,
  const std::string & _frame_id = ct_icp::main_frame_id,
  const std::string & _child_frame_id = ct_icp::child_frame_id)
{
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.pose.pose.orientation.x = pose.quat.x();
  odom_msg.pose.pose.orientation.y = pose.quat.y();
  odom_msg.pose.pose.orientation.z = pose.quat.z();
  odom_msg.pose.pose.orientation.w = pose.quat.w();
  odom_msg.pose.pose.position.x = pose.tr.x();
  odom_msg.pose.pose.position.y = pose.tr.y();
  odom_msg.pose.pose.position.z = pose.tr.z();
  odom_msg.header.frame_id = _frame_id;
  odom_msg.child_frame_id = _child_frame_id;
  odom_msg.header.stamp.sec = stamp.seconds();
  odom_msg.header.stamp.nanosec = stamp.nanoseconds();
  return odom_msg;
}

inline geometry_msgs::msg::TransformStamped TransformFromPose(
  const slam::SE3 & pose, const rclcpp::Time & stamp,
  const std::string & _frame_id = ct_icp::main_frame_id,
  const std::string & _child_frame_id = ct_icp::child_frame_id)
{
  geometry_msgs::msg::TransformStamped tf_gt;
  Eigen::Isometry3d iso(pose.Matrix());
  tf_gt = tf2::eigenToTransform(iso);
  tf_gt.header.stamp.sec = stamp.seconds();
  tf_gt.header.stamp.nanosec = stamp.nanoseconds();
  tf_gt.header.frame_id = _frame_id;
  tf_gt.child_frame_id = _child_frame_id;
  return tf_gt;
}

}  // namespace ct_icp

#endif  // CT_ICP_ODOMETRY_UTILS_HPP
