#ifndef CT_ICP_ODOMETRY__ROSCORE__NAV_MSGS_CONVERSION_HPP
#define CT_ICP_ODOMETRY__ROSCORE__NAV_MSGS_CONVERSION_HPP

#include <rclcpp/time.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <SlamCore/types.h>
#include <SlamCore/imu.h>

namespace slam
{

inline double ROSTimeToSeconds(const rclcpp::Time & time)
{
  return time.seconds();
}

inline rclcpp::Time SecondsToROSTime(double secs)
{
  rclcpp::Time time(int32_t(secs), uint32_t((secs - int32_t(secs)) * 1.e9));
  return time;
}

inline geometry_msgs::msg::Vector3 EigenToROSVec3(const Eigen::Vector3d & xyz)
{
  geometry_msgs::msg::Vector3 vec;
  vec.x = xyz.x();
  vec.y = xyz.y();
  vec.z = xyz.z();
  return vec;
}

inline geometry_msgs::msg::Quaternion EigenToROSQuat(const Eigen::Quaterniond & quat)
{
  geometry_msgs::msg::Quaternion vec;
  vec.x = quat.x();
  vec.y = quat.y();
  vec.z = quat.z();
  vec.w = quat.w();
  return vec;
}

inline Eigen::Vector3d ROSToEigenVec3(const geometry_msgs::msg::Vector3 & xyz)
{

  return {xyz.x, xyz.y, xyz.z};
}

inline Eigen::Quaterniond ROSToEigenQuat(const geometry_msgs::msg::Quaternion & quat)
{
  Eigen::Quaterniond _quat;
  _quat.x() = quat.x;
  _quat.y() = quat.y;
  _quat.z() = quat.z;
  _quat.w() = quat.w;
  return _quat;
}


// Converts an odometry message to a slam::Pose
slam::Pose ROSOdometryToPose(
  const nav_msgs::msg::Odometry & odometry,
  slam::frame_id_t frame_id = 0);

// Converts an odometry message to a slam::Pose
nav_msgs::msg::Odometry PoseToROSOdometry(
  const slam::Pose & pose,
  const std::string & src_frame_id = "map",
  const std::string & dest_frame_id = "base_link");

sensor_msgs::msg::Imu SlamToROSImu(const slam::ImuData & imu);

slam::ImuData ROSToSlamImu(const sensor_msgs::msg::Imu & imu);

}  // namespace slam

#endif  // CT_ICP_ODOMETRY__ROSCORE__NAV_MSGS_CONVERSION_HPP
