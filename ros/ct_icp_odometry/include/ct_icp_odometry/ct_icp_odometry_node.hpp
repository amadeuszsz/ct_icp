// Copyright 2024 Amadeusz Szymko
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CT_ICP_ODOMETRY__CT_ICP_ODOMETRY_NODE_HPP_
#define CT_ICP_ODOMETRY__CT_ICP_ODOMETRY_NODE_HPP_

#include <memory>

#include <ct_icp/odometry.h>
#include <ct_icp/config.h>
#include <rclcpp/rclcpp.hpp>
#include <SlamCore/timer.h>
#include <SlamCore/config_utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "ct_icp_odometry/utils.hpp"
#include "ct_icp_odometry/visibility_control.hpp"


namespace ct_icp_odometry
{
struct Config
{
  ct_icp::OdometryOptions odometry_options = ct_icp::OdometryOptions::DefaultDrivingProfile();
  bool output_state_on_failure = true;
  std::string failure_output_dir = "/tmp";
  ct_icp::TIME_UNIT unit = ct_icp::SECONDS;
  bool check_timestamp_consistency = true;
  double expected_frame_time_sec = 0.1;
} config_;

typedef pcl::PointCloud<slam::XYZTPoint> CloudMessageT;

class CT_ICP_ODOMETRY_PUBLIC CtIcpOdometryNode : public rclcpp::Node
{
public:
  explicit CtIcpOdometryNode(const rclcpp::NodeOptions & options);
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void timer_callback();
  void set_twist_from_displacement(nav_msgs::msg::Odometry & current_odom_msg);

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr world_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr key_points_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr timer_ = nullptr;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = nullptr;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = nullptr;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ = nullptr;
  std::unique_ptr<ct_icp::Odometry> odometry_ptr_ = nullptr;
  std::atomic<double> previous_timestamp_;
  std::atomic<bool> is_initialized_ = false;
  slam::frame_id_t frame_id_ = 0;
  std::mutex registration_mutex_;
  slam::Timer avg_timer;
  nav_msgs::msg::Odometry previous_odom_msg_;
  std::string main_frame_;
  std::string child_frame_;
  double dt_odom_ = 0.0;
  bool init_pose_ = true;
  bool debug_print_ = false;
  bool is_tf_ = false;
  bool is_already_failed_ = false;
};
}  // namespace ct_icp_odometry

#endif  // CT_ICP_ODOMETRY__CT_ICP_ODOMETRY_NODE_HPP_
