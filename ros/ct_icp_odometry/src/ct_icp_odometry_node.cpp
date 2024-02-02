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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ct_icp_odometry/ct_icp_odometry_node.hpp"
#include "ct_icp_odometry/utils.hpp"
#include "ct_icp_odometry/roscore/nav_msgs_conversion.hpp"
#include "ct_icp_odometry/roscore/pc2_conversion.hpp"


namespace ct_icp_odometry
{

CtIcpOdometryNode::CtIcpOdometryNode(const rclcpp::NodeOptions & options)
:  Node("ct_icp_odometry", options)
{
  rclcpp::QoS qos_pub(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  rclcpp::QoS qos_sub(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos_sub.best_effort();
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/cloud", qos_sub,
    std::bind(&CtIcpOdometryNode::cloud_callback, this, std::placeholders::_1));
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("~/output/odom", qos_pub);
  world_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/output/world_points", qos_pub);
  key_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/output/key_points", qos_pub);
  diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", qos_pub);

  auto config_path = this->declare_parameter("config_path", "");
  main_frame_ = this->declare_parameter("main_frame", "map");
  child_frame_ = this->declare_parameter("child_frame", "base_link");
  init_pose_ = this->declare_parameter("init_pose", true);
  debug_print_ = this->declare_parameter("debug_print", false);

  if (init_pose_) {
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&CtIcpOdometryNode::timer_callback, this));
  }

  ct_icp::OdometryOptions odometry_options;
  if (config_path.empty()) {
    RCLCPP_WARN_STREAM(
      get_logger(), "The path to the yaml config is empty, loading the default config.");
    odometry_options = ct_icp::OdometryOptions::DefaultDrivingProfile();
  } else {
    try {
      RCLCPP_INFO_STREAM(get_logger(), "Loading Config from yaml file: " << config_path);
      auto node = slam::config::RootNode(config_path);
      if (node["odometry_options"]) {
        auto odometry_node = node["odometry_options"];
        odometry_options = ct_icp::yaml_to_odometry_options(odometry_node);

        RCLCPP_INFO_STREAM(
          get_logger(),
          "debug print: " << odometry_options.debug_print << ", ct_icp print: "
                          << odometry_options.ct_icp_options.debug_print);
        config_.odometry_options = odometry_options;
      }

      FIND_OPTION(node, config_, failure_output_dir, std::string)
      FIND_OPTION(node, config_, output_state_on_failure, bool)
      FIND_OPTION(node, config_, check_timestamp_consistency, bool)
      config_.unit = ct_icp::TimeUnitFromNode(node, "unit");

    } catch (...) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Error while loading the config from path: `" << config_path << "`");
      rclcpp::shutdown();
      throw;
    }
  }
  odometry_ptr_ = std::make_unique<ct_icp::Odometry>(odometry_options);

}

void CtIcpOdometryNode::timer_callback()
{
  try {
    auto init_transform = tf_buffer_->lookupTransform(
      "map", "base_link", this->now(), rclcpp::Duration::from_seconds(0.5));
    is_tf_ = true;
    RCLCPP_INFO_STREAM(get_logger(), "Found TF");

    nav_msgs::msg::Odometry init_odom;
    init_odom.header = init_transform.header;
    init_odom.child_frame_id = init_transform.child_frame_id;
    init_odom.pose.pose.position.x = init_transform.transform.translation.x;
    init_odom.pose.pose.position.y = init_transform.transform.translation.y;
    init_odom.pose.pose.position.z = init_transform.transform.translation.z;
    init_odom.pose.pose.orientation = init_transform.transform.rotation;
    auto slam_pose = slam::ROSOdometryToPose(init_odom);
    odometry_ptr_->SetInitialPose(slam_pose);
    timer_->cancel();
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(this->get_logger(), ex.what());
  }
}

// TODO: apply filtering to the twist
void CtIcpOdometryNode::set_twist_from_displacement(nav_msgs::msg::Odometry & current_odom_msg)
{
  if (previous_odom_msg_.header.stamp.sec == 0) {
    previous_odom_msg_ = current_odom_msg;
    return;
  }
  auto dt = rclcpp::Time(current_odom_msg.header.stamp) - rclcpp::Time(
    previous_odom_msg_.header.stamp);
  dt_odom_ = dt.seconds();


  tf2::Transform tf_current, tf_previous, tf_rel;
  tf2::fromMsg(current_odom_msg.pose.pose, tf_current);
  tf2::fromMsg(previous_odom_msg_.pose.pose, tf_previous);
  tf_rel = tf_previous.inverseTimes(tf_current);

  current_odom_msg.twist.twist.linear.x = tf_rel.getOrigin().x() / dt_odom_;
  current_odom_msg.twist.twist.linear.y = tf_rel.getOrigin().y() / dt_odom_;
  current_odom_msg.twist.twist.linear.z = tf_rel.getOrigin().z() / dt_odom_;
  auto q = tf_rel.getRotation();
  tf2::Matrix3x3 m(q);
  double droll, dpitch, dyaw;
  m.getRPY(droll, dpitch, dyaw);
  current_odom_msg.twist.twist.angular.x = droll / dt_odom_;
  current_odom_msg.twist.twist.angular.y = dpitch / dt_odom_;
  current_odom_msg.twist.twist.angular.z = dyaw / dt_odom_;

  previous_odom_msg_ = current_odom_msg;
}

void CtIcpOdometryNode::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!is_tf_ && init_pose_) {
    return;
  }
  if (debug_print_) {
    RCLCPP_INFO_STREAM(get_logger(), "Received Point Cloud Message!");
  }
  // std::lock_guard<std::mutex> guard{registration_mutex_};
  auto & pointcloud2 = *const_cast<sensor_msgs::msg::PointCloud2::SharedPtr &>(msg);
  auto stamp = pointcloud2.header.stamp;
  auto stamp_sec = slam::ROSTimeToSeconds(stamp);
  auto pointcloud = slam::ROSCloud2ToSlamPointCloudShallow(pointcloud2);
  pointcloud->RegisterFieldsFromSchema();   // Registers default fields (timestamp, intensity, rgb, etc ...)

  // -- CHECK THAT THE TIMESTAMPS FIELD EXIST
  if (!pointcloud->HasTimestamps()) {
    if (config_.odometry_options.ct_icp_options.parametrization == ct_icp::CONTINUOUS_TIME) {
      RCLCPP_INFO_STREAM(
        get_logger(),
        "The point cloud does not have timestamps, this is incompatible "
        "with the `CONTINUOUS_TIME` representation of pose in CT-ICP");
      RCLCPP_INFO_STREAM(
        get_logger(),
        "The schema of the point cloud is:\n"
          << pointcloud->GetCollection().GetItemInfo(0).item_schema);
      rclcpp::shutdown();
      return;
    }

    // Add timestamps which will all be set to 1.0
    auto copy = pointcloud->DeepCopyPtr();
    {
      auto field = copy->AddElementField<double, slam::FLOAT64>("new_timestamps");
      copy->SetTimestampsField(std::move(field));
    }
    auto timestamps = copy->Timestamps<double>();
    for (auto & t: timestamps) {
      t = stamp_sec;
    }
    pointcloud = copy;

  } else {


    if (debug_print_) {
      RCLCPP_INFO_STREAM(
        get_logger(),
        "\n\n/* ---------------------------------------------------------------------- */\n"
          << "Processing Frame at timestamp " << stamp.sec << "(sec) " << stamp.nanosec
          << " (nsec). Containing " << pointcloud->size() << " points.");
    }

    auto timestamps = pointcloud->TimestampsProxy<double>();
    auto minmax = std::minmax_element(timestamps.begin(), timestamps.end());
    double min_t = *minmax.first;
    double max_t = *minmax.second;


    double dt = max_t - min_t;
    double expected_dt;
    switch (config_.unit) {
      case ct_icp::SECONDS:
        expected_dt = config_.expected_frame_time_sec;
        break;
      case ct_icp::MILLI_SECONDS:
        expected_dt = config_.expected_frame_time_sec * 1.e3;
        break;
      case ct_icp::NANO_SECONDS:
        expected_dt = config_.expected_frame_time_sec * 1.e9;
        break;
    }

    if (!is_initialized_) {
      is_initialized_ = true;
      if (debug_print_) {
        RCLCPP_INFO_STREAM(get_logger(), "Min_t=" << *minmax.first << ", Max_t=" << *minmax.second);
      }
    } else {
      bool invalid_timestamps = false;
      double r_dt = dt / expected_dt;

      if (debug_print_) {
        RCLCPP_INFO_STREAM(
          get_logger(),
          "Min_t=" << *minmax.first << ", Max_t=" << *minmax.second
                   << ", dt=" << dt << " r_dt=" << r_dt);
      }

      if (r_dt > 1.05 || r_dt < 0.95) {
        invalid_timestamps = true;
        if (debug_print_) {
          RCLCPP_INFO_STREAM(
            get_logger(),
            "Found Inconsistent Timestamp for the frame : "
            "difference does not match the expected frequency");
        }

        std::vector<size_t> ids;
        ids.reserve(pointcloud->size());
        auto timestamps = pointcloud->TimestampsProxy<double>();

        double prev_t = previous_timestamp_;
        double next_t = prev_t + expected_dt;

        for (auto idx(0); idx < pointcloud->size(); idx++) {
          double timestamp = timestamps[idx];
          if (prev_t <= timestamp && timestamp <= next_t) {
            ids.push_back(idx);
          }
        }
        {
          if (debug_print_) {
            RCLCPP_WARN_STREAM(get_logger(), "Skipping the frame");
          }
          return;
        }
      } else if (std::abs(previous_timestamp_ - min_t) > expected_dt) {
        // Potentially skipped a frame
        if (debug_print_) {
          RCLCPP_WARN_STREAM(
            get_logger(),
            "Found Inconsistent Timestamp for the frame : "
            "difference does not match the expected frequency");
          RCLCPP_WARN_STREAM(get_logger(), "Will continue the acquisition");
        }
      }
    }

    previous_timestamp_ = max_t;
  }

  // -- REGISTER NEW FRAME
  slam::Timer timer;
  ct_icp::Odometry::RegistrationSummary summary;

  std::shared_ptr<pcl::PointCloud<slam::XYZTPoint>>
  corrected_pc = nullptr, keypoints_pc = nullptr;
  {
    slam::Timer::Ticker avg_ticker(avg_timer, "registration");
    {
      slam::Timer::Ticker ticker(timer, "registration");
      summary = odometry_ptr_->RegisterFrame(*pointcloud, frame_id_++);
    }
  }
  if (debug_print_) {
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Registration took: " << timer.AverageDuration(
        "registration",
        slam::Timer::MILLISECONDS) << "(ms)");
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Average Registration time: " << avg_timer.AverageDuration(
        "registration",
        slam::Timer::MILLISECONDS)
                                    << "(ms)");
  }

  if (summary.success) {
    if (debug_print_) {
      RCLCPP_INFO(get_logger(), "Registration is a success.");
      is_already_failed_ = false;
    }
  } else {
    if (debug_print_) {
      RCLCPP_INFO(get_logger(), "Registration is a failure");
    }


    if (config_.output_state_on_failure && !is_already_failed_) {
      if (debug_print_) {
        RCLCPP_INFO(get_logger(), "Persisting last state:");
      }

      fs::path output_dir_path(config_.failure_output_dir);
      if (!exists(output_dir_path)) {
        create_directories(output_dir_path);
      }

      {
        auto initial_frame_path = output_dir_path / "initial_frame.ply";
        if (debug_print_) {
          RCLCPP_INFO_STREAM(get_logger(), "Saving Initial Frame to " << initial_frame_path);
        }
        std::vector<slam::Pose> initial_frames{summary.initial_frame.begin_pose,
          summary.initial_frame.end_pose};
        slam::SavePosesAsPLY(initial_frame_path, initial_frames);
      }

      {
        auto map_path = output_dir_path / "map.ply";
        if (debug_print_) {
          RCLCPP_INFO_STREAM(get_logger(), "Saving Map to " << map_path);
        }
        auto pc = odometry_ptr_->GetMapPointCloud();
        pc->RegisterFieldsFromSchema();
        auto mapper = slam::PLYSchemaMapper::BuildDefaultFromBufferCollection(pc->GetCollection());
        slam::WritePLY(map_path, *pc, mapper);
      }

      {
        auto frame_path = output_dir_path / "frame.ply";
        if (debug_print_) {
          RCLCPP_INFO_STREAM(get_logger(), "Saving frame to " << frame_path);
        }
        auto mapper = slam::PLYSchemaMapper::BuildDefaultFromBufferCollection(
          pointcloud->GetCollection());
        slam::WritePLY(frame_path, *pointcloud, mapper);
      }

    }
    is_already_failed_ = true;

    return;
  }

  auto odom_msg = ct_icp::SlamPoseToROSOdometry(
    summary.frame.end_pose.pose, stamp, main_frame_,
    child_frame_);
  set_twist_from_displacement(odom_msg);
  odom_pub_->publish(odom_msg);

  if (!summary.corrected_points.empty()) {
    ct_icp::PublishPoints(
      *world_points_pub_, summary.corrected_points, ct_icp::main_frame_id, stamp);
  }

  if (!summary.keypoints.empty()) {
    ct_icp::PublishPoints(*key_points_pub_, summary.keypoints, ct_icp::main_frame_id, stamp);
  }


  // -- PUBLISH Logging values
  diagnostic_msgs::msg::DiagnosticStatus diag_status_msg;
  diag_status_msg.name = "ct_icp_odometry";
  diag_status_msg.hardware_id = "";
  diag_status_msg.message = "";

  if (is_already_failed_) {
    diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  } else {
    diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  }

  auto log_value = [&diag_status_msg](const std::string & key, double value) {
      diagnostic_msgs::msg::KeyValue key_value_msg;
      key_value_msg.key = key;
      key_value_msg.value = std::to_string(value);
      diag_status_msg.values.push_back(key_value_msg);
    };

  for (auto & kvp : summary.logged_values) {
    log_value(kvp.first, kvp.second);
  }
  log_value("avg_duration_iter", summary.icp_summary.avg_duration_iter);
  log_value("duration_total", summary.icp_summary.duration_total);
  log_value("avg_duration_neighborhood", summary.icp_summary.avg_duration_neighborhood);
  log_value("dt_odom", dt_odom_);

  diagnostic_msgs::msg::DiagnosticArray diag_msg;
  diag_msg.header.stamp = stamp;
  diag_msg.header.frame_id = ct_icp::child_frame_id;
  diag_msg.status.push_back(diag_status_msg);
  diagnostics_pub_->publish(diag_msg);
}

}  // namespace ct_icp_odometry

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ct_icp_odometry::CtIcpOdometryNode)
