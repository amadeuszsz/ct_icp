 ct_icp_odometry
<!-- Required -->
<!-- Package description -->
ROS 2 interface for CT ICP odometry.

## Installation
<!-- Required -->
<!-- Things to consider:
    - How to build package? 
    - Are there any other 3rd party dependencies required? -->

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --packages-up-to ct_icp_odometry --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=On -DSUPERBUILD_INSTALL_DIR=<path-to-superbuild-install-dir>
```

## Usage
<!-- Required -->
<!-- Things to consider:
    - Launching package. 
    - Exposed API (example service/action call. -->

```bash
ros2 launch ct_icp_odometry ct_icp_odometry.launch.py input_cloud_topic:=/your/pointcloud/topic
```

You can change input cloud topic in Input/Cloud rviz panel to see the actual input.

## API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

### Input

| Name            | Type                          | Description       |
| --------------- | ----------------------------- | ----------------- |
| `~/input/cloud` | sensor_msgs::msg::PointCloud2 | Input pointcloud. |

### Output

| Name                    | Type                                  | Description                          |
| ----------------------- | ------------------------------------- | ------------------------------------ |
| `~/output/odom`         | nav_msgs::msg::Odometry               | Final lidar odometry.                |
| `~/output/world_points` | sensor_msgs::msg::PointCloud2         | Transformed cloud to odometry frame. |
| `~/output/key_points`   | sensor_msgs::msg::PointCloud2         | Key points for CT ICP.               |
| `/diagnostics`          | diagnostic_msgs::msg::DiagnosticArray | CT ICP logs.                         |

### Parameters

| Name           | Type | Description                                                        |
| -------------- | ---- | ------------------------------------------------------------------ |
| `config_path`  | str  | Path to CT ICP config.                                             |
| `main_frame`   | str  | Main frame for output odometry.                                    |
| `child_frame`  | str  | Child frame for output odometry.                                   |
| `init_pose`    | bool | Whether to use init pose from transform main_frame -> child_frame. |
| `debug_print`  | bool | Whether to print debug info.                                       |
| `use_sim_time` | bool | Whether to use sim time.                                           |

## References / External links
<!-- Optional -->
