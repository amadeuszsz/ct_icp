# Copyright 2024 Amadeusz Szymko
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg_prefix = FindPackageShare('ct_icp_odometry')
    config_path = LaunchConfiguration('config_path').perform(context)
    if not config_path:
        config_path = PathJoinSubstitution(
            [pkg_prefix, 'config', 'default.yaml']
        ).perform(context)

    rviz_cfg = PathJoinSubstitution([pkg_prefix, 'rviz', 'ct_icp.rviz']).perform(context)

    ct_icp_odometry_node = Node(
        package='ct_icp_odometry',
        executable='ct_icp_odometry_node_exe',
        name='ct_icp_odometry_node',
        parameters=[
            {
                'config_path': config_path,
                'main_frame': 'map',
                'child_frame': 'base_link',
                'init_pose': LaunchConfiguration('init_pose'),
                'debug_print': LaunchConfiguration('debug_print'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ],
        remappings=[
            ('~/input/cloud', LaunchConfiguration('input_cloud_topic')),
            ('~/output/odom', LaunchConfiguration('output_odom_topic')),
            ('~/output/world_points', LaunchConfiguration('output_world_points_topic')),
            ('~/output/key_points', LaunchConfiguration('output_key_points_topic'))
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info', '--enable-stdout-logs']
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(LaunchConfiguration('with_rviz'))
    )

    return [
        ct_icp_odometry_node,
        rviz
    ]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    add_launch_arg('config_path', '')
    add_launch_arg('init_pose', 'false')
    add_launch_arg('debug_print', 'false')
    add_launch_arg('use_sim_time', 'true')
    add_launch_arg('with_rviz', 'true')
    add_launch_arg('input_cloud_topic', '/cloud')
    add_launch_arg('output_odom_topic', '/odom')
    add_launch_arg('output_world_points_topic', '/world_points')
    add_launch_arg('output_key_points_topic', '/key_points')

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
