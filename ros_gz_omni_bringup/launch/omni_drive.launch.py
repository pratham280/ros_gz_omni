# Copyright 2022 Open Source Robotics Foundation, Inc.
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('ros_gz_omni_bringup')
    pkg_project_gazebo = get_package_share_directory('ros_gz_omni_gazebo')
    pkg_project_description = get_package_share_directory('ros_gz_omni_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    # Setup to launch the simulator and Gazebo world
    gz_args = LaunchConfiguration('gz_args')
    headless = LaunchConfiguration('headless')

    # Load the SDF file from "description" package
    sdf_file  =  os.path.join(pkg_project_description, 'models', 'omni_drive', 'model.sdf')
    # sdf_file  =  os.path.join(pkg_project_gazebo, 'worlds', 'mecanum_drive.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()


    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': {PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'omni_drive.sdf',
        ]), ' -r ', gz_args, " ", "-s -v 4 "},'on_exit_shutdown': 'true'}.items(),
        condition=IfCondition(headless)
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': {PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'omni_drive.sdf',
        ]), ' -r ', gz_args, " "},'on_exit_shutdown': 'true'}.items(),
        condition=UnlessCondition(headless)
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'omni_drive.rviz'), "-t", "ros_gz_omni"],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_omni_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        condition=IfCondition(LaunchConfiguration('teleop')),
        output='screen'
    )

    teleop_twist_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[
            os.path.join(
                pkg_project_bringup,
                'config',
                'teleop_twist_joy.yaml'
            ),
            {
                'use_sim_time': True,
                'cmd_vel': '/omni_drive/cmd_vel'
            }
        ],
        remappings=[
            ('cmd_vel', '/omni_drive/cmd_vel')
        ],
        condition=IfCondition(LaunchConfiguration('teleop')),
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('gz_args', default_value=' ', description='Pass Gazebo arguments'),
        DeclareLaunchArgument('headless', default_value='true', description='Run gz sim headless'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
        DeclareLaunchArgument('teleop', default_value='true', description='Enable joystick teleoperation'),
        gz_sim_headless,
        gz_sim,
        bridge,
        teleop_twist_joy,
        joy_node,
        robot_state_publisher,
        rviz
    ])
