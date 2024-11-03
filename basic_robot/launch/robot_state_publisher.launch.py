#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Darby Lim

import os
import xacro  # Import the xacro module

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Path to URDF/Xacro
    xacro_path = os.path.join(
        get_package_share_directory('basic_robot'),
        'urdf',
        'mycobot_280_classic_gazebo.urdf.xacro')  # Ensure this is the correct path to your .xacro file

    # Read and process the Xacro file
    robot_description_config = xacro.process_file(xacro_path)
    robot_desc = robot_description_config.toxml()

    # Launch configuration for simulation time
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        # Declare use_sim_time argument for toggling simulation time
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # Node for robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }],
        ),
    ])

