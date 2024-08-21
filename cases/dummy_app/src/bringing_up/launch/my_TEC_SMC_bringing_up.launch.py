#!/usr/bin/env python3
# Copyright 2021 OROCA
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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():      # launch file basic method
    param_dir = LaunchConfiguration(        # Init execution configuration (Set parameter file location)
        'param_dir',
        default=os.path.join(               # Parameter file location
            get_package_share_directory('bringing_up'),
            'param',
            'my_TEC_SMC_bringing_up_config.yaml'))

    return LaunchDescription([              # Return description
        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description='Full path of parameter file'),

        Node(
            package='sensor',
            executable='my_sensor',
            name='my_sensor',
            parameters=[param_dir],
            output='screen'),
        Node(
            package='localization',
            executable='my_localization',
            name='my_localization',
            parameters=[param_dir],
            output='screen'),
        # Node(
        #     package='perception',
        #     executable='my_camera_perception',
        #     name='my_camera_perception',
        #     # parameters=[param_dir],
        #     output='screen'),
        Node(
            package='perception',
            executable='my_perception',
            name='my_perception',
            parameters=[param_dir],
            output='screen'),
        Node(
            package='planning',
            executable='my_planning',
            name='my_planning',
            parameters=[param_dir],
            output='screen'),
        Node(
            package='control',
            executable='my_control',
            name='my_control',
            parameters=[param_dir],
            output='screen'),
        Node(
            package='vehicle_interface',
            executable='my_vehicle_interface',
            name='my_vehicle_interface',
            parameters=[param_dir],
            output='screen'),
    ])