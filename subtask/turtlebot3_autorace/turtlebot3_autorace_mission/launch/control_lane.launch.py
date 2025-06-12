#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
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
# Author: Hyungyu Kim

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    param_files = [
        os.path.join(
            get_package_share_directory('turtlebot3_autorace_mission'),
            'param',
            'drive_modes.yaml'
        ),
        os.path.join(
            get_package_share_directory('turtlebot3_autorace_mission'),
            'param',
            'ui_texts.yaml'
        )
    ]

    control_node = Node(
        package='turtlebot3_autorace_mission',
        executable='control_lane2',
        name='control_lane',
        output='screen',
        parameters=param_files,
        remappings=[
            ('/control/lane', '/detect/lane'),
            ('/control/cmd_vel', '/cmd_vel'),
        ],
        prefix=['xterm -e'],  # 필요 시 주석 해제
        emulate_tty=True       # ROS2 Node에서 기본 지원 안 함, 제거
    )
        
    return LaunchDescription([
        control_node
    ])
