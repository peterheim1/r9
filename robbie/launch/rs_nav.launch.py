# Copyright (c) 2019 Intel Corporation
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

# /* Author: Gary Liu */

import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # config the serial number and base frame id of each camer
    rgbd_base_frame_id = LaunchConfiguration('base_frame_id', default='camera_link')
    rgbd_serial_no = LaunchConfiguration('serial_no', default='831612070384')

   
    scan_node = Node(
            package='depthimage_to_laserscan',
            node_executable='depthimage_to_laserscan_node',
            node_name='scan',
            output='screen',
            parameters=[{'output_frame':'/scan'}],
            remappings=[('depth','/camera/depth/image_rect_raw'),
                        ('depth_camera_info', '/camera/depth/camera_info')],
            )

    rgbd_node = Node(
        package='realsense_node',
        node_executable='realsense_node',
        node_namespace="",
        output='screen',
        parameters=[{'serial_no':rgbd_serial_no, 
                'base_frame_id': rgbd_base_frame_id,
                }]
        )
    return launch.LaunchDescription([scan_node, rgbd_node])
