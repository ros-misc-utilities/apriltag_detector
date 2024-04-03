# -----------------------------------------------------------------------------
# Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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
#

import launch
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    """Create composable node."""
    container = ComposableNodeContainer(
        name='apriltag_detector_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='apriltag_detector',
                plugin='apriltag_detector_ros::ApriltagDetector',
                namespace=LaunchConfig('camera'),
                parameters=[{'tag_family': LaunchConfig('tag_family')}],
                remappings=[('image', 'image_raw')],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        ],
        output='screen',
    )
    return [container]


def generate_launch_description():
    """Create composable node by calling opaque function."""
    return launch.LaunchDescription(
        [
            LaunchArg('camera', default_value=['camera'], description='camera name'),
            LaunchArg('tag_family', default_value='tf36h11', description='tag family'),
            OpaqueFunction(function=launch_setup),
        ]
    )
