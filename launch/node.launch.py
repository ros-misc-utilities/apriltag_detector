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
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Create simple node."""
    node = Node(
        package='apriltag_detector',
        executable='apriltag_detector_node',
        output='screen',
        namespace=LaunchConfig('camera'),
        # prefix=['xterm -e gdb -ex run --args'],
        name='apriltag_detector',
        parameters=[{'tag_family': LaunchConfig('tag_family')}],
        remappings=[('image', 'image_raw')],
    )
    return [node]


def generate_launch_description():
    """Create simple node by calling opaque function."""
    return launch.LaunchDescription(
        [
            LaunchArg('camera', default_value=['camera'], description='camera name'),
            LaunchArg('tag_family', default_value='tf36h11', description='tag family'),
            OpaqueFunction(function=launch_setup),
        ]
    )
