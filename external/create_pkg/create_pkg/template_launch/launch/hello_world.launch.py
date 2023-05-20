# Copyright YEAR MAINTAINER_NAME
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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg_prefix = FindPackageShare("hello_world")
    # config_param = PathJoinSubstitution([pkg_prefix, LaunchConfiguration('config_param_file')])
    config_rviz = PathJoinSubstitution([pkg_prefix, 'rviz/default.rviz'])

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(config_rviz.perform(context))],
        condition=IfCondition(LaunchConfiguration('with_rviz'))
    )

    return [rviz2]


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
            DeclareLaunchArgument(
                'config_param_file',
                default_value='param/defaults.param.yaml',
                description='Node config (relative path).'
            )
    )

    declared_arguments.append(
            DeclareLaunchArgument(
                'with_rviz',
                default_value='False',
                description='Run RViz2.'
            )
    )

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
