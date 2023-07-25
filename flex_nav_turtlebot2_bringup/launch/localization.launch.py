# Copyright 2022 CHRISLab, Christopher Newport University
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
# Author: David Conner

# Based on https://www.robotandchisel.com/2020/08/19/slam-in-ros2/

import os
import yaml

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    package_map_file = 'chris_world_models/maps/creech_map_050'
    initial_pose = LaunchConfiguration('use_sim_time', default='[0.0, 0.0, 0.0]')

    package_name = package_map_file.split("/")[0]
    package_dir = get_package_share_directory(package_name)
    map_file_path = os.path.join(package_dir, *package_map_file.split("/")[1:])
    print("Map file: ", map_file_path)

    bring_up_path = get_package_share_directory("flex_nav_turtlebot2_bringup")
    print("bring up: ", bring_up_path)

    params_file = os.path.join(bring_up_path, 'param', 'localization.yaml')
    print("Parameters file: ", params_file)
    with open(params_file, 'r') as f:
        localization_params = yaml.safe_load(f)['slam_toolbox']['ros__parameters']

    localization_params['use_sim_time'] = use_sim_time
    localization_params['map_file_name'] = map_file_path
    localization_params['map_start_pose'] = initial_pose

    start_sync_slam_toolbox_node = Node(
        parameters=[localization_params],
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    ld = LaunchDescription()
    ld.add_action(start_sync_slam_toolbox_node)

    return ld
