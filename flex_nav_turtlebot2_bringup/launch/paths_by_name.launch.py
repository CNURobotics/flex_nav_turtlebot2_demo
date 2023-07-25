# Copyright 2022, CHRISLab, Christopher Newport University
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    bringup_dir = get_package_share_directory('flex_nav_turtlebot2_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')  # For simulations

    # Load the high_level planner and costmap params
    yaml_paths_file = os.path.join(bringup_dir, "paths", "creech_path.yaml")

    param_substitutions = {'use_sim_time': use_sim_time,
                           'yaml_paths_file': yaml_paths_file,
                           'plan_topic': '/high_level_planner/plan'
                           }

    # Set up the nodes for launch
    paths_by_name = Node(package='flex_nav_planners',
                         executable='paths_by_name',
                         name='paths_by_name',
                         output='screen',
                         parameters=[param_substitutions],
                         )

    ld = LaunchDescription()
    ld.add_action(paths_by_name)

    return ld
