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
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

def generate_launch_description():

    bringup_dir = get_package_share_directory('flex_nav_turtlebot2_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true') # For simulations
    autostart = LaunchConfiguration('autostart', default='true')

    hlp_name = "high_level_planner"
    gcm_name = 'global_costmap'
    llp_name = 'low_level_planner'
    lcm_name = 'local_costmap'
    rsrv_name = 'recovery_server'

    # Load the high_level planner and costmap params
    yaml_paths_file = os.path.join(bringup_dir, "paths", "creech_path.yaml")

    param_substitutions = {
                            'use_sim_time': use_sim_time,
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
