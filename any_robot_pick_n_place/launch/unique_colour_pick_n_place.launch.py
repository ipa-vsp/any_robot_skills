# Copyright 2023 Vishnuprasad Prachandabhanu
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
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("prbt_cell", package_name="prbt_cell_moveit_config")
        .robot_description(
            file_path=get_package_share_directory("prbt_cell_description")
            + "/urdf/prbt_cell.urdf.xacro"
        )
        .moveit_cpp(
            file_path=get_package_share_directory("any_robot_pick_n_place")
            + "/config/moveitcpp.yaml"
        )
        .to_moveit_configs()
    )

    bt_tree_path = os.path.join(get_package_share_directory("any_robot_pick_n_place"), "config", "unique_colour", "colour_pick_n_place.xml")
    point_to_point_demo = Node(
        package="any_robot_pick_n_place",
        executable="simple_pick_n_place_demo",
        name="unique_color_pick_n_place",
        output="screen",
        # prefix=["gdb -ex run --args"],
        # prefix=["xterm -e gdb -ex run --args"],
        parameters=[
            os.path.join(
                get_package_share_directory("any_robot_pick_n_place"),
                "config",
                "unique_colour", 
                "colour_pick_n_place.yaml",
            ),
            moveit_config.to_dict(),
            #{ "behavior_tree_file": bt_tree_path },
        ],
    )

    return LaunchDescription([point_to_point_demo])
