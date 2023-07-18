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

    point_to_point_demo = Node(
        package="any_robot_pick_n_place",
        executable="simple_pick_n_place_node",
        name="simple_pick_n_place_demo",
        output="screen",
        # prefix=["gdb -ex run --args"],
        # prefix=["xterm -e gdb -ex run --args"],
        parameters=[
            os.path.join(
                get_package_share_directory("any_robot_pick_n_place"),
                "config",
                "simple_pick_n_place.yaml",
            ),
            moveit_config.to_dict(),
        ],
    )

    return LaunchDescription([point_to_point_demo])
