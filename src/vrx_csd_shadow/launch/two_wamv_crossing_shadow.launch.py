import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("vrx_csd_shadow")
    vrx_share = get_package_share_directory("vrx_gz")
    names = ["wamv_1", "wamv_2"]
    log_dir = LaunchConfiguration("log_dir")

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vrx_share, "launch", "competition.launch.py")
        ),
        launch_arguments={
            "world": "sydney_regatta",
            "sim_mode": "full",
            "config_file": os.path.join(share, "config", "two_wamv_crossing.yaml"),
            "bridge_competition_topics": "true",
            "competition_mode": "false",
        }.items(),
    )

    driver = Node(
        package="vrx_csd_shadow",
        executable="two_vessel_crossing_driver.py",
        output="screen",
        parameters=[{"log_dir": log_dir}],
    )
    shadow = Node(
        package="vrx_csd_shadow",
        executable="csd_shadow_graph.py",
        name="csd_shadow_graph",
        output="screen",
        parameters=[{
            "vessel_names": names,
            "model_path": os.path.join(share, "models", "conditional_strict_model.json"),
            "log_dir": log_dir,
            "world_origin_x": 0.0,
            "world_origin_y": 0.0,
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument("log_dir", default_value="/tmp/vrx_two_wamv_crossing_shadow_v1"),
        simulator,
        driver,
        shadow,
    ])
