import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from vrx_gz.model import Model


def generate_launch_description():
    share = get_package_share_directory("vrx_csd_shadow")
    vrx_share = get_package_share_directory("vrx_gz")
    csv_path = LaunchConfiguration("csv_path")
    playback_speed = LaunchConfiguration("playback_speed")
    hold_seconds = LaunchConfiguration("hold_seconds")

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vrx_share, "launch", "competition.launch.py")),
        launch_arguments={
            "world": os.path.join(share, "worlds", "csd_harbor.sdf"),
            "sim_mode": "full",
            "config_file": os.path.join(share, "config", "empty_models.yaml"),
            "bridge_competition_topics": "false",
            "competition_mode": "false",
            "paused": "true",
            "headless": "false",
        }.items(),
    )

    with open(os.path.join(share, "config", "harbor_wamv.yaml"), "r") as stream:
        models = Model.FromConfig(stream)

    spawns = []
    for index, model in enumerate(models):
        spawns.append(TimerAction(
            period=3.0 + 4.0 * index,
            actions=[Node(
                package="vrx_csd_shadow",
                executable="spawn_model_retry.py",
                output="screen",
                arguments=model.spawn_args(),
            )],
        ))

    replay = Node(
        package="vrx_csd_shadow",
        executable="replay_historical_trajectory.py",
        output="screen",
        arguments=[
            "--csv", csv_path,
            "--speed", playback_speed,
            "--hold-seconds", hold_seconds,
        ],
        on_exit=Shutdown(reason="Historical C05 replay completed"),
    )

    return LaunchDescription([
        DeclareLaunchArgument("csv_path"),
        DeclareLaunchArgument("playback_speed", default_value="1.0"),
        DeclareLaunchArgument("hold_seconds", default_value="30.0"),
        simulator,
        *spawns,
        TimerAction(period=29.0, actions=[replay]),
    ])
