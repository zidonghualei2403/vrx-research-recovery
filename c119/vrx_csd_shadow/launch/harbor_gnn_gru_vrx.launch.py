import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    Shutdown,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from vrx_gz.model import Model


def generate_launch_description():
    share = get_package_share_directory("vrx_csd_shadow")
    vrx_share = get_package_share_directory("vrx_gz")
    names = [f"wamv_{index}" for index in range(1, 7)]
    log_dir = LaunchConfiguration("log_dir")
    world_file = LaunchConfiguration("world_file")
    control_mode = LaunchConfiguration("control_mode")
    graph_mode = LaunchConfiguration("graph_mode")
    encounter_mode = LaunchConfiguration("encounter_mode")
    max_experiment_duration = LaunchConfiguration("max_experiment_duration")
    goal_slowdown_radius = LaunchConfiguration("goal_slowdown_radius")
    environment_uncertainty = LaunchConfiguration("environment_uncertainty")
    headless = LaunchConfiguration("headless")
    sea_state = LaunchConfiguration("sea_state")
    residual_sac_authority = LaunchConfiguration("residual_sac_authority")
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(vrx_share, "launch", "competition.launch.py")),
        launch_arguments={
            "world": world_file,
            "sim_mode": "full",
            "config_file": os.path.join(share, "config", "empty_models.yaml"),
            "bridge_competition_topics": "true",
            "competition_mode": "false",
            "paused": "true",
            "headless": headless,
        }.items(),
    )

    with open(os.path.join(share, "config", "harbor_wamv.yaml"), "r") as stream:
        models = Model.FromConfig(stream)

    sequential_actions = []
    for index, model in enumerate(models):
        # Give Gazebo enough time to finish each physical WAM-V insertion.
        # The helper retries transient create-service failures without changing
        # the validated controller or learned policy.
        delay = 3.0 + 4.0 * index
        spawn = Node(
            package="vrx_csd_shadow",
            executable="spawn_model_retry.py",
            output="screen",
            arguments=model.spawn_args(),
        )

        name = model.model_name
        gps_gz = (
            f"/world/csd_harbor/model/{name}/link/{name}/gps_wamv_link/"
            "sensor/navsat/navsat"
        )
        imu_gz = (
            f"/world/csd_harbor/model/{name}/link/{name}/imu_wamv_link/"
            "sensor/imu_wamv_sensor/imu"
        )
        left_gz = f"{name}/thrusters/left/thrust"
        right_gz = f"{name}/thrusters/right/thrust"
        bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            namespace=name,
            output="screen",
            arguments=[
                f"{gps_gz}@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat",
                f"{imu_gz}@sensor_msgs/msg/Imu[gz.msgs.IMU",
                f"{left_gz}@std_msgs/msg/Float64]gz.msgs.Double",
                f"{right_gz}@std_msgs/msg/Float64]gz.msgs.Double",
            ],
            remappings=[
                (gps_gz, "sensors/gps/gps/fix"),
                (imu_gz, "sensors/imu/imu/data"),
                (left_gz, "thrusters/left/thrust"),
                (right_gz, "thrusters/right/thrust"),
            ],
        )
        sequential_actions.extend([
            TimerAction(period=delay, actions=[spawn]),
            TimerAction(period=delay + 1.25, actions=[bridge]),
        ])

    driver = Node(
        package="vrx_csd_shadow", executable="six_vessel_crossing_driver.py",
        output="screen", on_exit=Shutdown(
            reason="Six-vessel experiment reached its terminal condition"), parameters=[{
            "log_dir": log_dir,
            "control_mode": control_mode,
            "graph_mode": graph_mode,
            "encounter_mode": encounter_mode,
            "max_experiment_duration": ParameterValue(
                max_experiment_duration, value_type=float),
            "goal_slowdown_radius": ParameterValue(
                goal_slowdown_radius, value_type=float),
            "environment_uncertainty": ParameterValue(
                environment_uncertainty, value_type=float),
            "cooperative_count": 4,
            # One synchronized GPS sample is sufficient because the world is
            # paused during spawning. Waiting for five samples after unpause
            # gave wind and waves an avoidable open-loop interval.
            "required_start_samples": 1,
            "sea_state": sea_state,
            "residual_sac_authority": ParameterValue(
                residual_sac_authority, value_type=float),
            "residual_sac_model_path": os.path.join(
                share, "models", "harbor_residual_sac_actor.json"),
        }],
    )
    shadow = Node(
        package="vrx_csd_shadow", executable="csd_shadow_graph.py",
        name="csd_shadow_graph", output="screen",
        parameters=[{
            "vessel_names": names,
            "model_path": os.path.join(share, "models", "conditional_strict_model.json"),
            "gnn_model_path": os.path.join(share, "models", "graph_conditional_model.json"),
            "learned_gnn_gru_model_path": os.path.join(
                share, "models", "harbor_gnn_gru_best.json"),
            "log_dir": log_dir,
            "world_origin_x": 0.0,
            "world_origin_y": 0.0,
            "graph_mode": graph_mode,
            "encounter_mode": "harbor",
            "cooperative_count": 4,
            "environment_uncertainty": ParameterValue(
                environment_uncertainty, value_type=float),
        }],
    )
    unpause = ExecuteProcess(
        cmd=[
            "gz", "service",
            "-s", "/world/csd_harbor/control",
            "--reqtype", "gz.msgs.WorldControl",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "5000",
            "--req", "pause: false",
        ],
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("log_dir", default_value="/tmp/vrx_six_wamv_crossing_shadow_v1"),
        DeclareLaunchArgument("control_mode", default_value="baseline"),
        DeclareLaunchArgument("graph_mode", default_value="conditional_csd"),
        DeclareLaunchArgument("encounter_mode", default_value="harbor"),
        DeclareLaunchArgument("max_experiment_duration", default_value="300.0"),
        DeclareLaunchArgument("goal_slowdown_radius", default_value="14.0"),
        DeclareLaunchArgument("environment_uncertainty", default_value="0.0"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("sea_state", default_value="calm"),
        DeclareLaunchArgument("residual_sac_authority", default_value="0.35"),
        DeclareLaunchArgument(
            "world_file",
            default_value=os.path.join(share, "worlds", "csd_harbor.sdf"),
        ),
        simulator,
        *sequential_actions,
        TimerAction(period=38.0, actions=[driver, shadow]),
        TimerAction(period=40.0, actions=[unpause]),
    ])
