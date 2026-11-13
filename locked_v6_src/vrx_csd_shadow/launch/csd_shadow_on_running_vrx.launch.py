from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    share = get_package_share_directory("vrx_csd_shadow")
    names = [
        "wamv", "ci_nav_dynamic_circle_obstacle",
        "ci_nav_dynamic_circle_obstacle_2", "ci_nav_dynamic_circle_obstacle_3",
    ]
    bridges = [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=f"csd_shadow_pose_bridge_{index}",
            output="screen",
            arguments=[
                f"/model/{name}/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V"
            ],
        )
        for index, name in enumerate(names)
    ]
    shadow = Node(
        package="vrx_csd_shadow",
        executable="csd_shadow_graph.py",
        name="csd_shadow_graph",
        output="screen",
        parameters=[{
            "vessel_names": names,
            "model_path": os.path.join(share, "models", "conditional_strict_model.json"),
            "log_dir": LaunchConfiguration("log_dir"),
            "world_origin_x": LaunchConfiguration("world_origin_x"),
            "world_origin_y": LaunchConfiguration("world_origin_y"),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument("log_dir", default_value="/tmp/vrx_csd_shadow_v1"),
        DeclareLaunchArgument("world_origin_x", default_value="0.0"),
        DeclareLaunchArgument("world_origin_y", default_value="0.0"),
        *bridges,
        shadow,
    ])
