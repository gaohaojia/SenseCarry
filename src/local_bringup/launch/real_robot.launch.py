import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")
    robot_type = LaunchConfiguration("robot_type")
    lidar_type = LaunchConfiguration("lidar_type")

    declare_robot_id = DeclareLaunchArgument(
        "robot_id", default_value="0", description=""
    )
    declare_robot_type = DeclareLaunchArgument(
        "robot_type", default_value="simulated", description=""
    )
    declare_lidar_type = DeclareLaunchArgument(
        "lidar_type", default_value="mid360", description=""
    )

    optional_nodes = []
    try:
        start_livox_mid360 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("livox_ros_driver2"),
                    "launch",
                    "msg_MID360_launch.py",
                )
            )
        )
        optional_nodes.append(start_livox_mid360)
    except:
        print("Not found livox_ros_driver2 package.")

    try:
        start_unilidar_lidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("unitree_lidar_ros2"),
                    "launch.py",
                )
            )
        )
        optional_nodes.append(start_unilidar_lidar)
    except:
        print("Not found unitree_lidar_ros2 package.")

    try:
        start_starros_lidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("starros"),
                    "launch",
                    "Dome.launch.py"
                )
            )
        )
        optional_nodes.append(start_starros_lidar)
    except:
        print("Not found starros package.")

    try:
        start_realsense = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("realsense2_camera"),
                    "launch",
                    "rs_launch.py",
                )
            ),
            launch_arguments={"pointcloud.enable": "true"}.items(),
        )
        optional_nodes.append(start_realsense)
    except:
        print("Not found realsense2_camera package.")

    start_lidar_transform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("lidar_transform"),
                "launch",
                "lidar_transform.launch.py",
            )
        ),
        launch_arguments={
            "robot_type": robot_type,
        }.items(),
    )

    start_point_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("point_lio"),
                "launch",
                "point_lio.launch.py",
            )
        ),
        launch_arguments={
            "lidar_type": lidar_type,
        }.items(),
    )

    start_robot_communication = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_communication"),
                "launch",
                "robot_communication.launch.py",
            )
        ),
        launch_arguments={
            "robot_id": robot_id,
        }.items(),
    )

    start_rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("local_bringup"), "rviz", "real_robot.rviz"
            ),
        ],
        output="screen",
    )

    ld = LaunchDescription()

    # Add the actions
    ld.add_action(declare_robot_id)
    ld.add_action(declare_robot_type)
    ld.add_action(declare_lidar_type)

    for node in optional_nodes:
        ld.add_action(node)
    ld.add_action(start_lidar_transform)
    ld.add_action(start_point_lio)
    ld.add_action(TimerAction(period=10.0, actions=[start_robot_communication]))
    ld.add_action(TimerAction(period=8.0, actions=[start_rviz]))

    return ld
