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

    declare_robot_id = DeclareLaunchArgument(
        "robot_id", default_value="0", description=""
    )
    declare_lidar_type = DeclareLaunchArgument(
        "lidar_type", default_value="mid360", description=""
    )
    declare_lio_mode = DeclareLaunchArgument(
        "lio_mode", default_value="point_lio", description=""
    )
    declare_realsense_mode = DeclareLaunchArgument(
        "realsense_mode", default_value="rs_d435", description=""
    )

    start_livox_mid360 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("livox_ros_driver2"),
                "launch",
                "msg_MID360_launch.py",
            )
        ),
        condition=LaunchConfigurationEquals("lidar_type", "mid360"),
    )

    start_unilidar_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("unitree_lidar_ros2"),
                "launch.py",
            )
        ),
        condition=LaunchConfigurationEquals("lidar_type", "unilidar"),
    )

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
            condition=LaunchConfigurationEquals("realsense_mode", "rs_d435"),
        )
    except:
        pass

    start_lidar_transform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("lidar_transform"),
                "launch",
                "lidar_transform.launch.py",
            )
        )
    )

    start_fast_lio = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("fast_lio"),
                        "launch",
                        "mapping_mid360.launch.py",
                    )
                ),
                condition=LaunchConfigurationEquals("lidar_type", "mid360"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("fast_lio"),
                        "launch",
                        "mapping_unilidar.launch.py",
                    )
                ),
                condition=LaunchConfigurationEquals("lidar_type", "unilidar"),
            ),
        ],
        condition=LaunchConfigurationEquals("lio_mode", "fast_lio"),
    )

    start_point_lio = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("point_lio"),
                        "launch",
                        "mapping_mid360.launch.py",
                    )
                ),
                condition=LaunchConfigurationEquals("lidar_type", "mid360"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("point_lio"),
                        "launch",
                        "mapping_unilidar.launch.py",
                    )
                ),
                condition=LaunchConfigurationEquals("lidar_type", "unilidar"),
            ),
        ],
        condition=LaunchConfigurationEquals("lio_mode", "point_lio"),
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
    ld.add_action(declare_lidar_type)
    ld.add_action(declare_lio_mode)
    ld.add_action(declare_realsense_mode)

    try:
        ld.add_action(start_realsense)
    except:
        pass
    ld.add_action(start_livox_mid360)
    ld.add_action(start_unilidar_lidar)
    ld.add_action(start_lidar_transform)
    ld.add_action(start_fast_lio)
    ld.add_action(start_point_lio)
    ld.add_action(TimerAction(period=10.0, actions=[start_robot_communication]))
    ld.add_action(TimerAction(period=8.0, actions=[start_rviz]))

    return ld
