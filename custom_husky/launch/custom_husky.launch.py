import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('custom_husky'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    control_yaml = PathJoinSubstitution([
        FindPackageShare("custom_husky"),
        "config",
        "control.yaml"
    ])

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    world = os.path.join(
        get_package_share_directory('custom_husky'),
        'worlds',
        'empty_world.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_custom_husky_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_custom_husky.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    spawn_diff_drive_controller = Node(
        package = "controller_manager",
        executable = "spawner",
        arguments = ["diff_drive_controller", "-c", "/controller_manager"],
        output = "screen",
    )

    spawn_joint_state_broadcaster = Node(
        package = "controller_manager",
        executable = "spawner",
        arguments = ["joint_state_broadcaster", "-c", "/controller_manager"],
        output = "screen",
    )

    ros2_controller_manager = Node(
        package = "controller_manager",
        executable = "ros2_control_node",
        parameters = [control_yaml],
        output = "screen"
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_custom_husky_cmd)
    ld.add_action(ros2_controller_manager)
    ld.add_action(
        TimerAction(
            period=1.0,
            actions=[
                spawn_joint_state_broadcaster,
                spawn_diff_drive_controller
            ]
        )
    )

    return ld