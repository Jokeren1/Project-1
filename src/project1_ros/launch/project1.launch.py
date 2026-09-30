from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():

    # ---------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------

    x_arg = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='Initial X position of the TurtleBot'
    )

    y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='Initial Y position of the TurtleBot'
    )

    z_arg = DeclareLaunchArgument(
        'z',
        default_value='0.0',
        description='Initial Z position of the TurtleBot'
    )

    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Initial yaw angle of the TurtleBot'
    )

    # ---------------------------------------------------------
    # Package locations
    # ---------------------------------------------------------

    project_share = get_package_share_directory(
        'project1_ros'
    )

    turtlebot_share = get_package_share_directory(
        'turtlebot4_gz_bringup'
    )

    navigation_share = get_package_share_directory(
        'turtlebot4_navigation'
    )

    project_sim_launch = os.path.join(
        project_share,
        'launch',
        'project1_sim.launch.py'
    )

    turtlebot_spawn_launch = os.path.join(
        turtlebot_share,
        'launch',
        'turtlebot4_spawn.launch.py'
    )

    slam_launch = os.path.join(
        navigation_share,
        'launch',
        'slam.launch.py'
    )

    # ---------------------------------------------------------
    # Gazebo + custom world
    # ---------------------------------------------------------

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            project_sim_launch
        ),

        launch_arguments={
            'world': 'project1',
            'model': 'standard',
            'use_sim_time': 'true',
        }.items()
    )

    # ---------------------------------------------------------
    # Spawn TurtleBot
    # ---------------------------------------------------------

    turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            turtlebot_spawn_launch
        ),

        launch_arguments={
            'namespace': '',
            'rviz': 'false',

            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
        }.items()
    )

    # ---------------------------------------------------------
    # Project ROS nodes
    # ---------------------------------------------------------

    reactive_controller = Node(
        package='project1_ros',
        executable='reactive_controller',
        name='reactive_controller',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    distance_tracker = Node(
        package='project1_ros',
        executable='distance_tracker',
        name='distance_tracker',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    # ---------------------------------------------------------
    # SLAM
    # ---------------------------------------------------------

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            slam_launch
        ),

        launch_arguments={
            'use_sim_time': 'true',
        }.items()
    )

    # ---------------------------------------------------------
    # RViz
    # ---------------------------------------------------------

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    # ---------------------------------------------------------
    # Launch everything
    # ---------------------------------------------------------

    return LaunchDescription([
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,

        simulation,
        turtlebot,

        reactive_controller,
        distance_tracker,

        slam,
        rviz,
    ])