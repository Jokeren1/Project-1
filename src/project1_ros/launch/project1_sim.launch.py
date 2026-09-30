# Copyright 2023 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0.
#
# Adapted from:
# turtlebot4_gz_bringup/launch/sim.launch.py
#
# Modification:
# Added the project1_ros worlds directory to GZ_SIM_RESOURCE_PATH
# so Gazebo can locate the custom Project 1 world.

import os

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Use simulation time'
    ),

    DeclareLaunchArgument(
        'world',
        default_value='project1',
        description='Simulation world'
    ),

    DeclareLaunchArgument(
        'model',
        default_value='standard',
        choices=['standard', 'lite'],
        description='TurtleBot 4 model'
    ),
]


def generate_launch_description():

    # Project package.
    pkg_project1_ros = get_package_share_directory(
        'project1_ros'
    )

    # TurtleBot / Gazebo packages.
    pkg_turtlebot4_gz_bringup = get_package_share_directory(
        'turtlebot4_gz_bringup'
    )

    pkg_turtlebot4_gz_gui_plugins = get_package_share_directory(
        'turtlebot4_gz_gui_plugins'
    )

    pkg_turtlebot4_description = get_package_share_directory(
        'turtlebot4_description'
    )

    pkg_irobot_create_description = get_package_share_directory(
        'irobot_create_description'
    )

    pkg_irobot_create_gz_bringup = get_package_share_directory(
        'irobot_create_gz_bringup'
    )

    pkg_irobot_create_gz_plugins = get_package_share_directory(
        'irobot_create_gz_plugins'
    )

    pkg_ros_gz_sim = get_package_share_directory(
        'ros_gz_sim'
    )

    # Gazebo normally searches only the TurtleBot and Create 3
    # resource directories. The project worlds directory is added
    # first so project1.sdf can be found.
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(pkg_project1_ros, 'worlds'),

            os.path.join(
                pkg_turtlebot4_gz_bringup,
                'worlds'
            ),

            os.path.join(
                pkg_irobot_create_gz_bringup,
                'worlds'
            ),

            str(
                Path(
                    pkg_turtlebot4_description
                ).parent.resolve()
            ),

            str(
                Path(
                    pkg_irobot_create_description
                ).parent.resolve()
            ),
        ])
    )

    gz_gui_plugin_path = SetEnvironmentVariable(
        name='GZ_GUI_PLUGIN_PATH',
        value=':'.join([
            os.path.join(
                pkg_turtlebot4_gz_gui_plugins,
                'lib'
            ),

            os.path.join(
                pkg_irobot_create_gz_plugins,
                'lib'
            ),
        ])
    )

    gz_sim_launch = PathJoinSubstitution([
        pkg_ros_gz_sim,
        'launch',
        'gz_sim.launch.py'
    ])

    # Gazebo adds ".sdf" to the world argument.
    # Therefore world:=project1 loads project1.sdf.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [gz_sim_launch]
        ),

        launch_arguments=[
            (
                'gz_args',
                [
                    LaunchConfiguration('world'),
                    '.sdf',
                    ' -r',
                    ' -v 4',
                    ' --gui-config ',
                    PathJoinSubstitution([
                        pkg_turtlebot4_gz_bringup,
                        'gui',
                        LaunchConfiguration('model'),
                        'gui.config'
                    ])
                ]
            )
        ]
    )

    # Gazebo simulation clock -> ROS.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock'
            '@rosgraph_msgs/msg/Clock'
            '[gz.msgs.Clock'
        ]
    )

    ld = LaunchDescription(ARGUMENTS)

    ld.add_action(gz_resource_path)
    ld.add_action(gz_gui_plugin_path)
    ld.add_action(gazebo)
    ld.add_action(clock_bridge)

    return ld