from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    turtlebot4 = ExecuteProcess(
        cmd=[
            'ros2',
            'launch',
            'turtlebot4_gz_bringup',
            'turtlebot4_gz.launch.py',
            'world:=/src/project1_ros/project1_world.sdf',
            'x:=0.0',
            'y:=0.0',
            'yaw:=0.0'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        rviz,
        turtlebot4
    ])
