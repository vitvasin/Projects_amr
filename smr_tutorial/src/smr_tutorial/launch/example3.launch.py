from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node


def generate_launch_description():

    example_node =  Node(
        package='smr_tutorial',
        executable='example3',
        name='example3',
        output='screen'
    ) 

    interface_node =  Node(
        package='hardware_interface_cpp',
        executable='cmd_vel_interface',
        name='cmd_vel_interface',
        output='screen'
    ) 

    launch_elements = GroupAction(
        actions=[
            example_node,
            interface_node
        ]
    )

    return LaunchDescription([
        launch_elements
    ])
