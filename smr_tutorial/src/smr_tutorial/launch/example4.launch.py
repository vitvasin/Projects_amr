from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node


def generate_launch_description():

    example_node =  Node(
        package='smr_tutorial',
        executable='example4',
        name='example4',
        output='screen'
    ) 

    interface_node =  Node(
        package='hardware_interface_cpp',
        executable='bidirectional_interface',
        name='bidirectional_interface',
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
