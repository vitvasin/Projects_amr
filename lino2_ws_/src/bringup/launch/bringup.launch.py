import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml
from launch_ros.descriptions import ParameterFile


def generate_launch_description():

    robot_description_dir = get_package_share_directory("description")
    robot_bringup_dir = get_package_share_directory("bringup")

    params_file = LaunchConfiguration("params_file")
    namespace = LaunchConfiguration("namespace")
    use_rviz = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("sim")
    namespace = LaunchConfiguration("namespace")

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        # default_value= [EnvironmentVariable('NAMESPACE')],
        default_value= '',
        description='prefix for node name')
    
    
    declare_model_cmd = DeclareLaunchArgument(
            name="model", 
            default_value=os.path.join(robot_description_dir, "urdf/robot", "robot.urdf.xacro"),
            description="Absolute path to robot urdf file")
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz') 
    declare_use_sim_time_cmd = DeclareLaunchArgument(
            name='sim', 
            default_value='z',
            description='Enable use_sime_time to true')
    declare_use_namespace_cmd = DeclareLaunchArgument(
            name='use_namespace', 
            default_value='true',
            description='Enable use_sime_time to true')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(robot_bringup_dir, "config", "bringup.yaml"),
        description='Full path to the parameters file')


    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True),
        allow_substs=True)
    
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}])

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",)

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(robot_description_dir, "rviz", "bringup.rviz")],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}])
    
    scan = IncludeLaunchDescription(os.path.join(
        get_package_share_directory("sllidar_ros2"),"launch","sllidar_c1_launch.py"),
        launch_arguments={
                'serial_port': '/dev/rplidar',
        }.items()) 

    camera = Node(
            package='usb_cam', 
            executable='usb_cam_node_exe',
            output='screen',
            name="usb_camera",
            parameters=[configured_params])

    imu_filter = Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[configured_params],)

    robot_localization = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[configured_params],
            remappings=[("odometry/filtered", "odom")]) 

    hardware_node = Node(
        package="robot_hardware_interface",
        executable="robot_hardware",
        name="robot_hardware",
        output="screen",
        parameters=[{'serial_port': "/dev/esp32"}],)

    launch_elements = GroupAction(
     actions=[
        PushRosNamespace(namespace),
        SetRemap('/tf','tf'),
        SetRemap('/tf_static','tf_static'), 
        camera, 
        robot_state_publisher_node,
        joint_state_publisher_node,   
        scan,
        rviz_node, 
        robot_localization,
        imu_filter,
        hardware_node
      ]
   )

    return LaunchDescription([
        # Declare the launch options
        declare_namespace_cmd,
        declare_params_file_cmd,
        declare_model_cmd,
        declare_use_sim_time_cmd,
        declare_use_rviz_cmd,
        declare_use_namespace_cmd,
        #launch all of the bringup nodes
        launch_elements
    ])
