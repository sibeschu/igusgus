from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    env = {
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': ':'.join([
            os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
            os.environ.get('LD_LIBRARY_PATH', default='')]
        ),
    }
    
    hardware_protocol_arg = DeclareLaunchArgument(
        "hardware_protocol",
        default_value="rebel",
        choices=["mock_hardware", "gazebo", "rebel"],
        description="Which hardware protocol or mock hardware should be used",
    )
    
    igus_rebel_pkg = get_package_share_directory("igus_rebel")
    igus_rebel_desc_pkg = get_package_share_directory("igus_rebel_description")
    
    controller_config_path = os.path.join(igus_rebel_pkg, "config", "controller.yaml")

    robot_description_file = os.path.join(
        igus_rebel_desc_pkg,
        'urdf',
        'igus_rebel_robot2.urdf.xacro'
    )

    robot_description = {"robot_description" : Command(
        [
            FindExecutable(name="xacro"),
            " ",
            robot_description_file,
            " hardware_protocol:=",
            LaunchConfiguration("hardware_protocol"),
        ]
    )}
        
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controller_config_path],
        remappings=[
        ],
        output="both",
    )
    
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        remappings=[],
        output="both",
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="both",
    )
    
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rebel_arm_controller", "--controller-manager", "/controller_manager"],
        output="both",
    )
    
    return LaunchDescription([
        hardware_protocol_arg,
        ros2_control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
    ])
    