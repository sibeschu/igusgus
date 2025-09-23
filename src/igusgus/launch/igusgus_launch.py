from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    igus_dir = PathJoinSubstitution([FindPackageShare('igus_rebel'), 'launch'])
    igusmoveit_dir = PathJoinSubstitution([FindPackageShare('igus_rebel_moveit_config'), 'launch'])
    camera_dir = PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch'])
    yolo_dir = PathJoinSubstitution([FindPackageShare('yolo_bringup'), 'launch'])

    camera_to_link6_rel = TimerAction(
    period=10.0, # delay in s
    actions=[
    Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '-1.57', '0', 'link6', 'camera_link']
        )]
    )


    return LaunchDescription([

        IncludeLaunchDescription(
            PathJoinSubstitution([igus_dir, 'rebel.launch.py'])
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([igusmoveit_dir, 'igus_rebel_motion_planner.launch.py']),
            launch_arguments={'use_gui': 'true'}.items(),
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([camera_dir, 'rs_launch.py']),
            launch_arguments={
            'depth_module.depth_profile': '424x240x30',
            'rgb_camera.color_profile': '424x240x30',
            'pointcloud.enable': 'true',
            }.items()
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([yolo_dir, 'yolov8.launch.py']),
            launch_arguments={
                'model': 'best.pt',
                'use_3d': 'True',
                'imgsz_width': '640',
                'imgsz_height': '480',
                'input_image_topic': '/camera/camera/color/image_raw',
                'input_depth_topic': '/camera/camera/depth/image_rect_raw',
                'input_depth_info_topic': '/camera/camera/depth/camera_info',
                'target_frame': 'camera_link',
                'device': 'cpu',
            }.items()
        ),
        camera_to_link6_rel
    ])
