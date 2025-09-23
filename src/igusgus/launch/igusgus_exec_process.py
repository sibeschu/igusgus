from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def term(cmd: str):
    # Use gnome-terminal; replace with 'xterm -hold -e' or 'konsole -e' if needed
    return ['gnome-terminal', '--', 'bash', '-lc', f'{cmd}; exec bash']

def generate_launch_description():
    # 10s delayed static TF stays as a Node (or wrap with ExecuteProcess similarly)
    camera_to_link6_rel = TimerAction(
        period=10.0,
        actions=[Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0','0','0','0','-1.57','0','link6','camera_link']
        )]
    )

    return LaunchDescription([
        # igus robot
        ExecuteProcess(cmd=term('ros2 launch igus_rebel rebel.launch.py')),

        # MoveIt motion planner (with GUI)
        ExecuteProcess(cmd=term('ros2 launch igus_rebel_moveit_config igus_rebel_motion_planner.launch.py use_gui:=true')),

        # RealSense
        ExecuteProcess(cmd=term(
            'ros2 launch realsense2_camera rs_launch.py '
            'depth_module.depth_profile:=424x240x30 '
            'rgb_camera.color_profile:=424x240x30 '
            'pointcloud.enable:=true'
        )),

        # YOLOv8 (matches your working CLI)
        ExecuteProcess(cmd=term(
            'ros2 launch yolo_bringup yolov8.launch.py '
            'model:=best.pt use_3d:=True imgsz_width:=640 imgsz_height:=480 '
            'input_image_topic:=/camera/camera/color/image_raw '
            'input_depth_topic:=/camera/camera/depth/image_rect_raw '
            'input_depth_info_topic:=/camera/camera/depth/camera_info '
            'target_frame:=camera_link device:=cpu'
        )),

        camera_to_link6_rel,
    ])
