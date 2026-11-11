"""Robot side (Raspberry Pi): camera publisher + motion subscriber.

    ros2 launch pirobot robot.launch.py
    ros2 launch pirobot robot.launch.py fps:=15 hflip:=true vflip:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = {
        'width': '320', 'height': '240', 'fps': '15.0', 'jpeg_quality': '80',
        'hflip': 'false', 'vflip': 'false', 'full_fov': 'true',
        'timeout': '0.5',
    }
    decl = [DeclareLaunchArgument(k, default_value=v) for k, v in args.items()]
    lc = {k: LaunchConfiguration(k) for k in args}

    def typed(name, t):
        return ParameterValue(lc[name], value_type=t)

    camera = Node(
        package='pirobot', executable='picam_publisher', output='screen',
        parameters=[{
            'width': typed('width', int), 'height': typed('height', int),
            'fps': typed('fps', float), 'jpeg_quality': typed('jpeg_quality', int),
            'hflip': typed('hflip', bool), 'vflip': typed('vflip', bool),
            'full_fov': typed('full_fov', bool),
        }],
    )
    motion = Node(
        package='pirobot', executable='motion_subscriber', output='screen',
        parameters=[{'timeout': typed('timeout', float)}],
    )
    return LaunchDescription(decl + [camera, motion])
