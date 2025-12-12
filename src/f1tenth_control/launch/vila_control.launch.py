from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/camera/color/image_raw',
            description='相机话题'
        ),
        
        DeclareLaunchArgument(
            'drive_topic',
            default_value='/drive',
            description='驱动话题'
        ),
        
        DeclareLaunchArgument(
            'check_interval',
            default_value='1.0',
            description='检查间隔（秒）'
        ),
        
        DeclareLaunchArgument(
            'safe_speed',
            default_value='1.0',
            description='安全速度 (m/s)'
        ),
        
        Node(
            package='f1tenth_control',
            executable='vila_f1tenth_node.py',
            name='vila_controller',
            output='screen',
            parameters=[{
                'camera_topic': LaunchConfiguration('camera_topic'),
                'drive_topic': LaunchConfiguration('drive_topic'),
                'check_interval': LaunchConfiguration('check_interval'),
                'safe_speed': LaunchConfiguration('safe_speed'),
                'model': 'Efficient-Large-Model/VILA-2.7b'
            }]
        )
    ])
