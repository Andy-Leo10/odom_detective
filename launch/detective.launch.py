from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'real_life',
            default_value='false',
            description='Set to true for real-life data, false for simulation'
        ),
        DeclareLaunchArgument(
            'odom_topics',
            default_value='["/odom_matcher"]',
            description='List of odometry topics to subscribe to'
        ),
        Node(
            package='odom_detective',
            executable='detective_node',
            parameters=[{
                'real_life': LaunchConfiguration('real_life'),
                'odom_topics': LaunchConfiguration('odom_topics')
            }],
            output='screen'
        )
    ])