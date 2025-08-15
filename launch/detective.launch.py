from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # PARAMETERS
    declare_real_life = DeclareLaunchArgument(
        'real_life',
        default_value='false',
        description='Set to true for real-life data, false for simulation'
    )

    declare_odom_topics = DeclareLaunchArgument(
        'odom_topics',
        default_value='["/odom_matcher", "/odometry/filtered"]',
        description='List of odometry topics to subscribe to'
    )
    
    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='demo_world',
        description='Name of the world to use for the simulation'
    )

    # NODES
    world_name = LaunchConfiguration('world_name')
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            ['/world/', world_name, '/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V']
        ],
        output='screen'
    )
    
    detective_node = Node(
        package='odom_detective',
        executable='detective_node',
        parameters=[{
            'real_life': LaunchConfiguration('real_life'),
            'odom_topics': LaunchConfiguration('odom_topics'),
            'world_name': LaunchConfiguration('world_name'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        declare_real_life,
        declare_odom_topics,
        declare_world_name,
        bridge_node,
        detective_node
    ])