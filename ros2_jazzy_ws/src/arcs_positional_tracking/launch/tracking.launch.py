from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Path to RViz config
    rviz_config_path = PathJoinSubstitution([
        get_package_share_directory('arcs_positional_tracking'),
        'rviz',
        'pos_tracking_config.rviz'
    ])

    return LaunchDescription([
        # Fake ZED Pose Publisher
        Node(
            package='arcs_positional_tracking',
            executable='fake_pose_pub',
            name='fake_pose_pub',
            output='screen'
        ),

        # Positional Tracking Node
        Node(
            package='arcs_positional_tracking',
            executable='positional_tracking_node',
            name='positional_tracking_node',
            output='screen'
        ),

        # RVIZ2 with custom config
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            output='screen'
        )
    ])
