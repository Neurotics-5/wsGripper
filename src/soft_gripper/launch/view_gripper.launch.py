from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('soft_gripper')
    urdf_path = os.path.join(pkg_path, 'URDF', 'URDF.urdf')

    with open(urdf_path, 'r') as inf:
        robot_description = inf.read()

    # Optional RViz config
    rviz_config_path = os.path.join(pkg_path, 'config', 'default.rviz')
    rviz_args = ['-d', rviz_config_path] if os.path.exists(rviz_config_path) else []

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            remappings=[('/joint_states', '/gripper/joint_states')],
            output='screen',
            name='robot_state_publisher',
        ),

        # 🚫 Removed joint_state_publisher

        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=rviz_args,
                    output='screen'
                )
            ]
        )
    ])
