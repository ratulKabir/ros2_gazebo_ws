from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('task_1_wheeled_robot')

    world = os.path.join(pkg_path, 'world', 'world.sdf')
    urdf = os.path.join(pkg_path, 'urdf', 'diff_drive.urdf')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py',
                 f'gz_args:={world}'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_sim', 'create',
                 '-file', urdf,
                 '-name', 'simple_bot', 
                 '-z', '0.11'],
            output='screen'
        ),
        Node(
            package='task_1_wheeled_robot',
            executable='motion_node',
            output='screen'
        )
    ])
