from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('task_1_wheeled_robot')

    world = os.path.join(pkg_path, 'world', 'world.sdf')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py',
                 f'gz_args:={world}'],
            output='screen'
        ),
        # Bridge between ROS 2 and Gazebo
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'
            ],
            output='screen'
        ),

        # Motion node (publishes Twist messages)
        Node(
            package='task_1_wheeled_robot',
            executable='motion_node',
            output='screen'
        )
    ])
