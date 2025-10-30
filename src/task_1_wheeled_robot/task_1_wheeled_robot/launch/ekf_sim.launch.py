import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('task_1_wheeled_robot')
    world = os.path.join(pkg_path, 'world', 'world.sdf')

    # Declare a flag to enable/disable EKF
    declare_use_ekf = DeclareLaunchArgument(
        'use_ekf',
        default_value='true',
        description='Use EKF for odometry filtering'
    )
    use_ekf = LaunchConfiguration('use_ekf')

    return LaunchDescription([
        declare_use_ekf,

        # Gazebo world
        ExecuteProcess(
            cmd=['ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py', f'gz_args:={world}'],
            output='screen'
        ),

        # Bridge between ROS 2 and Gazebo
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                '/model/vehicle_blue/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry'
            ],
            output='screen'
        ),

        # Noise injector node
        Node(
            package='task_1_wheeled_robot',
            executable='noise_injector_node',
            name='noise_injector_node',
            output='screen',
        ),

        # EKF node (only if enabled)
        Node(
            package='task_1_wheeled_robot',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            condition=launch.conditions.IfCondition(use_ekf),
        ),

        # Motion node
        Node(
            package='task_1_wheeled_robot',
            executable='motion_node',
            name='motion_node',
            output='screen',
            parameters=[{'USE_EKF': use_ekf}]
        ),

        # Live plotter node
        Node(
            package='task_1_wheeled_robot',
            executable='live_plotter',
            name='live_plotter',
            output='screen',
        ),
    ])
