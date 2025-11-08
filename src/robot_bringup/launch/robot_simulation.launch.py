from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Resolve package share and URDF/RViz paths
    pkg_share = get_package_share_directory('robot_bringup')
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'simulation.rviz')

    # Read URDF contents and pass as parameter to robot_state_publisher
    robot_description = ''
    try:
        with open(urdf_file_path, 'r') as fh:
            robot_description = fh.read()
    except Exception as e:
        # Let the launch still run; robot_state_publisher will warn if empty
        print(f'Warning: failed to read URDF file: {e}')

    return LaunchDescription([
        Node(
            package='robot_simulator_cpp',
            executable='odometry_node',
            output='screen'
        ),
        Node(
            package='robot_simulator_py',
            executable='controller_node',
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
    ])
