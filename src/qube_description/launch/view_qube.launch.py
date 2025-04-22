from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('qube_description')
    xacro_path = os.path.join(pkg_path, 'urdf', 'qube.urdf.xacro')

    # Konverter Xacro til URDF-streng
    robot_description = xacro.process_file(xacro_path).toxml()
    print("=== ROBOT DESCRIPTION START ===")
    print(robot_description)
    print("=== ROBOT DESCRIPTION END ===")

    return LaunchDescription([
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])