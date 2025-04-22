from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
import xacro
import os

def generate_launch_description():
    # Hent inn våre paths som vanlige strings
    bringup_share = get_package_share_directory('qube_bringup')
    driver_share  = get_package_share_directory('qube_driver')

    urdf_path = os.path.join(bringup_share, 'urdf', 'controlled_qube.urdf.xacro')
    robot_description = xacro.process_file(urdf_path).toxml()

    # joint_controllers.yaml fra qube_driver
    driver_cfg = os.path.join(driver_share, 'config', 'joint_controllers.yaml')

    return LaunchDescription([

        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # ros2_control_node med riktig navn *controller_manager* + YAML
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',       # <-- SÅNN at YAML-toppen matcher
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                driver_cfg,
            ],
        ),

        # Spawn velocity_controller
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'controller_manager', 'spawner',
                'velocity_controller',
                '--controller-manager', '/controller_manager'
            ],
            output='screen'
        ),

        # Spawn joint_state_broadcaster
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'controller_manager', 'spawner',
                'joint_state_broadcaster',
                '--controller-manager', '/controller_manager'
            ],
            output='screen'
        ),

        # Din PID‐controller
        Node(
            package='qube_controller',
            executable='controller_node',
            name='qube_controller',
            output='screen',
            parameters=[{'setpoint': 1.0}],
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(bringup_share, 'rviz', 'view.rviz')],
        ),
    ])