from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    
    # UR5 Sterownik
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'),
                'launch',
                'ur_control.launch.py'
            ])
        ),
        launch_arguments={
            'ur_type': 'ur5',
            'robot_ip': 'yyy.yyy.yyy.yyy',
            'use_fake_hardware': 'true',
            'launch_rviz': 'true',
            'initial_joint_controller': 'scaled_joint_trajectory_controller'
        }.items()
    )
    
    # Wyświetlanie obrazu kamery
    aruco_camera_node = Node(
        package='ur_control',
        executable='aruco_camera',  # Nasz nowy skrypt
        name='aruco_camera_publisher',
        output='screen'
    )


    # Kontroler robota ArUco
    robot_controller_node = Node(
        package='ur_control',
        executable='robot_controller',
        name='robot_controller',
        output='screen'
    )

    return LaunchDescription([
        ur_control_launch,
        aruco_camera_node,
        robot_controller_node
    ])