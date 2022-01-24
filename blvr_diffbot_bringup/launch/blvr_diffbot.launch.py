import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def generate_launch_description():
    blvr_diffbot_description_path = os.path.join(
        get_package_share_directory('blvr_diffbot_description'))

    xacro_file = os.path.join(blvr_diffbot_description_path,
                              'robots',
                              'blvr_diffbot.urdf.xacro')
    # xacroをロード
    doc = xacro.process_file(xacro_file)
    # xacroを展開してURDFを生成
    robot_desc = doc.toprettyxml(indent='  ')

    params = {'robot_description': robot_desc}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("blvr_diffbot_bringup"),
            "config",
            "blvr_diffbot.yaml",
        ]
    )
    
    joy_config = PathJoinSubstitution(
        [
            FindPackageShare("blvr_diffbot_bringup"),
            "config",
            "joy_config.yaml",
        ]
    )

    front_urg_config = PathJoinSubstitution(
        [
            FindPackageShare("blvr_diffbot_bringup"),
            "config",
            "front_urg_config.yaml",
        ]
    )

    rear_urg_config = PathJoinSubstitution(
        [
            FindPackageShare("blvr_diffbot_bringup"),
            "config",
            "rear_urg_config.yaml",
        ]
    )
    
    joy = Node(
            package='joy',
            name='joy',
            executable='joy_node',
            output='screen',
    )

    teleop_joy_node = Node(
            package='teleop_twist_joy',
            name='teleop_twist_joy_node',
            executable='teleop_node',
            output='screen',
            parameters=[
                joy_config
            ],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[params, robot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'blvr_diffbot'],
                        output='screen')

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    velocity_converter = Node(
        package='velocity_pub',
        name='velocity_pub',
        executable='velocity_pub',
    )
    
    front_urg_node = Node(
        package='urg_node',
        name='front_urg_node',
        executable='urg_node_driver',
        remappings=[
            ('scan', '/front_lrf/laserscan'),
        ],
        parameters=[front_urg_config],
    )

    rear_urg_node = Node(
        package='urg_node',
        name='rear_urg_node',
        executable='urg_node_driver',
        remappings=[
            ('scan', '/rear_lrf/laserscan'),
        ],
        parameters=[rear_urg_config],
    )

    stop_daemon_node = Node(
        package='stop_daemon',
        executable='stop_daemon',
        remappings=[
            ('cmd_vel', '/cmd_vel_stamped'),
            ("safety_cmd_vel", '/diff_drive_controller/cmd_vel'),
            ("scan", '/front_lrf/laserscan'),
        ],
    )
        
    return LaunchDescription([
        joy,
        teleop_joy_node,
        control_node,
        node_robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        velocity_converter,
        front_urg_node,
        rear_urg_node,
        stop_daemon_node,
    ])
