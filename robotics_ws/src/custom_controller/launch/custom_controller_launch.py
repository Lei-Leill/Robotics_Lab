import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.0.1',
        description='IP address of the robot'
    )
    arm_id_arg = DeclareLaunchArgument(
        'arm_id',
        default_value='fr3',
        description='Name of the robot, e.g., fr3'
    )

    robot_ip = LaunchConfiguration('robot_ip')
    arm_id = LaunchConfiguration('arm_id')
    
    pkg_share = get_package_share_directory('custom_controller')
    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')
    
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("franka_description"), "robots", "fr3", "fr3.urdf.xacro"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            xacro_file, " ",
            "robot_ip:=", robot_ip, " ",
            "arm_id:=", arm_id, " ",
            "use_gripper:=true"
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    ros2_control_node = Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=[controllers_yaml],
    output='screen'
    )

    spawner_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    spawner_franka_robot_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['franka_robot_state_broadcaster'],
        output='screen'
    )
    spawner_franka_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['franka_gripper_controller'],
        output='screen'
    )
    custom_controller_node = Node(
        package='custom_controller',
        executable='custom_controller',
        name='custom_controller',
        output='screen'
    )

    return LaunchDescription([
        robot_ip_arg,
        arm_id_arg,
        robot_state_publisher_node,
        ros2_control_node,
        TimerAction(period=3.0, actions=[spawner_joint_state_broadcaster]),
        TimerAction(period=4.0, actions=[spawner_franka_robot_state_broadcaster]),
        TimerAction(period=5.0, actions=[spawner_franka_gripper_controller]),
        TimerAction(period=7.0, actions=[custom_controller_node]), 
    ])
