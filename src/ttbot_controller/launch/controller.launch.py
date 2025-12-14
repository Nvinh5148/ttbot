import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.15"
    )

    wheel_base_arg = DeclareLaunchArgument(
        "wheel_base",
        default_value="0.65"
    )

    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_base   = LaunchConfiguration("wheel_base")

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    front_steering_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "front_steering_position_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    rear_wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rear_wheel_velocity_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    ackermann_controller_node = Node(
        package="ttbot_controller",          
        executable="ackermann_controller",   
        parameters=[
            {"wheel_radius": wheel_radius},
            {"wheel_base": wheel_base},
            {"use_sim_time": use_sim_time}  
        ],
        output="screen",
    )

    return LaunchDescription([
        use_sim_time_arg, 
        wheel_radius_arg,
        wheel_base_arg,

        joint_state_broadcaster_spawner,
        front_steering_controller_spawner,
        rear_wheel_controller_spawner,
        ackermann_controller_node,
    ])