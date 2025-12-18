import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # === 1. KHAI BÁO ARGUMENT ===
    # Cho phép bạn gõ lệnh: ros2 launch ... imu_port:=/dev/ttyUSB0
    imu_port_arg = DeclareLaunchArgument(
        'imu_port',
        default_value='/dev/ttbot_imu',
        description='Port for IMU '
    )
    
    imu_port = LaunchConfiguration('imu_port')

    # --- CẤU HÌNH ĐƯỜNG DẪN ---
    imu_pkg_dir = get_package_share_directory('adis16488_driver') 
    localization_pkg_dir = get_package_share_directory('ttbot_localization')

    # === 2. INCLUDE CÁC FILE CON ===
    
    # 2.1. Chạy IMU (Truyền tham số port xuống)
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(imu_pkg_dir, 'launch', 'imu_system.launch.py') 
        ),
        launch_arguments={
            'imu_port': imu_port  # <--- TRUYỀN BIẾN XUỐNG DƯỚI
        }.items()
    )

    # 2.2. Chạy Controller
    ackermann_node = Node(
        package='ttbot_controller',
        executable='ackermann_controller',
        name='ackermann_controller',
        output='screen',
        parameters=[{'use_sim_time': False}] 
    )

    # 2.3. Chạy Localization
    local_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_pkg_dir, 'launch', 'local_localization.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'use_python': 'False'
        }.items()
    )

    delayed_localization = TimerAction(
        period=5.0, 
        actions=[local_localization_launch]
    )

    return LaunchDescription([
        imu_port_arg,      # 1. Đăng ký argument
        imu_launch,        # 2. Chạy IMU (với port tùy chỉnh)
        ackermann_node,
        delayed_localization
    ])