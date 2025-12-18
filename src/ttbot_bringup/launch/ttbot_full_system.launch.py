import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # === 1. Khai báo Argument chung ===
    # Mặc định là True (Mô phỏng)
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    # === 2. Lấy đường dẫn ===
    controller_pkg = get_package_share_directory("ttbot_controller")
    description_pkg = get_package_share_directory("ttbot_description")
    localization_pkg = get_package_share_directory("ttbot_localization")

    # === 3. Include Simulation (Gazebo) ===
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg, "launch", "gazebo.launch.py") 
        ),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items()
    )

    # === 4. Include Controller ===
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_pkg, "launch", "controller.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time, 
            # ĐÃ XÓA 2 dòng use_simple_controller và use_python vì file con không dùng
            "wheel_radius": "0.15", # Có thể truyền đè tham số tại đây nếu muốn
            "wheel_base": "0.65",
        }.items(),
    )

    # === 5. Include Localization (EKF) ===
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_pkg, "launch", "local_localization.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time, 
        }.items(),
    )

    # === 6. Trả về ===
    return LaunchDescription([
        use_sim_time_arg,
        simulation_launch,
        localization_launch, 
        controller_launch,
    ])