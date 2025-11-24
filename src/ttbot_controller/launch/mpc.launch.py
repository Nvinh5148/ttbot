import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # =========================================
    # 1. KHAI BÁO THAM SỐ LAUNCH
    # =========================================

    # --- PATH FILE ---
    path_file_arg = DeclareLaunchArgument(
        "path_file",

        default_value="path_l.csv",
        description="Tên file CSV trong share/ttbot_controller/path/"

    )

    # --- THÔNG SỐ XE ---
    desired_speed_arg = DeclareLaunchArgument(
        "desired_speed",
        default_value="1.5",
        description="Vận tốc xe (m/s)"
    )

    wheel_base_arg = DeclareLaunchArgument(
        "wheel_base",
        default_value="0.8",
        description="Chiều dài trục bánh xe"
    )

    max_steer_deg_arg = DeclareLaunchArgument(
        "max_steer_deg",
        default_value="60.0",
        description="Giới hạn góc lái (độ)"
    )

    # --- THAM SỐ MPC ---
    Np_arg = DeclareLaunchArgument(
        "N_p",
        default_value="10",
        description="Prediction Horizon"
    )

    dt_mpc_arg = DeclareLaunchArgument(
        "dt_mpc",
        default_value="0.1",
        description="Bước thời gian MPC (s)"
    )

    # Cost function weights (đúng theo ey, epsi, delta)
    Q_ey_arg = DeclareLaunchArgument(
        "Q_ey",
        default_value="10.0",
        description="Trọng số lỗi ngang e_y"
    )

    Q_epsi_arg = DeclareLaunchArgument(
        "Q_epsi",
        default_value="5.0",
        description="Trọng số lỗi góc e_psi"
    )

    R_delta_arg = DeclareLaunchArgument(
        "R_delta",
        default_value="1.0",
        description="Trọng số độ lớn góc lái delta"
    )


    # =========================================
    # 2. CẤU HÌNH NODE MPC
    # =========================================
    mpc_node = Node(
        package="ttbot_controller",
        executable="mpc_controller",
        name="mpc_controller",
        output="screen",
        parameters=[{
            "path_file":      LaunchConfiguration("path_file"),
            "desired_speed":  LaunchConfiguration("desired_speed"),
            "wheel_base":     LaunchConfiguration("wheel_base"),
            "max_steer_deg":  LaunchConfiguration("max_steer_deg"),

            "N_p":            LaunchConfiguration("N_p"),
            "dt_mpc":         LaunchConfiguration("dt_mpc"),
            "Q_ey":           LaunchConfiguration("Q_ey"),
            "Q_epsi":         LaunchConfiguration("Q_epsi"),
            "R_delta":        LaunchConfiguration("R_delta"),
        }]
    )

    # =========================================
    # 3. RETURN LAUNCH DESCRIPTION
    # =========================================
    return LaunchDescription([
        path_file_arg,
        desired_speed_arg,
        wheel_base_arg,
        max_steer_deg_arg,

        Np_arg,
        dt_mpc_arg,
        Q_ey_arg,
        Q_epsi_arg,
        R_delta_arg,

        mpc_node
    ])
