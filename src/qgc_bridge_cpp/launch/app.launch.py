import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. Khai báo tham số launch (để có thể chỉnh từ bên ngoài)
    heading_offset_arg = DeclareLaunchArgument(
        'heading_offset_deg',
        default_value='90.0',
        description='Góc lệch (độ) để chỉnh hướng mũi tên trên QGC (Ví dụ: 90.0 hoặc -90.0)'
    )

    # 2. Định nghĩa Node
    bridge_node = Node(
        package='qgc_bridge_cpp',
        executable='qgc_bridge_node',  # Tên file thực thi (xem trong CMakeLists.txt)
        name='qgc_bridge_node',
        output='screen',
        parameters=[{
            # Lấy giá trị từ tham số launch truyền vào node
            'heading_offset_deg': LaunchConfiguration('heading_offset_deg')
        }]
    )

    return LaunchDescription([
        heading_offset_arg,
        bridge_node
    ])