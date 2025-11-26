from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():

    # Gom lại cho gọn, dùng chung cho tất cả node
    # QUAN TRỌNG: Đây là chìa khóa sửa lỗi TF_OLD_DATA
    common_params = [{"use_sim_time": True}]

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        # Đã sửa lại quaternion về mặc định (nằm phẳng). 
        # Nếu bạn thực sự cần lật úp thì đổi lại như cũ nhé.
        arguments=["--x", "0", "--y", "0","--z", "0.103",
                   "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                   "--frame-id", "base_link",
                   "--child-frame-id", "imu_link"],
        parameters=common_params # Nên thêm cả vào đây cho đồng bộ
    )

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(get_package_share_directory("ttbot_localization"), "config", "ekf.yaml"),
            common_params[0] # Merge dict use_sim_time vào
        ],
        # Remap nếu cần thiết (ví dụ nếu imu_republisher ra topic khác)
        # remappings=[('/odometry/filtered', '/odom')] 
    )

    imu_republisher_cpp = Node(
        package="ttbot_localization",
        executable="imu_republisher",
        parameters=common_params
    )

    return LaunchDescription([
        static_transform_publisher,
        robot_localization,
        imu_republisher_cpp,   
    ])