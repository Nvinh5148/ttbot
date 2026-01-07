#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>
#include <vector>

class PathPublisher : public rclcpp::Node
{
public:
    PathPublisher() : Node("path_publisher_node")
    {
        rclcpp::QoS qos_profile(1);
        qos_profile.transient_local();
        qos_profile.reliable();

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/mpc_path", qos_profile);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PathPublisher::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "--> Path Publisher: SQUARE 15x15m");
    }

private:
    void timerCallback()
    {
        nav_msgs::msg::Path path_msg = generateSquarePath();
        path_pub_->publish(path_msg);
    }

    // Hàm tạo 1 điểm Pose nhanh gọn
    void addPointToPath(nav_msgs::msg::Path &path, double x, double y) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0; // Mặc định
        path.poses.push_back(pose);
    }

    nav_msgs::msg::Path generateSquarePath()
    {
        nav_msgs::msg::Path path;
        path.header.stamp = this->now();
        path.header.frame_id = "odom"; 

        double side_len = 15.0; // Cạnh hình vuông 15m
        double step = 0.05;     // Mật độ điểm: 5cm một điểm (càng nhỏ path càng mịn)

        // --- CẠNH 1: Đi từ (0,0) -> (15,0) ---
        for (double x = 0; x <= side_len; x += step) {
            addPointToPath(path, x, 0.0);
        }

        // --- CẠNH 2: Đi từ (15,0) -> (15,15) ---
        for (double y = 0; y <= side_len; y += step) {
            addPointToPath(path, side_len, y);
        }

        // --- CẠNH 3: Đi từ (15,15) -> (0,15) ---
        for (double x = side_len; x >= 0; x -= step) {
            addPointToPath(path, x, side_len);
        }

        // --- CẠNH 4: Đi từ (0,15) -> (0,0) ---
        for (double y = side_len; y >= 0; y -= step) {
            addPointToPath(path, 0.0, y);
        }

        return path;
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}