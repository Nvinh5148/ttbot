#ifndef TTBOT_CONTROLLER_STANLEY_CONTROLLER_HPP_
#define TTBOT_CONTROLLER_STANLEY_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/utils.h"
#include <vector>
#include <utility>
#include <algorithm>
#include <cmath>

// Định nghĩa Controller
class StanleyController : public rclcpp::Node
{
public:
    StanleyController();
    ~StanleyController();

private:
    // ==== Callbacks ====
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

    // ==== Logic Stanley ====
    // Tìm điểm gần nhất trên quỹ đạo (tính từ trục trước xe)
    size_t findClosestPoint(double front_x, double front_y);

    // Tính toán góc lái Stanley
    double computeSteering(double front_x, double front_y, double yaw, double v);

    // Load path (nếu cần load từ file CSV như MPC cũ, hiện tại dùng topic)
    void loadPathFromCSV(); 

    // ==== Parameters ====
    double desired_speed_; // m/s
    double wheel_base_;    // L (khoảng cách trục)
    double max_steer_;     // Giới hạn góc lái (rad)
    
    // Stanley gains
    double k_gain_;        // Gain hệ số lỗi ngang (Cross track error gain)
    double k_soft_;        // Hệ số làm mềm (Softening gain)

    // Goal parameters
    double goal_tolerance_;
    bool reached_goal_;

    // ==== Path Data ====
    std::vector<std::pair<double, double>> path_points_;
    size_t current_index_;
    bool has_path_ = false;

    // ==== ROS Interfaces ====
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
};

#endif // TTBOT_CONTROLLER_STANLEY_CONTROLLER_HPP_