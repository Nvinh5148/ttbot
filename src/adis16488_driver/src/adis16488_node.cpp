#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>

// Linux Serial Headers
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

// ROS 2 Headers
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class Adis16488Node : public rclcpp::Node
{
public:
  Adis16488Node()
  : Node("adis16488_driver"), serial_fd_(-1)
  {
    // 1. Khai báo tham số
    this->declare_parameter("port", "/dev/ttyUSB0");
    this->declare_parameter("baudrate", 230400); // Mặc định theo tài liệu [cite: 13]
    this->declare_parameter("frame_id", "imu_link");

    std::string port = this->get_parameter("port").as_string();
    int baudrate = this->get_parameter("baudrate").as_int();
    frame_id_ = this->get_parameter("frame_id").as_string();

    // 2. Mở cổng Serial
    if (open_serial_port(port, baudrate)) {
      RCLCPP_INFO(this->get_logger(), "Connected to IMU on %s at %d", port.c_str(), baudrate);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", port.c_str());
      return; // Dừng nếu không mở được
    }

    // 3. Tạo Publisher
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/out", 10);

    // 4. Tạo Timer để đọc dữ liệu (100Hz = 10ms theo cấu hình mặc định của IMU) [cite: 14]
    // Chạy nhanh hơn một chút (5ms) để đảm bảo không bị miss data
    timer_ = this->create_wall_timer(
      5ms, std::bind(&Adis16488Node::timer_callback, this));
  }

  ~Adis16488Node()
  {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
    }
  }

private:
  int serial_fd_;
  std::string frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string serial_buffer_;

  // Mở và cấu hình Serial Port
  bool open_serial_port(const std::string &port, int baudrate)
  {
    serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ < 0) return false;

    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0) return false;

    // Cấu hình Baudrate
    speed_t speed;
    switch (baudrate) {
      case 115200: speed = B115200; break;
      case 230400: speed = B230400; break;
      case 460800: speed = B460800; break;
      case 921600: speed = B921600; break;
      default: speed = B230400;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // Cấu hình 8N1 (8 data bits, No parity, 1 stop bit)
    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;     // 8 bits
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines

    // Raw mode (đọc byte thô, không xử lý ký tự đặc biệt)
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable flow control
    tty.c_iflag &= ~(ICRNL | INLCR); // Disable CR mapping

    tcsetattr(serial_fd_, TCSANOW, &tty);
    return true;
  }

  void timer_callback()
  {
    if (serial_fd_ < 0) return;

    char buf[256];
    int n = read(serial_fd_, buf, sizeof(buf) - 1);
    
    if (n > 0) {
      buf[n] = '\0';
      serial_buffer_ += buf;
      process_buffer();
    }
  }

  void process_buffer()
  {
    // Tìm ký tự kết thúc 0x0D (CR) 
    size_t pos;
    while ((pos = serial_buffer_.find('\r')) != std::string::npos) {
      std::string line = serial_buffer_.substr(0, pos);
      serial_buffer_.erase(0, pos + 1);

      // Frame bắt đầu bằng 0x0A (LF) [cite: 45]
      // Đôi khi buffer có thể dính ký tự rác ở đầu, cần tìm 0x0A
      size_t start_pos = line.find('\n');
      if (start_pos != std::string::npos) {
        std::string data_str = line.substr(start_pos + 1); // Bỏ qua ký tự 0x0A
        parse_and_publish(data_str);
      }
    }
  }

  void parse_and_publish(const std::string &line)
  {
    std::stringstream ss(line);
    std::string segment;
    std::vector<double> values;

    // Tách chuỗi bằng khoảng trắng (0x20) [cite: 68]
    while (std::getline(ss, segment, ' ')) {
      if (!segment.empty()) {
        try {
          values.push_back(std::stod(segment));
        } catch (...) {
          // Bỏ qua lỗi parse
        }
      }
    }

    // Kiểm tra đủ 9 giá trị cơ bản (Roll, Pitch, Yaw, Wx, Wy, Wz, Ax, Ay, Az)
    // Dựa vào bảng cấu hình gửi dữ liệu mặc định [cite: 67]
    if (values.size() >= 9) {
      auto msg = sensor_msgs::msg::Imu();
      msg.header.stamp = this->now();
      msg.header.frame_id = frame_id_;

      // --- CHUYỂN ĐỔI ĐƠN VỊ ---
      // Constants
      const double DEG_TO_RAD = M_PI / 180.0;
      const double G_TO_MS2 = 9.80665;

      // 1. Angles: Roll, Pitch, Yaw (Unit: 0.001 deg) [cite: 46]
      double roll  = values[0] * 0.001 * DEG_TO_RAD;
      double pitch = values[1] * 0.001 * DEG_TO_RAD;
      double yaw   = values[2] * 0.001 * DEG_TO_RAD;

      // Chuyển Euler -> Quaternion
      tf2::Quaternion q;
      q.setRPY(roll, pitch, yaw);
      msg.orientation = tf2::toMsg(q);

      // 2. Angular Velocity: Wx, Wy, Wz (Unit: 0.001 deg/s) [cite: 48]
      msg.angular_velocity.x = values[3] * 0.001 * DEG_TO_RAD;
      msg.angular_velocity.y = values[4] * 0.001 * DEG_TO_RAD;
      msg.angular_velocity.z = values[5] * 0.001 * DEG_TO_RAD;

      // 3. Acceleration: Ax, Ay, Az (Unit: 0.1 mg) [cite: 49]
      // 1 unit = 0.1 mg = 0.0001 g
      msg.linear_acceleration.x = values[6] * 0.0001 * G_TO_MS2;
      msg.linear_acceleration.y = values[7] * 0.0001 * G_TO_MS2;
      msg.linear_acceleration.z = values[8] * 0.0001 * G_TO_MS2;

      // Gán Covariance (Ước lượng, cần thiết cho EKF)
      // Giá trị 0.01 là ví dụ, bạn nên đo nhiễu thực tế khi IMU đứng yên
      for (int i = 0; i < 9; i++) {
        msg.orientation_covariance[i] = 0.0;
        msg.angular_velocity_covariance[i] = 0.0;
        msg.linear_acceleration_covariance[i] = 0.0;
      }
      msg.orientation_covariance[0] = 0.01;
      msg.orientation_covariance[4] = 0.01;
      msg.orientation_covariance[8] = 0.01;

      // Publish
      imu_pub_->publish(msg);
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Adis16488Node>());
  rclcpp::shutdown();
  return 0;
}