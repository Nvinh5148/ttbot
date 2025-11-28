#include "ttbot_controller/mpc_controller.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <iostream>

// Helper logs
static double deg2rad(double deg) { return deg * M_PI / 180.0; }
static double rad2deg(double rad) { return rad * 180.0 / M_PI; }

MpcController::MpcController()
: Node("mpc_controller")
{
    // 1. Declare & Load Parameters
    this->declare_parameter("desired_speed", 1.5);
    this->declare_parameter("wheel_base", 0.8);
    this->declare_parameter("max_steer_deg", 30.0);
    this->declare_parameter("goal_tolerance", 0.3);

    // MPC Weights
    this->declare_parameter("N_p", 10);
    this->declare_parameter("dt_mpc", 0.1);
    this->declare_parameter("Q_ey", 10.0);
    this->declare_parameter("Q_epsi", 5.0);
    this->declare_parameter("R_delta", 1.0);

    desired_speed_ = this->get_parameter("desired_speed").as_double();
    wheel_base_    = this->get_parameter("wheel_base").as_double();
    double max_steer_deg = this->get_parameter("max_steer_deg").as_double();
    
    N_p_    = this->get_parameter("N_p").as_int();
    dt_mpc_ = this->get_parameter("dt_mpc").as_double();
    Q_ey_   = this->get_parameter("Q_ey").as_double();
    Q_epsi_ = this->get_parameter("Q_epsi").as_double();
    R_delta_= this->get_parameter("R_delta").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();

    // Constraints
    max_steer_ = deg2rad(max_steer_deg);
    // Tính max omega dựa trên mô hình Ackermann: w = (v/L) * tan(delta)
    max_omega_ = (std::abs(desired_speed_) / wheel_base_) * std::tan(max_steer_);

    // Init State
    current_index_ = 0;
    has_path_ = false;
    reached_goal_ = false;

    // 2. ROS setup
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&MpcController::odomCallback, this, std::placeholders::_1));

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/mpc_path", 1,
        std::bind(&MpcController::pathCallback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/ackermann_controller/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), 
        "MPC Initialized: N=%d, dt=%.2f, Speed=%.2f, MaxSteer=%.1f deg", 
        N_p_, dt_mpc_, desired_speed_, max_steer_deg);
}

MpcController::~MpcController()
{
    freeOSQPMemory();
}

void MpcController::freeOSQPMemory()
{
    // Clean up Solver
    if (solver_) { osqp_cleanup(solver_); solver_ = nullptr; }
    if (settings_) { free(settings_); settings_ = nullptr; }

    // Clean up Data Arrays
    if (P_x_) { free(P_x_); P_x_ = nullptr; }
    if (P_i_) { free(P_i_); P_i_ = nullptr; }
    if (P_p_) { free(P_p_); P_p_ = nullptr; }

    if (A_x_) { free(A_x_); A_x_ = nullptr; }
    if (A_i_) { free(A_i_); A_i_ = nullptr; }
    if (A_p_) { free(A_p_); A_p_ = nullptr; }

    if (q_data_) { free(q_data_); q_data_ = nullptr; }
    if (l_data_) { free(l_data_); l_data_ = nullptr; }
    if (u_data_) { free(u_data_); u_data_ = nullptr; }
}

void MpcController::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    if (msg->poses.empty()) return;

    path_points_.clear();
    path_points_.reserve(msg->poses.size());

    for (const auto &pose : msg->poses) {
        path_points_.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }

    current_index_ = 0;
    has_path_ = true;
    reached_goal_ = false; // Reset trạng thái khi có path mới

    RCLCPP_INFO(this->get_logger(), "--> NEW PATH: %zu points. MPC Ready.", path_points_.size());
}

size_t MpcController::findClosestPoint(double x, double y)
{
    if (path_points_.empty()) return 0;

    size_t best_idx = current_index_;
    double min_dist_sq = std::numeric_limits<double>::infinity();

    // Tối ưu: Chỉ tìm kiếm trong cửa sổ +100 điểm phía trước
    size_t search_end = std::min(current_index_ + 100, path_points_.size());
    if (current_index_ == 0) search_end = path_points_.size(); // Tìm hết nếu mới bắt đầu

    for (size_t i = current_index_; i < search_end; ++i) {
        double dx = x - path_points_[i].first;
        double dy = y - path_points_[i].second;
        double dist_sq = dx*dx + dy*dy;
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            best_idx = i;
        }
    }
    current_index_ = best_idx;
    return best_idx;
}

void MpcController::computeReference(size_t idx, double &rx, double &ry, double &rpsi)
{
    rx = path_points_[idx].first;
    ry = path_points_[idx].second;

    // Tính psi_ref từ điểm tiếp theo
    if (idx + 1 < path_points_.size()) {
        double dx = path_points_[idx+1].first - rx;
        double dy = path_points_[idx+1].second - ry;
        rpsi = std::atan2(dy, dx);
    } else if (idx > 0) {
        // Điểm cuối, dùng hướng cũ
        double dx = rx - path_points_[idx-1].first;
        double dy = ry - path_points_[idx-1].second;
        rpsi = std::atan2(dy, dx);
    } else {
        rpsi = 0.0;
    }
}

void MpcController::computeErrorState(double x, double y, double yaw,
                                      double rx, double ry, double rpsi,
                                      double &ey, double &epsi)
{
    // Vector từ Ref -> Robot (Khác với Stanley, MPC thường dùng trạng thái lỗi kiểu này)
    // Để khớp với công thức Stanley cho dễ hiểu: Ta tính lỗi so với đường cơ sở Frenet
    double dx = x - rx;
    double dy = y - ry;

    // Lỗi ngang e_y: Chiếu vector lỗi lên trục vuông góc với path
    // Trục Path: (cos, sin) -> Trục Vuông góc (trái): (-sin, cos)
    ey = -std::sin(rpsi) * dx + std::cos(rpsi) * dy;

    // Lỗi góc e_psi
    epsi = yaw - rpsi;
    while (epsi > M_PI) epsi -= 2.0*M_PI;
    while (epsi < -M_PI) epsi += 2.0*M_PI;
}

void MpcController::linearizeErrorModel(double v, double dt)
{
    // Mô hình lỗi động học (Kinematic Error Model):
    // e_y(k+1)   = e_y(k) + v * sin(e_psi(k)) * dt  ~ e_y(k) + v * e_psi(k) * dt
    // e_psi(k+1) = e_psi(k) + (v/L * tan(delta)) * dt ~ e_psi(k) + v/L * delta * dt

    // State: [ey, epsi]
    // Input: [delta]
    
    // Matrix A (2x2)
    // 1   v*dt
    // 0   1
    Eigen::Matrix2d A;
    A << 1.0, v * dt,
         0.0, 1.0;

    // Vector B (2x1)
    // 0
    // v*dt/L
    Eigen::Vector2d B;
    B << 0.0,
         (v * dt) / wheel_base_;

    Ad_ = A;
    Bd_ = B;
}

void MpcController::eigenToOSQPCsc(const Eigen::SparseMatrix<double>& mat,
                                   OSQPCscMatrix& out_mat,
                                   OSQPFloat*& out_x, OSQPInt*& out_i, OSQPInt*& out_p)
{
    // Phải dùng malloc để tương thích với OSQP free
    OSQPInt nnz = mat.nonZeros();
    out_x = (OSQPFloat*)malloc(sizeof(OSQPFloat) * nnz);
    out_i = (OSQPInt*)malloc(sizeof(OSQPInt) * nnz);
    out_p = (OSQPInt*)malloc(sizeof(OSQPInt) * (mat.cols() + 1));

    for (int k = 0; k < nnz; k++) {
        out_x[k] = (OSQPFloat)mat.valuePtr()[k];
        out_i[k] = (OSQPInt)mat.innerIndexPtr()[k];
    }
    for (int k = 0; k < mat.cols() + 1; k++) {
        out_p[k] = (OSQPInt)mat.outerIndexPtr()[k];
    }

    out_mat.m = mat.rows();
    out_mat.n = mat.cols();
    out_mat.nzmax = nnz;
    out_mat.nz = -1;
    out_mat.owned = 0; // Chúng ta quản lý bộ nhớ
    out_mat.x = out_x;
    out_mat.i = out_i;
    out_mat.p = out_p;
}

Control MpcController::solveMPC(double ey0, double epsi0, double v_ref)
{
    // 1. Dọn dẹp bộ nhớ cũ
    freeOSQPMemory();

    // 2. Thiết lập bài toán
    int nx = 2; // [ey, epsi]
    int nu = 1; // [delta]
    int N  = N_p_;
    
    // Biến: x0, u0, x1, u1 ... xN (Không có uN) -> Tổng biến = (N+1)*nx + N*nu
    int n_vars = (N + 1) * nx + N * nu;
    
    // Ràng buộc (Constraints):
    // 1. Động lực học: x(k+1) = A*x(k) + B*u(k) -> (N)*nx phương trình
    // 2. Trạng thái đầu: x(0) = [ey0, epsi0] -> nx phương trình
    // 3. Giới hạn input: -max < u < max -> N*nu bất đẳng thức
    int n_eq   = (N + 1) * nx; 
    int n_ineq = N * nu;
    int n_cons = n_eq + n_ineq;

    // Cập nhật ma trận động lực học Ad, Bd
    linearizeErrorModel(v_ref, dt_mpc_);

    // --- Xây dựng ma trận P (Cost) ---
    Eigen::SparseMatrix<double> P(n_vars, n_vars);
    std::vector<Eigen::Triplet<double>> p_triplets;
    
    // Cost cho States
    for (int k = 0; k <= N; ++k) {
        int offset = k * nx;
        p_triplets.emplace_back(offset,     offset,     Q_ey_);
        p_triplets.emplace_back(offset + 1, offset + 1, Q_epsi_);
    }
    // Cost cho Inputs (Bắt đầu sau các biến state)
    int u_start_idx = (N + 1) * nx;
    for (int k = 0; k < N; ++k) {
        int offset = u_start_idx + k * nu;
        p_triplets.emplace_back(offset, offset, R_delta_);
    }
    P.setFromTriplets(p_triplets.begin(), p_triplets.end());

    // --- Xây dựng ma trận A_constraints ---
    // Ax = [Eq_Dynamics; Eq_Init; Ineq_Input]
    Eigen::SparseMatrix<double> A_cons(n_cons, n_vars);
    std::vector<Eigen::Triplet<double>> a_triplets;
    
    // Bounds l, u
    Eigen::VectorXd l = Eigen::VectorXd::Zero(n_cons);
    Eigen::VectorXd u = Eigen::VectorXd::Zero(n_cons);

    // 1. Ràng buộc init: x0 = current_state
    // Row 0..1
    for (int i = 0; i < nx; ++i) {
        a_triplets.emplace_back(i, i, 1.0);
        l(i) = (i == 0) ? ey0 : epsi0;
        u(i) = (i == 0) ? ey0 : epsi0;
    }

    // 2. Ràng buộc động lực học: -Ad*x(k) + I*x(k+1) - Bd*u(k) = 0
    // Bắt đầu từ row = nx (tức là row 2)
    for (int k = 0; k < N; ++k) {
        int row = nx + k * nx;
        int xk  = k * nx;
        int xk1 = (k + 1) * nx;
        int uk  = u_start_idx + k * nu;

        // -Ad * x(k)
        for (int r = 0; r < nx; ++r) {
            for (int c = 0; c < nx; ++c) {
                if (std::abs(Ad_(r,c)) > 1e-5)
                    a_triplets.emplace_back(row + r, xk + c, -Ad_(r,c));
            }
        }
        // +I * x(k+1)
        for (int r = 0; r < nx; ++r) {
            a_triplets.emplace_back(row + r, xk1 + r, 1.0);
        }
        // -Bd * u(k)
        for (int r = 0; r < nx; ++r) {
            if (std::abs(Bd_(r)) > 1e-5)
                a_triplets.emplace_back(row + r, uk, -Bd_(r));
        }

        // Bounds = 0
        l(row) = 0.0; u(row) = 0.0;
        l(row+1) = 0.0; u(row+1) = 0.0;
    }

    // 3. Ràng buộc Input: -max < u < max
    int ineq_start = n_eq;
    for (int k = 0; k < N; ++k) {
        int row = ineq_start + k;
        int col = u_start_idx + k * nu;
        a_triplets.emplace_back(row, col, 1.0);
        l(row) = -max_steer_;
        u(row) =  max_steer_;
    }

    A_cons.setFromTriplets(a_triplets.begin(), a_triplets.end());

    // --- Convert to OSQP & Solve ---
    OSQPCscMatrix P_mat, A_mat;
    eigenToOSQPCsc(P, P_mat, P_x_, P_i_, P_p_);
    eigenToOSQPCsc(A_cons, A_mat, A_x_, A_i_, A_p_);

    // Setup q (linear cost = 0)
    q_data_ = (OSQPFloat*)calloc(n_vars, sizeof(OSQPFloat));
    
    // Setup l, u arrays
    l_data_ = (OSQPFloat*)malloc(n_cons * sizeof(OSQPFloat));
    u_data_ = (OSQPFloat*)malloc(n_cons * sizeof(OSQPFloat));
    for(int i=0; i<n_cons; ++i){
        l_data_[i] = (OSQPFloat)l(i);
        u_data_[i] = (OSQPFloat)u(i);
    }

    // Settings
    settings_ = (OSQPSettings*)malloc(sizeof(OSQPSettings));
    osqp_set_default_settings(settings_);
    settings_->verbose = 0; // Tắt log solver cho đỡ rối

    osqp_setup(&solver_, &P_mat, q_data_, &A_mat, l_data_, u_data_, n_cons, n_vars, settings_);
    
    osqp_solve(solver_);

    double delta_opt = 0.0;
    if (solver_->info->status_val == OSQP_SOLVED) {
        // Lấy u0 (phần tử đầu tiên của khối input)
        delta_opt = solver_->solution->x[u_start_idx];
    } else {
        RCLCPP_WARN(get_logger(), "MPC Unsolved: %s", solver_->info->status);
    }

    return {delta_opt};
}

void MpcController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // ==========================================
    // 1. FIREWALL: Dừng xe tuyệt đối nếu đã tới đích
    // ==========================================
    if (reached_goal_) {
        geometry_msgs::msg::TwistStamped stop_cmd;
        stop_cmd.header.stamp = this->now();
        stop_cmd.header.frame_id = "base_link";
        stop_cmd.twist.linear.x = 0.0;
        stop_cmd.twist.angular.z = 0.0;
        cmd_pub_->publish(stop_cmd);
        return;
    }

    if (!has_path_ || path_points_.empty()) return;

    // Lấy state robot
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double yaw = tf2::getYaw(msg->pose.pose.orientation);
    double v_ref = desired_speed_;

    // ==========================================
    // 2. CHECK GOAL (Để bật cờ reached_goal_)
    // ==========================================
    double dx_g = x - path_points_.back().first;
    double dy_g = y - path_points_.back().second;
    double dist_to_goal = std::sqrt(dx_g*dx_g + dy_g*dy_g);

    bool near_end = current_index_ > (path_points_.size() * 0.9);
    if (dist_to_goal < goal_tolerance_ && near_end) {
        reached_goal_ = true;
        RCLCPP_WARN(this->get_logger(), "!!! MPC GOAL REACHED (%.2fm) - STOPPING !!!", dist_to_goal);
        return; 
    }

    // ==========================================
    // 3. MPC CALCULATION
    // ==========================================
    // Tìm ref
    size_t idx = findClosestPoint(x, y);
    double rx, ry, rpsi;
    computeReference(idx, rx, ry, rpsi);

    // Tính lỗi
    double ey, epsi;
    computeErrorState(x, y, yaw, rx, ry, rpsi, ey, epsi);

    // Giải MPC
    Control u = solveMPC(ey, epsi, v_ref);
    double delta = u[0];

    // Chuyển delta -> omega
    double omega = (v_ref / wheel_base_) * std::tan(delta);
    omega = std::clamp(omega, -max_omega_, max_omega_);

    // Log chi tiết (Throttled 500ms)
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "MPC LOG | idx: %zu | ey: %.3f | epsi: %.3f | Delta: %.1f deg",
        idx, ey, epsi, rad2deg(delta));

    // ==========================================
    // 4. PUBLISH
    // ==========================================
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";
    cmd.twist.linear.x = v_ref;
    cmd.twist.angular.z = omega;
    cmd_pub_->publish(cmd);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MpcController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}