cmake_minimum_required(VERSION 3.8)
project(qgc_bridge_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 2. Include Directories
# Chỉ cần dùng SYSTEM là đủ, nó bao gồm cả việc include và tắt warning
include_directories(SYSTEM include)

# =========================================================
# TARGET 1: NODE CHÍNH (Chạy file bridge_node.cpp)
# =========================================================
add_executable(qgc_bridge_node src/bridge_node.cpp)
ament_target_dependencies(qgc_bridge_node
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
)

# =========================================================
# TARGET 2: NODE TEST (Chạy file test.cpp cũ)
# =========================================================
add_executable(mavlink_test src/test.cpp)
ament_target_dependencies(mavlink_test
  rclcpp
  # Link đủ thư viện phòng khi code test của bạn cần dùng
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
)

# 3. Install (Cài đặt cả 2 executable)
install(TARGETS
  qgc_bridge_node
  mavlink_test
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()