cmake_minimum_required(VERSION 3.8)
project(ttbot_gui)

# 1. Setup Cờ biên dịch
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 2. Tìm gói ROS & Qt
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)

# 3. Tự động xử lý file Qt (Moc, Uic, Rcc) -> Rất quan trọng!
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# 4. Tìm tất cả file code trong thư mục src/ (khỏi phải add thủ công)
file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "src/*.h")
file(GLOB_RECURSE UIS "src/*.ui")

# 5. Tạo file chạy
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS} ${UIS})

# 6. Link thư viện
target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets Qt5::Core Qt5::Gui
  ${rclcpp_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

# 7. Install (Để chạy được lệnh ros2 run)
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()