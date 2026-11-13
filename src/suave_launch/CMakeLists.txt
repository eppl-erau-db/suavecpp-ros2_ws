cmake_minimum_required(VERSION 3.8)
project(suave_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(MAVSDK REQUIRED)

add_executable(suave_main
  src/suave_main.cpp
  src/common/SystemTask.cpp
  src/controllers/SuaveFlightController.cpp
  src/mavutil/Drone.cpp
  src/mavutil/MavUtil.cpp
  src/mavutil/TelemetryProperty.cpp
  src/ros/RosNodeSpinner.cpp
  src/vio/VIOBridge.cpp
)
target_compile_features(suave_main PUBLIC cxx_std_17)
ament_target_dependencies(suave_main rclcpp nav_msgs ament_index_cpp)
target_link_libraries(suave_main MAVSDK::mavsdk)

install(TARGETS suave_main DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
