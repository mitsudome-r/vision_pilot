cmake_minimum_required(VERSION 3.22.1)

find_package(rclcpp REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)

add_library(autoware_vehicle_ros2_interface
        src/autoware_vehicle_ros2_interface.cpp
)

target_include_directories(autoware_vehicle_ros2_interface
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(autoware_vehicle_ros2_interface
        PUBLIC
        vehicle_interface
        rclcpp::rclcpp
        ${autoware_control_msgs_TARGETS}
        ${autoware_vehicle_msgs_TARGETS}
)

target_compile_features(autoware_vehicle_ros2_interface PUBLIC cxx_std_17)
