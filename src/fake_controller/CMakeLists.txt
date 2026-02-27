cmake_minimum_required(VERSION 3.24)
project(fake_controller)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


find_package(rclcpp_action REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)


add_executable(fake_controller_node fake_controller.cpp)
ament_target_dependencies(fake_controller_node rclcpp rclcpp_action std_msgs sensor_msgs trajectory_msgs control_msgs)


install(TARGETS
        fake_controller_node
        DESTINATION lib/${PROJECT_NAME})


ament_package()

