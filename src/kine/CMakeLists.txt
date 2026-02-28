cmake_minimum_required(VERSION 3.24)
project(kine)

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
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

find_package(assimp REQUIRED)

include(FetchContent)
set(THREEPP_BUILD_TESTS OFF)
set(THREEPP_BUILD_EXAMPLES OFF)
FetchContent_Declare(
        threepp
        GIT_REPOSITORY https://github.com/markaren/threepp.git
        GIT_TAG b2e3404
)
FetchContent_MakeAvailable(threepp)

#using imgui from threepp examples
add_subdirectory(${threepp_SOURCE_DIR}/examples/external ${CMAKE_CURRENT_BINARY_DIR}/threepp_examples_external)


add_executable(kine_environment kine_environment.cpp src/KineEnvironment.cpp src/TrajectoryAnimator.cpp)
target_include_directories(kine_environment PUBLIC include)
ament_target_dependencies(kine_environment rclcpp ament_index_cpp std_msgs sensor_msgs geometry_msgs moveit_ros_planning_interface)
target_link_libraries(kine_environment threepp::threepp imgui assimp::assimp)
target_compile_definitions(kine_environment PRIVATE PROJECT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")


install(TARGETS
        kine_environment
        DESTINATION lib/${PROJECT_NAME})


ament_package()
