cmake_minimum_required(VERSION 3.24)
project(kine)

if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()

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
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
# target_planner provides the action interfaces. In a colcon build it is
# installed and findable; in a monolithic CLion/CMake build the rosidl targets
# are already in the same CMake tree and we can reference them directly.
if(TARGET target_planner__rosidl_typesupport_cpp)
    # Monolithic CLion/CMake build: target_planner is in the same CMake tree
    set(_tp_ts_target target_planner__rosidl_typesupport_cpp)
else()
    # Colcon build: find the installed package and use its exported target
    find_package(target_planner REQUIRED)
    set(_tp_ts_target target_planner::target_planner__rosidl_typesupport_cpp)
endif()

include(FetchContent)
set(THREEPP_BUILD_TESTS OFF)
set(THREEPP_BUILD_EXAMPLES OFF)
if (UNIX)
    set(THREEPP_USE_EXTERNAL_GLFW ON)
    message(STATUS "On Unix, use system GLFW. Make sure to install the development package (e.g. libglfw3-dev on Debian/Ubuntu).")
endif ()
FetchContent_Declare(
        threepp
        GIT_REPOSITORY https://github.com/markaren/threepp.git
        GIT_TAG 857ac13
)
FetchContent_MakeAvailable(threepp)

#using imgui from threepp examples
add_subdirectory("${threepp_SOURCE_DIR}/examples/external" "${CMAKE_CURRENT_BINARY_DIR}/threepp_examples_external")


add_executable(kine_environment
        kine_environment.cpp
        src/KineEnvironment.cpp
        src/TrajectoryAnimator.cpp
)
ament_target_dependencies(kine_environment
        rclcpp
        rclcpp_action
        ament_index_cpp
        sensor_msgs
        geometry_msgs
        moveit_ros_planning_interface
)
target_link_libraries(kine_environment threepp::threepp imgui ${_tp_ts_target})

install(TARGETS
        kine_environment
        DESTINATION lib/${PROJECT_NAME})


ament_package()
