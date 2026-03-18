cmake_minimum_required(VERSION 3.24)
project(target_planner)

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
find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# rosidl_generator_py requires python_d.exe on Windows Debug builds,
# which does not exist in conda/pixi environments. Fall back to release Python.
if(WIN32 AND NOT PYTHON_EXECUTABLE_DEBUG)
    find_package(Python3 QUIET COMPONENTS Interpreter)
    set(PYTHON_EXECUTABLE_DEBUG "${Python3_EXECUTABLE}" CACHE FILEPATH "" FORCE)
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
        "action/Plan.action"
        "action/Execute.action"
        "action/PlanAndExecute.action"
        DEPENDENCIES geometry_msgs
)

add_executable(${PROJECT_NAME}_node target_planner.cpp)
ament_target_dependencies(${PROJECT_NAME}_node
        rclcpp
        rclcpp_action
        geometry_msgs
        moveit_ros_planning_interface
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_node ${cpp_typesupport_target})

install(TARGETS
        ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
