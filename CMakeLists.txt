cmake_minimum_required(VERSION 3.16)
project(ros2_ur_ws LANGUAGES C CXX)


# This file exists ONLY so IDEs (CLion) have a single entry point.
# colcon still drives the actual build.

if (WIN32)
    list(APPEND CMAKE_PREFIX_PATH
            "C:/robostack/.pixi/envs/default/Library"
            "C:/robostack/.pixi/envs/jazzy/Library"
    )
endif ()

find_package(ament_cmake REQUIRED)

# Add each package as a subdirectory
add_subdirectory(src/fake_controller)
add_subdirectory(src/target_planner)
add_subdirectory(src/kine)


# ---- colcon build target ----
add_custom_target(colcon_build
        COMMAND colcon build --base-paths src --merge-install --event-handlers console_direct+
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        USES_TERMINAL
        COMMENT "Running colcon build --merge-install (Windows)"
)
