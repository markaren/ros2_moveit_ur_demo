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

# Add each cpp package as a subdirectory
add_subdirectory(src/fake_controller)
add_subdirectory(src/target_planner)
add_subdirectory(src/kine)


# ---- colcon build targets ----
if (WIN32)
    set(COLCON_INSTALL_FLAG --merge-install)
else ()
    set(COLCON_INSTALL_FLAG --symlink-install)
endif ()

file(GLOB _pkg_xmls RELATIVE "${CMAKE_SOURCE_DIR}/src" "src/*/package.xml")
foreach (_xml IN LISTS _pkg_xmls)
    get_filename_component(_pkg "${_xml}" DIRECTORY)
    list(APPEND PACKAGES "${_pkg}")
endforeach ()

foreach (pkg IN LISTS PACKAGES)
    add_custom_target(colcon_build_${pkg}
            COMMAND colcon build --base-paths src --packages-select ${pkg} ${COLCON_INSTALL_FLAG} --event-handlers console_direct+
            WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
            USES_TERMINAL
            COMMENT "Running colcon build ${COLCON_INSTALL_FLAG} for package ${pkg}"
    )
endforeach ()

add_custom_target(colcon_build_all
        COMMAND colcon build --base-paths src ${COLCON_INSTALL_FLAG} --event-handlers console_direct+
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        USES_TERMINAL
        COMMENT "Running colcon build ${COLCON_INSTALL_FLAG}"
)
