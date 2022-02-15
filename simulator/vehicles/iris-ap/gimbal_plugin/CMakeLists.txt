cmake_minimum_required(VERSION 3.5)
project(gimbal_plugin)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wno-inconsistent-missing-override")
endif()

if(WIN32)
  add_compile_definitions(
    # For math constants
    _USE_MATH_DEFINES
    # Minimize Windows namespace collision
    NOMINMAX
    WIN32_LEAN_AND_MEAN
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(trajectory_msgs REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

# Gimbal2dPlugin
add_library(Gimbal2dPlugin SHARED
  src/Gimbal2dPlugin.cpp
)
target_include_directories(Gimbal2dPlugin PUBLIC include)
ament_target_dependencies(Gimbal2dPlugin
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "geometry_msgs"
)
ament_export_libraries(Gimbal2dPlugin)

# ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_msgs)
ament_export_dependencies(gazebo_ros)

if(NOT WIN32)
  if(NOT APPLE)
    set(
      AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_gazebo_plugins
      "prepend-non-duplicate;LD_LIBRARY_PATH;${GAZEBO_PLUGIN_PATH}")
  else()
    set(
      AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_gazebo_plugins
      "prepend-non-duplicate;DYLD_LIBRARY_PATH;${GAZEBO_PLUGIN_PATH}")
  endif()
endif()
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo_plugins.sh.in")

ament_package()

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
    Gimbal2dPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

