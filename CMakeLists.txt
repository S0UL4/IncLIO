cmake_minimum_required(VERSION 3.16)
project(inclio_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")


# ─────────────────────────────────────────────────────────────────────────────
# ROS2 dependencies
# ─────────────────────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

# Optional Livox support
find_package(livox_ros_driver2 REQUIRED)

# ─────────────────────────────────────────────────────────────────────────────
# IncLIO core — built directly from source inside this package.
#
# We disable tools (needs Pangolin) and tests so colcon only builds the
# two libraries we care about: inclio_common and inclio.
# ─────────────────────────────────────────────────────────────────────────────
set(BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)

option(BUILD_BENCHMARKS "Build NDT / transform benchmarks" OFF)

add_subdirectory(IncLIO)   # → defines targets: inclio_common, inclio

# ─────────────────────────────────────────────────────────────────────────────
# ROS2 wrapper — composable node
# ─────────────────────────────────────────────────────────────────────────────
add_library(inclio_ros2_component SHARED
  src/cloud_convert.cpp
  src/imu_convert.cpp
  src/lio_node.cpp
)

target_include_directories(inclio_ros2_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link directly against the targets defined by add_subdirectory(inclio)
target_link_libraries(inclio_ros2_component
    inclio
    inclio_common
    ${PCL_LIBRARIES}
)

ament_target_dependencies(inclio_ros2_component
  rclcpp
  rclcpp_components
  tf2_ros
  tf2_eigen
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  pcl_conversions
  std_srvs
)

if(livox_ros_driver2_FOUND)
  target_compile_definitions(inclio_ros2_component PUBLIC HAVE_LIVOX_ROS_DRIVER2)
  ament_target_dependencies(inclio_ros2_component livox_ros_driver2)
endif()

# Register as a composable node (for use in component containers)
rclcpp_components_register_nodes(inclio_ros2_component
  "inclio_ros2::LioNode"
)

# Standalone executable with MultiThreadedExecutor
add_executable(inclio_ros2_node src/main.cpp)
target_link_libraries(inclio_ros2_node inclio_ros2_component)
ament_target_dependencies(inclio_ros2_node rclcpp)

# ─────────────────────────────────────────────────────────────────────────────
# Install
# ─────────────────────────────────────────────────────────────────────────────
install(TARGETS inclio_ros2_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS inclio_ros2_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/  DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/  DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_BENCHMARKS)
  add_subdirectory(test)
endif()

ament_package()


