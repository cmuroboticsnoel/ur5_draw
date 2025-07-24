cmake_minimum_required(VERSION 3.8)
project(ur5_drawing)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Check for JSON library (both options)
find_package(PkgConfig)
pkg_check_modules(JSONCPP jsoncpp)

if(JSONCPP_FOUND)
  message(STATUS "Found jsoncpp library")
  set(JSON_LIBRARIES ${JSONCPP_LIBRARIES})
  set(JSON_INCLUDE_DIRS ${JSONCPP_INCLUDE_DIRS})
  add_definitions(-DUSE_JSONCPP)
else()
  find_path(NLOHMANN_JSON_INCLUDE_DIR 
    NAMES nlohmann/json.hpp
    PATHS /usr/include /usr/local/include /opt/ros/${ROS_DISTRO}/include
  )
  if(NLOHMANN_JSON_INCLUDE_DIR)
    message(STATUS "Found nlohmann/json library")
    set(JSON_INCLUDE_DIRS ${NLOHMANN_JSON_INCLUDE_DIR})
    set(JSON_LIBRARIES "")
    add_definitions(-DUSE_NLOHMANN_JSON)
  else()
    message(FATAL_ERROR "Neither jsoncpp nor nlohmann/json found. Please install one:
    - Ubuntu: sudo apt install libjsoncpp-dev OR sudo apt install nlohmann-json3-dev")
  endif()
endif()

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

#==============================================================================
# LIBRARIES
#==============================================================================

# Main drawing utilities library (now joint-based)
add_library(${PROJECT_NAME}_utils SHARED
  src/drawing_utils.cpp
  src/config_loader.cpp
)

target_include_directories(${PROJECT_NAME}_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${JSON_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}_utils
  rclcpp
  rclcpp_action
  geometry_msgs
  trajectory_msgs
  control_msgs
  moveit_core
  moveit_ros_planning_interface
  tf2
  tf2_geometry_msgs
  yaml-cpp
)

target_link_libraries(${PROJECT_NAME}_utils
  ${JSON_LIBRARIES}
  yaml-cpp
)

#==============================================================================
# EXECUTABLES
#==============================================================================

# Main drawing node (now joint-based implementation with original name)
add_executable(ur5_drawing_node
  src/ur5_drawing_node.cpp
)

target_include_directories(ur5_drawing_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${JSON_INCLUDE_DIRS}
)

ament_target_dependencies(ur5_drawing_node
  rclcpp
  rclcpp_action
  std_srvs
  sensor_msgs
  trajectory_msgs
  control_msgs
  geometry_msgs
  visualization_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  tf2
  tf2_geometry_msgs
  yaml-cpp
)

target_link_libraries(ur5_drawing_node
  ${PROJECT_NAME}_utils
  ${JSON_LIBRARIES}
  yaml-cpp
)

# Joint calibration tool
add_executable(joint_calibration
  src/joint_calibration.cpp
)

target_include_directories(joint_calibration PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(joint_calibration
  rclcpp
  sensor_msgs
  std_srvs
  yaml-cpp
)

target_link_libraries(joint_calibration
  yaml-cpp
)

#==============================================================================
# INSTALL TARGETS
#==============================================================================

# Install executables
install(TARGETS 
  ur5_drawing_node
  joint_calibration
  DESTINATION lib/${PROJECT_NAME}
)

# Install libraries
install(TARGETS
  ${PROJECT_NAME}_utils
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install directories
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  image_description/
  DESTINATION share/${PROJECT_NAME}/image_description
)

# Install header files
install(DIRECTORY 
  include/
  DESTINATION include/${PROJECT_NAME}
)

# Install package.xml
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

#==============================================================================
# EXPORT CONFIGURATION
#==============================================================================

# Export include directories
ament_export_include_directories(
  include
)

# Export libraries
ament_export_libraries(
  ${PROJECT_NAME}_utils
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  rclcpp_action
  std_srvs
  sensor_msgs
  trajectory_msgs
  control_msgs
  geometry_msgs
  visualization_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  tf2
  tf2_geometry_msgs
  yaml-cpp
)

# Export targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

#==============================================================================
# BUILD CONFIGURATION SUMMARY
#==============================================================================

message(STATUS "")
message(STATUS "=== UR5 Drawing System Build Configuration ===")
message(STATUS "Control Mode: Joint-based (with original file names)")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
if(JSON_LIBRARIES)
  message(STATUS "JSON Library: jsoncpp")
else()
  message(STATUS "JSON Library: nlohmann/json (header-only)")
endif()
message(STATUS "YAML Library: yaml-cpp")
message(STATUS "Install Prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "")
message(STATUS "=== Executables ===")
message(STATUS "- ur5_drawing_node (Joint-based control)")
message(STATUS "- joint_calibration")
message(STATUS "")
message(STATUS "=== Libraries ===")
message(STATUS "- ${PROJECT_NAME}_utils (joint-based utilities)")
message(STATUS "")
message(STATUS "NOTE: Original file names maintained, but implementation")
message(STATUS "      has been updated to use joint-based control.")
message(STATUS "")

# Final package setup
ament_package()