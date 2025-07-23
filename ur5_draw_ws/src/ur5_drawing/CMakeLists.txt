cmake_minimum_required(VERSION 3.8)
project(ur5_drawing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)

# Find yaml-cpp
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

# Add nlohmann_json
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

if(NOT JSONCPP_FOUND)
  find_path(NLOHMANN_JSON_INCLUDE_DIR nlohmann/json.hpp
    PATHS /usr/include /usr/local/include)
  if(NOT NLOHMANN_JSON_INCLUDE_DIR)
    message(FATAL_ERROR "nlohmann/json not found. Please install: sudo apt-get install nlohmann-json3-dev")
  endif()
endif()

# Include directories
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${YAML_CPP_INCLUDE_DIRS})

# Add executable
add_executable(ur5_drawing_node 
  src/ur5_drawing_node.cpp
  src/drawing_utils.cpp
  src/config_loader.cpp
)

# Target dependencies
ament_target_dependencies(ur5_drawing_node
  rclcpp
  std_srvs
  geometry_msgs
  visualization_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

# Link libraries and set compile flags
target_link_libraries(ur5_drawing_node
  ${YAML_CPP_LIBRARIES}
)

target_compile_options(ur5_drawing_node PRIVATE ${YAML_CPP_CFLAGS_OTHER})

if(JSONCPP_FOUND)
  target_link_libraries(ur5_drawing_node ${JSONCPP_LIBRARIES})
  target_include_directories(ur5_drawing_node PRIVATE ${JSONCPP_INCLUDE_DIRS})
else()
  target_include_directories(ur5_drawing_node PRIVATE ${NLOHMANN_JSON_INCLUDE_DIR})
endif()

# Install targets
install(TARGETS
  ur5_drawing_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install resource files
install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()