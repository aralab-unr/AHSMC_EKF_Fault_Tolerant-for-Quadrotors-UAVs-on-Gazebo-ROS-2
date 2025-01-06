cmake_minimum_required(VERSION 3.8)  # ROS 2 requires at least version 3.8
project(ahsmcekf)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)

# Find the CasADi library

# Include directories
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}  # Ensure Eigen3 is included here
)

# Link directories (if needed for Gazebo or other custom libraries)
link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(ahsmc src/ahsmc.cpp)
ament_target_dependencies(ahsmc rclcpp sensor_msgs std_msgs tf2_ros gazebo_msgs std_srvs nav_msgs)

add_executable(pidcontroller src/pidcontroller.cpp)
ament_target_dependencies(pidcontroller rclcpp sensor_msgs std_msgs tf2_ros gazebo_msgs std_srvs nav_msgs)

add_executable(EKFnode src/EKFnode.cpp)
ament_target_dependencies(EKFnode rclcpp sensor_msgs std_msgs tf2_ros gazebo_msgs std_srvs nav_msgs)

# Add Eigen3 to the EKFnode target
target_include_directories(EKFnode PRIVATE ${EIGEN3_INCLUDE_DIR})
target_include_directories(ahsmc PRIVATE ${EIGEN3_INCLUDE_DIR})

add_library(uavplugin SHARED src/uavplugin.cc)
ament_target_dependencies(uavplugin rclcpp std_msgs)
target_link_libraries(uavplugin ${GAZEBO_LIBRARIES} rclcpp::rclcpp)

# Install targets
install(TARGETS
  ahsmc
  EKFnode
  pidcontroller
  uavplugin  # Install the uavplugin
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Export package dependencies
ament_export_dependencies(rclcpp sensor_msgs std_msgs tf2_ros gazebo_ros std_srvs)
ament_package()
