cmake_minimum_required(VERSION 3.5)
project(ur_proxysim_description)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(PCL_CONVERSIONS_INCLUDE_DIRS)

add_executable(tof_cloud_processing_node src/tof_cloud_processing_node.cpp src/custom_sync_policy.hpp)
ament_target_dependencies(tof_cloud_processing_node
  rclcpp
  sensor_msgs
  message_filters
  PCL
  tf2
  tf2_ros
  pcl_conversions
  pcl_msgs
  pcl_ros
  geometry_msgs
  sensor_msgs
)

install(
  DIRECTORY config launch meshes rviz urdf test gazebo_models
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS tof_cloud_processing_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(ur_urdf_xacro test/test_ur_urdf_xacro.py)
  ament_add_pytest_test(view_ur_launch test/test_view_ur_launch.py)
endif()

ament_package()
