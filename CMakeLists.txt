cmake_minimum_required(VERSION 2.8.3)
project(eurathlonTRADR2016)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nifti_robot_driver_msgs
  tf
  pcl_ros)

# find_package(pcl REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${pcl_INCLUDE_DIRS})

add_executable(velocity_conversion src/velocity_conversion.cpp)
target_link_libraries(velocity_conversion ${catkin_LIBRARIES})

add_executable(pcl_filter src/pcl_filter.cpp)
target_link_libraries(pcl_filter ${catkin_LIBRARIES})