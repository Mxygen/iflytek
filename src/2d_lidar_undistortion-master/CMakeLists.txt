cmake_minimum_required(VERSION 2.8.3)
project(lidar_undistortion)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  sensor_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES LaserUndistortion
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(lidar_undistortion src/lidar_undistortion.cpp)

target_link_libraries(lidar_undistortion
  ${catkin_LIBRARIES}
)

#add_executable(line_fitting_node src/line_fitting_node.cpp)

#target_link_libraries(line_fitting_node
#  ${catkin_LIBRARIES}
#)


#add_executable(line_fitting_new src/line_fitting_new.cpp)

#target_link_libraries(line_fitting_new
#  ${catkin_LIBRARIES}
#)
