cmake_minimum_required(VERSION 3.0.2)

project(visual_navigation)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
)
set(OpenCV_DIR "/usr/local/lib/opencv4")
find_package(OpenCV REQUIRED)

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES visual_navigation
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs geometry_msgs
  DEPENDS OpenCV
)

file(GLOB FILES
    "${PROJECT_SOURCE_DIR}/include/*.h"
    "${PROJECT_SOURCE_DIR}/src/*.cpp"
    "${PROJECT_SOURCE_DIR}/src/*.c"
)

add_executable(visual_navigation ${FILES})


target_link_libraries(visual_navigation ${catkin_LIBRARIES} ${OpenCV_LIBS})

