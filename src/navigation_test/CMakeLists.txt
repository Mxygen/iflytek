cmake_minimum_required(VERSION 3.0.2)
project(navigation_test)

set(CMAKE_CXX_STANDARD 17)
# set(ncnn_DIR "/home/ucar/Desktop/ncnn/build/install/lib/cmake/ncnn")
# set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-10.0")


## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  geometry_msgs
  roscpp
  std_srvs
  tf2
  tf2_ros
  dynamic_reconfigure
  rospy
  image_transport
  cv_bridge
  # costmap_2d
)

# find_package(OpenMP REQUIRED)
# if(OPENMP_FOUND)
#     message("OPENMP FOUND")
#     set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#     set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
# endif()

# find_package(OpenCV REQUIRED)
#find_package(OpenCV REQUIRED PATHS "/path/to/opencv-3.4.16_gcc8.2_ffmpeg")

#find_package(ncnn REQUIRED)
#find_package(ncnn REQUIRED PATHS "/path/to/ncnn/build/install/lib/cmake/ncnn")
# if(NOT TARGET ncnn)
#     message(WARNING "ncnn NOT FOUND!  Please set ncnn_DIR environment variable")
# else()
#     message("ncnn FOUND ")
# endif()

# find_package(OpenMP REQUIRED)
# if(OPENMP_FOUND)
#     message("OPENMP FOUND")
#     set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#     set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
# endif()

find_package(yaml-cpp REQUIRED)


include_directories(
    # ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${catkin_INCLUDE_DIRS}
    include
)

# find_package(yaml-cpp REQUIRED)
# include_directories(${YAML_CPP_INCLUDE_DIR})
# target_link_libraries(node_name yaml-cpp)

include_directories(include ${catkin_INCLUDE_DIRS})
generate_dynamic_reconfigure_options(
  cfg/TebLocalPlannerReconfigure.cfg
)



## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES navigation_test
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


include_directories("/usr/include/eigen3")

#这里是ucar_nav_province_first-1.launch的文件
#add_executable(ucar_nav_zhidaowuzhangaijiasu_7_6_gs2wdm src/ucar_nav_zhidaowuzhangaijiasu_7_6_gs2wdm.cpp)
#target_link_libraries(ucar_nav_zhidaowuzhangaijiasu_7_6_gs2wdm ${catkin_LIBRARIES})

#add_executable(nav_config src/nav_config.cpp)
#target_link_libraries(nav_config ${catkin_LIBRARIES})

#add_executable(get_pic_FL src/get_pic_FL.cpp src/picodet.cpp src/vision_ncnn.cpp src/find_line.cpp)
#target_link_libraries(get_pic_FL ncnn ${OpenCV_LIBS} ${catkin_LIBRARIES})

#add_executable(ucar_navigation src/ucar_navigation.cpp src/picodet.cpp src/vision_ncnn.cpp)
#target_link_libraries(ucar_navigation ncnn ${OpenCV_LIBS} ${catkin_LIBRARIES})

#target_link_libraries(get_pic_FL ncnn ${OpenCV_LIBS} ${catkin_LIBRARIES} yaml-cpp)
#target_link_libraries(ucar_navigation ncnn ${OpenCV_LIBS} ${catkin_LIBRARIES} yaml-cpp)

#ucar_nav_province_first.launch的文件
#add_executable(ucar_navigation_copy src/ucar_navigation_copy.cpp)
#target_link_libraries(ucar_navigation_copy ${catkin_LIBRARIES})

# add_executable(ucar_pure_pursuit_control_true_teb_cv_gs_one src/ucar_pure_pursuit_control_true_teb_cv_gs_one.cpp)
# target_link_libraries(ucar_pure_pursuit_control_true_teb_cv_gs_one ${catkin_LIBRARIES})


## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_navigation_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
