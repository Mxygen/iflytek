cmake_minimum_required(VERSION 3.10)

find_package(OpenCV)
include_directories(OpenCV_INCLUDE_DIRS)
message(${OpenCV_INCLUDE_DIRS})
message(${OpenCV_LIBRARIES})

add_definitions("-Wall -std=c++11")

if(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE -Ofast)
 
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -O3  -Wall")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3  -Wall")



 # add_executable(${PROJECT_NAME} ${SRC_LIST})

add_executable(Main_3 tesxt.cpp find_copy.cpp)

target_link_libraries(Main_3 ${OpenCV_LIBRARIES})
