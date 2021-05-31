cmake_minimum_required(VERSION 2.8.3)
project(uwb_posegraph)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    roslib
    )


find_package(Ceres REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR})

catkin_package()

add_executable(uwb_posegraph
    src/uwb_posegraph_node.cpp
    src/uwb_posegraph.cpp
    src/keyframe.cpp
    )

target_link_libraries(uwb_posegraph ${catkin_LIBRARIES}  ${CERES_LIBRARIES}) 
# message("catkin_lib  ${catkin_LIBRARIES}")
