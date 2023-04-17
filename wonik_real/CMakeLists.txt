cmake_minimum_required(VERSION 2.8.3)
project(wonik_real)

set(CMAKE_CXX_STANDARD 11)
add_compile_options(-std=c++11)


if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release"
  	  CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  trajectory_msgs
  fastech_ros_sdk
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES neo_relayboard
#  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

set(LIBRARY_NAME wonik_drive)
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_library(${LIBRARY_NAME} src/drive.cpp)
add_dependencies(${LIBRARY_NAME} ${catkin_EXPORTED_TARGETS})

# add_executable(${LIBRARY_NAME}_node src/drive_node.cpp)
# target_link_libraries(${LIBRARY_NAME}_node ${PROJECT_NAME})
# target_link_libraries(${LIBRARY_NAME}_node ${catkin_LIBRARIES})
# add_dependencies(${LIBRARY_NAME}_node${catkin_EXPORTED_TARGETS})

