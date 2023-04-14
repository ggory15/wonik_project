cmake_minimum_required(VERSION 2.8.3)
project(wonik_kinematics_mecanum)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
 nav_msgs
 roscpp
 tf 
 std_msgs
 sensor_msgs
 trajectory_msgs
 )

catkin_package(
    CATKIN_DEPENDS std_msgs tf nav_msgs sensor_msgs
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

SET(${PROJECT_NAME}_HEADERS
  include/wonik_kinematics_mecanum/Kinematics.h
  include/wonik_kinematics_mecanum/MecanumKinematics.h
)

SET(${PROJECT_NAME}_SOURCES
  src/Mecanum4WKinematics.cpp
)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS} )
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} SYSTEM PUBLIC $<INSTALL_INTERFACE:include>)
add_dependencies(${PROJECT_NAME}  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(wonik_kinematics_mecanum_node src/wonik_kinematics_mecanum_node.cpp)
add_dependencies(wonik_kinematics_mecanum_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(wonik_kinematics_mecanum_node ${PROJECT_NAME})
target_link_libraries(wonik_kinematics_mecanum_node ${catkin_LIBRARIES})

add_executable(wonik_kinematics_mecanum_sim_node src/wonik_kinematics_mecanum_sim_node.cpp)
add_dependencies(wonik_kinematics_mecanum_sim_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(wonik_kinematics_mecanum_sim_node ${PROJECT_NAME})
target_link_libraries(wonik_kinematics_mecanum_sim_node ${catkin_LIBRARIES})

