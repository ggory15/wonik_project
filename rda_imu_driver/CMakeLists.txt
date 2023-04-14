cmake_minimum_required(VERSION 2.8.3)
project(rda_imu_driver)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  geometry_msgs
  sensor_msgs
  tf
  )

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs tf
  )
include_directories(include ${catkin_INCLUDE_DIRS})
# Building
add_executable(rda_imu_driver_node src/rda_imu_driver_node.cpp)
target_link_libraries(rda_imu_driver_node ${catkin_LIBRARIES})
set_target_properties(rda_imu_driver_node PROPERTIES OUTPUT_NAME rda_imu_driver_node PREFIX "")

install(TARGETS rda_imu_driver_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# if more folders like config add below
#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
