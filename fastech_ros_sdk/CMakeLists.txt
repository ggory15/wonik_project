cmake_minimum_required(VERSION 3.0.2)
project(fastech_ros_sdk)

add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}" )

add_definitions(-D_OS_UNIX)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  sensor_msgs
)

add_service_files(
  FILES
    ServoOn.srv
)


generate_messages(
  DEPENDENCIES std_msgs  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp message_runtime std_msgs
  DEPENDS system_lib
)

link_directories(${Boost_LIBRARY_DIR})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

SET(LIBRARY_NAME fastech_sdk)

SET(${LIBRARY_NAME}_HEADERS
  include/fastech_ros_sdk/fastech/util/Definition.h

  include/fastech_ros_sdk/fastech/fastech_step.hpp
)

SET(${LIBRARY_NAME}_SOURCES
  src/fastech/fastech_step.cpp
)

ADD_LIBRARY(${LIBRARY_NAME} SHARED ${${LIBRARY_NAME}_SOURCES} ${${LIBRARY_NAME}_HEADERS} )
TARGET_INCLUDE_DIRECTORIES(${LIBRARY_NAME} SYSTEM PUBLIC $<INSTALL_INTERFACE:include>)
add_dependencies(${LIBRARY_NAME}  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${LIBRARY_NAME} ${PROJECT_NAME}_generate_messages_cpp )


add_executable(${PROJECT_NAME} src/fastech_ros.cpp)
add_dependencies(${PROJECT_NAME}  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${LIBRARY_NAME} ${catkin_LIBRARIES} -lpthread)
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME}  SYSTEM PUBLIC $<INSTALL_INTERFACE:include>)

install(
  DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)