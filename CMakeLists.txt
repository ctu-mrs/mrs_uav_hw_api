cmake_minimum_required(VERSION 3.1.2)
project(mrs_uav_hw_api)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  nodelet
  roscpp
  mrs_msgs
  mrs_lib
  sensor_msgs
  nav_msgs
  std_msgs
  geometry_msgs
  )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

set(LIBRARIES
  MrsUavHwApi MrsUAVHwDummyApi
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS roscpp nodelet mrs_msgs mrs_lib sensor_msgs nav_msgs std_msgs geometry_msgs
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

# MrsUavHwApi

add_library(MrsUavHwApi
  src/hw_api_manager.cpp
  )

add_dependencies(MrsUavHwApi
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsUavHwApi
  ${catkin_LIBRARIES}
  )

# DummyApi

add_library(MrsUAVHwDummyApi
  src/dummy_api.cpp
  )

add_dependencies(MrsUAVHwDummyApi
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsUAVHwDummyApi
  ${catkin_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  )

install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.xml"
  )
