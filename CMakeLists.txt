cmake_minimum_required(VERSION 3.28)
project(mrs_uav_hw_api)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[mrs_uav_hw_api]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

set(DEPENDENCIES
  ament_cmake
  ament_cmake_ros
  rclcpp
  rclcpp_components
  pluginlib
  mrs_msgs
  mrs_lib
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  Eigen3
  mrs_uav_testing
  backward_ros
)

set(LIBRARIES
  MrsUavHwApi_HwApiManager
  MrsUavHwApi_DummyApi
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

add_library(MrsUavHwApi_MrsUavHwApi INTERFACE)
add_library(mrs_uav_hw_api::mrs_uav_hw_api ALIAS MrsUavHwApi_MrsUavHwApi)
set_target_properties(MrsUavHwApi_MrsUavHwApi
  PROPERTIES
    EXPORT_NAME mrs_uav_hw_api
    OUTPUT_NAME mrs_uav_hw_api
)
target_sources(MrsUavHwApi_MrsUavHwApi
  INTERFACE FILE_SET HEADERS BASE_DIRS "include"
  FILES
    "include/mrs_uav_hw_api/api.h"
    "include/mrs_uav_hw_api/common_handlers.h"
    "include/mrs_uav_hw_api/publishers.h"
)
target_link_libraries(MrsUavHwApi_MrsUavHwApi
  INTERFACE
    rclcpp::rclcpp
    mrs_lib::mrs_lib
    ${mrs_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
)

# Hw API manager

add_library(MrsUavHwApi_HwApiManager SHARED
  src/hw_api_manager.cpp
)

target_link_libraries(MrsUavHwApi_HwApiManager
  PRIVATE
    mrs_uav_hw_api::mrs_uav_hw_api
    rclcpp::rclcpp
    rclcpp_components::component
    pluginlib::pluginlib
    mrs_lib::mrs_lib
    ${mrs_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
)

rclcpp_components_register_nodes(MrsUavHwApi_HwApiManager "mrs_uav_hw_api::HwApiManager")

target_compile_definitions(MrsUavHwApi_HwApiManager PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# Dummy API

add_library(MrsUavHwApi_DummyApi SHARED
  src/dummy_api.cpp
)

target_link_libraries(MrsUavHwApi_DummyApi
  PRIVATE
    mrs_uav_hw_api::mrs_uav_hw_api
    rclcpp::rclcpp
    rclcpp_components::component
    pluginlib::pluginlib
    mrs_lib::mrs_lib
    ${mrs_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
)

target_compile_definitions(MrsUavHwApi_DummyApi PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

## --------------------------------------------------------------
## |                       Export plugins                       |
## --------------------------------------------------------------

# <package of the base class>, <relative path to the xml>
pluginlib_export_plugin_description_file(mrs_uav_hw_api plugins.xml)

## --------------------------------------------------------------
## |                           testing                          |
## --------------------------------------------------------------

if(ENABLE_TESTS)

  add_subdirectory(test)

endif()

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(
  TARGETS
    MrsUavHwApi_MrsUavHwApi
    ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  FILE_SET HEADERS DESTINATION "include/${PROJECT_NAME}"
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)

ament_export_libraries(
  ${LIBRARIES}
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
