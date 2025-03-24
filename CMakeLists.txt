cmake_minimum_required(VERSION 3.5)
project(mrs_uav_hw_api)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[mrs_uav_managers]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MRS_ENABLE_TESTING)
  message(WARNING "Setting optimizaton levels for tests")
  add_definitions("-O0")
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
)

set(LIBRARIES
  MrsUavHwApi_HwApiManager
  MrsUavHwApi_DummyApi
  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${mrs_lib_INCLUDE_DIRS}
  )

# Hw API manager

add_library(MrsUavHwApi_HwApiManager SHARED
  src/hw_api_manager.cpp
  )

ament_target_dependencies(MrsUavHwApi_HwApiManager
  ${DEPENDENCIES}
)

rclcpp_components_register_nodes(MrsUavHwApi_HwApiManager PLUGIN "mrs_uav_hw_api::HwApiManager" EXECUTABLE MrsUavHwApi_HwApiManager)

target_compile_definitions(MrsUavHwApi_HwApiManager PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# Dummy API

add_library(MrsUavHwApi_DummyApi SHARED
  src/dummy_api.cpp
  )

ament_target_dependencies(MrsUavHwApi_DummyApi
  ${DEPENDENCIES}
)

target_compile_definitions(MrsUavHwApi_DummyApi PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

## --------------------------------------------------------------
## |                       Export plugins                       |
## --------------------------------------------------------------

# <package of the base class>, <relative path to the xml>
pluginlib_export_plugin_description_file(mrs_uav_hw_api plugins.xml)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

ament_export_libraries(
  ${LIBRARIES}
)

install(
  TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
