cmake_minimum_required(VERSION 3.5)
project(mrs_uav_hw_api)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
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

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

set(LIBRARIES
  MrsUavHwApi_HwApiManager
  MrsUavHwApi_DummyApi
  )

include_directories(
  include
  )

# MrsUavHwApi_HwApiManager

add_library(MrsUavHwApi_HwApiManager SHARED
  src/hw_api_manager.cpp
  )

ament_target_dependencies(MrsUavHwApi_HwApiManager
  ${DEPENDENCIES}
)

rclcpp_components_register_nodes(MrsUavHwApi_HwApiManager PLUGIN "mrs_uav_hw_api::HwApiManager" EXECUTABLE MrsUavHwApi_HwApiManager)

# MrsUavHwApi_DummyApi

add_library(MrsUavHwApi_DummyApi SHARED
  src/dummy_api.cpp
  )

ament_target_dependencies(MrsUavHwApi_DummyApi
  ${DEPENDENCIES}
)

# <package of the base class>, <relative path to the xml>
pluginlib_export_plugin_description_file(mrs_uav_hw_api plugins.xml)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

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

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories("include")

ament_export_libraries(
  ${LIBRARIES}
)

ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
