get_filename_component(TEST_NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)

add_executable(test_${TEST_NAME}
  test.cpp
  )

ament_target_dependencies(test_${TEST_NAME}
  rclcpp
  mrs_lib
  mrs_msgs
  backward_ros
  )

target_link_libraries(test_${TEST_NAME}
  MrsUavHwApi_HwApiManager
  MrsUavHwApi_DummyApi
  )

install(TARGETS
  test_${TEST_NAME}
  DESTINATION lib/${PROJECT_NAME}
  )

add_ros_isolated_launch_test(test.py TIMEOUT 300)
