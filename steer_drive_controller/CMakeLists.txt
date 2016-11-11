cmake_minimum_required(VERSION 2.8.3)
project(steer_drive_controller)

find_package(catkin REQUIRED COMPONENTS
    controller_interface
    urdf
    realtime_tools
    tf
    nav_msgs
    roscpp
    angles
    control_toolbox
    gazebo_ros_control
    hardware_interface
    joint_limits_interface
)

find_package(gazebo REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
      roscpp
      angles
      control_toolbox
      gazebo_ros_control
      hardware_interface
      joint_limits_interface
)

include_directories(
  include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(${PROJECT_NAME} src/steer_drive_controller.cpp src/odometry.cpp src/speed_limiter.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES steer_drive_controller_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(steer_drive_test test/steer_drive_test.cpp src/steer_drive_controller.cpp src/odometry.cpp src/speed_limiter.cpp)
target_link_libraries(steer_drive_test ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
#  find_package(catkin COMPONENTS rostest std_srvs controller_manager tf)
#  include_directories(include ${catkin_INCLUDE_DIRS})
#
#  #  add_executable(steer_drive_bot test/diffbot.cpp)
#  #  target_link_libraries(steer_drive_bot ${catkin_LIBRARIES})
#
#  add_executable(steer_drive_bot test/skidsteerbot.cpp)
#  target_link_libraries(steer_drive_bot ${catkin_LIBRARIES})

endif()
#  add_dependencies(tests diffbot skidsteerbot)
#  add_rostest_gtest(diff_drive_test
#    test/diff_drive_controller.test
#    test/diff_drive_test.cpp)
#  target_link_libraries(diff_drive_test ${catkin_LIBRARIES})
#  add_rostest_gtest(diff_drive_nan_test
#    test/diff_drive_controller_nan.test
#    test/diff_drive_nan_test.cpp)
#  target_link_libraries(diff_drive_nan_test ${catkin_LIBRARIES})
#  add_rostest_gtest(diff_drive_limits_test
#    test/diff_drive_controller_limits.test
#    test/diff_drive_limits_test.cpp)
#  target_link_libraries(diff_drive_limits_test ${catkin_LIBRARIES})
#  add_rostest_gtest(diff_drive_timeout_test
#    test/diff_drive_timeout.test
#    test/diff_drive_timeout_test.cpp)
#  target_link_libraries(diff_drive_timeout_test ${catkin_LIBRARIES})
#  add_rostest(test/diff_drive_multipliers.test)
#  add_rostest_gtest(diff_drive_fail_test
#    test/diff_drive_wrong.test
#    test/diff_drive_fail_test.cpp)
#  target_link_libraries(diff_drive_fail_test ${catkin_LIBRARIES})
#  add_rostest_gtest(diff_drive_odom_tf_test
#    test/diff_drive_odom_tf.test
#    test/diff_drive_odom_tf_test.cpp)
#  target_link_libraries(diff_drive_odom_tf_test ${catkin_LIBRARIES})
#  add_rostest_gtest(diff_drive_default_odom_frame_test
#    test/diff_drive_default_odom_frame.test
#    test/diff_drive_default_odom_frame_test.cpp)
#  target_link_libraries(diff_drive_default_odom_frame_test ${catkin_LIBRARIES})
#  add_rostest_gtest(diff_drive_odom_frame_test
#    test/diff_drive_odom_frame.test
#    test/diff_drive_odom_frame_test.cpp)
#  target_link_libraries(diff_drive_odom_frame_test ${catkin_LIBRARIES})
#  add_rostest(test/diff_drive_open_loop.test)
#  add_rostest(test/skid_steer_controller.test)
#  add_rostest(test/skid_steer_no_wheels.test)
#  add_rostest(test/diff_drive_radius_param.test)
#  add_rostest(test/diff_drive_radius_param_fail.test)
#  add_rostest(test/diff_drive_separation_param.test)
#  add_rostest(test/diff_drive_bad_urdf.test)
#endif()
