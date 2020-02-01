cmake_minimum_required(VERSION 2.8.3)
project(test_ariac)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(catkin REQUIRED COMPONENTS
  ariac_example
  nist_gear
)

# Used by test/
set(ROS_PACKAGE_NAME nist_gear)
catkin_package(
  CATKIN_DEPENDS
  nist_gear
  ariac_example
)

set(rostests
  test/ariac_rosapi.test
  test/ariac_rosapi_development_mode.test
  test/ariac_example.test
  test/ariac_tf_frames.test
  test/gripper.test
  test/gripper_drop_over_bins.test
  test/gripper_drop_over_box.test
  # gripper_unthrottled.test  # this needs a good gripper pose; regression is fixed.
  test/sensors.test
  test/sensor_blackout.test
  test_scoring/ariac_scoring_perfect_shipment.test
  test_scoring/ariac_scoring_after_waiting.test
  test_scoring/ariac_scoring_after_waiting_with_state_logging.test
  test_scoring/ariac_scoring_perfect_shipment_flipped.test
  test_scoring/ariac_scoring_correct_parts_incorrect_poses.test
  test_scoring/ariac_scoring_correct_parts_plus_unwanted.test
  test_scoring/ariac_scoring_correct_parts_plus_faulty.test
  test_scoring/ariac_scoring_correct_parts_not_flipped.test
  test_scoring/ariac_scoring_missing_parts.test
  test_scoring/ariac_scoring_low_priority_order.test
  test_scoring/ariac_scoring_interrupted.test
  test_scoring/ariac_scoring_updated_order.test
  test_scoring/ariac_scoring_not_interrupted.test
  test_scoring/ariac_scoring_faulty_parts.test
  test_scoring/ariac_scoring_faulty_parts_randomized.test
  test_scoring/ariac_scoring_arm_collision.test
  test_scoring/ariac_scoring_wrong_agv.test
)

foreach(rostest ${rostests})
  # We don't set a timeout here because we trust rostest to enforce the
  # timeout specified in each .test file.
  add_test(${rostest} rostest ${CMAKE_CURRENT_SOURCE_DIR}/${rostest})
  # Check for test result file and create one if needed.  rostest can fail to
  # generate a file if it throws an exception.
  add_test(check_${rostest} rosrun rosunit check_test_ran.py --rostest ${ROS_PACKAGE_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${rostest})
endforeach()

# Install test files because we'll run tests from the install tree.  We could
# get away with running most of the test from the source tree, but it's clearer
# and safer to run them from the install tree.
install(DIRECTORY
  test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)
install(DIRECTORY
  test_scoring
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  ros_api_checker
  test_example_node.py
  test_gripper.py
  test_gripper_drop_over_bins.py
  test_gripper_drop_over_box.py
  test_gripper_unthrottled.py
  test_scoring_against_expected_score.py
  test_sensor_blackout.py
  test_sensors.py
  test_tf_frames.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)
