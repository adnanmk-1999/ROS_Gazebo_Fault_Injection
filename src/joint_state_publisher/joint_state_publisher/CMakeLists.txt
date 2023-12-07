cmake_minimum_required(VERSION 3.0.2)
project(joint_state_publisher)

find_package(catkin REQUIRED
  roscpp
  rospy
  rosmsg
  std_msgs
  sensor_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  RecorderMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
 
)
catkin_package(
  CATKIN_DEPENDS
  std_msgs
  sensor_msgs
  message_runtime
)
catkin_install_python(PROGRAMS
  scripts/joint_state_publisher scripts/fault_injector scripts/action_publisher scripts/recorder
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



if(CATKIN_ENABLE_TESTING)
  find_package(rostest)
  add_rostest(test/test_mimic_chain.launch)
  add_rostest(test/test_mimic_cycle.launch)
  add_rostest(test/test_zero_joints.launch)
  add_rostest(test/test_multi_joints_urdf.launch)
  add_rostest(test/test_multi_joints_collada.launch)
  add_rostest(test/test_64_joint_robot.launch)
  add_rostest(test/test_slash_fiction.launch)
endif()
