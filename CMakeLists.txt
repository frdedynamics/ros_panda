cmake_minimum_required(VERSION 3.0.2)
project(ros_panda)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  dynamic_reconfigure
  franka_control
  franka_description
  franka_gripper
  franka_hw
  geometry_msgs
  hardware_interface
  pluginlib
  realtime_tools
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  genmsg
)

add_message_files(FILES JointCommandPosition.msg)

generate_messages(
	DEPENDENCIES
	std_msgs
	sensor_msgs
	geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_library(ros_panda_controllers
  src/joint_position_controller.cpp
)

add_dependencies(ros_panda_controllers
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(ros_panda_controllers PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(ros_panda_controllers SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

install(TARGETS ros_panda_controllers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES ros_panda_controllers.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
