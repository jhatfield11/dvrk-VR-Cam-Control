cmake_minimum_required(VERSION 3.0.2)
project(ros_dvrk_vr)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  ros_tcp_endpoint
  message_generation
  sensor_msgs
  controller_manager
  roslaunch
  trajectory_msgs
  moveit_servo
  moveit_msgs
)
roslaunch_add_file_check(launch)

add_message_files(
FILES
XYZcloud.msg)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS moveit_servo moveit_msgs roscpp sensor_msgs std_msgs geometry_msgs message_runtime
)

catkin_install_python(PROGRAMS
  scripts/server_endpoint.py
  scripts/Camera_control.py
  scripts/Base_subscriber.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


add_executable(pose_tracking_arms src/pose_tracking_arms.cpp)
target_link_libraries(pose_tracking_arms 
${catkin_LIBRARIES}
${Boost_LIBRARIES})
add_dependencies(pose_tracking_arms ${catkin_EXPORTED_TARGETS})

add_executable(service_call src/service_call.cpp)
target_link_libraries(service_call 
${catkin_LIBRARIES})
add_dependencies(service_call ${catkin_EXPORTED_TARGETS})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM
  ${Boost_INCLUDE_DIR}
)
#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_niryo_moveit.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
