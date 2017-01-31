cmake_minimum_required(VERSION 2.8.3)
project(rpc_game_client)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  sensor_msgs
)

add_message_files(
  FILES
  Alive.msg
  GameState.msg
  Score.msg
)

add_service_files(
  FILES
  ClientScore.srv
  PlayerScore.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
install(PROGRAMS
  scripts/game_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
