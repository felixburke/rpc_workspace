cmake_minimum_required(VERSION 3.8)
project(tams_lasertag_client)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SubmitHit.srv"
  "msg/Scoreboard.msg"
  "msg/Score.msg"
  DEPENDENCIES sensor_msgs
)

add_executable(game_client_node
  src/game_client_node.cpp
)

ament_target_dependencies(game_client_node
  rclcpp
  sensor_msgs
  cv_bridge
  nlohmann_json
  irobot_create_msgs
)

target_link_libraries(game_client_node
  CURL::libcurl
  ${OpenCV_LIBS}
  nlohmann_json::nlohmann_json
)

rosidl_get_typesupport_target(
  cpp_typesupport_target
  "${PROJECT_NAME}"
  "rosidl_typesupport_cpp"
)

target_link_libraries(
  game_client_node
  "${cpp_typesupport_target}"
)

install(TARGETS
  game_client_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS
)
# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS
)

ament_package()

