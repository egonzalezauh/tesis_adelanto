cmake_minimum_required(VERSION 3.8)
project(posture_game)

# ========== Configuración para C++ ==========
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(posture_game_interfaces REQUIRED)
find_package(trajectory_msgs REQUIRED)


# Ejecutables de  C++ (pueden haber mas de un add_executable)
add_executable(checker_node src/checker_node.cpp)

ament_target_dependencies(checker_node
  rclcpp
  std_msgs
  posture_game_interfaces
  trajectory_msgs
)

add_executable(movement_publisher_node src/movement_publisher_node.cpp)
ament_target_dependencies(movement_publisher_node
  rclcpp
  std_msgs
  trajectory_msgs
)

# Incluir headers
include_directories(
  include
)

# ========== Configuración para Python ==========
find_package(rclpy REQUIRED)

install(PROGRAMS
  scripts/game_manage_node.py
  scripts/detect_emotions_node.py
  scripts/mediapipe_node.py
  scripts/message.py
  scripts/speaker_node.py
  scripts/stats_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

# ========== Instalación común ==========
install(DIRECTORY
  include/
  scripts/
  src/
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  resource/models/TTS
  DESTINATION share/${PROJECT_NAME}/models
)

install(DIRECTORY
  resource/models/emotions
  DESTINATION share/${PROJECT_NAME}/models
)


install(TARGETS
  checker_node
  movement_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  rclpy
  std_msgs
  posture_game_interfaces
  trajectory_msgs
)

#include_directories(
  #include
  #${rclcpp_INCLUDE_DIRS}
#)

ament_export_include_directories(include)
ament_package()
