cmake_minimum_required(VERSION 3.5)
project(posture_game)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(posture_game_interfaces REQUIRED)

ament_package()
