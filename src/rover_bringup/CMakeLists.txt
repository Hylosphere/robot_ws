cmake_minimum_required(VERSION 3.5)
project(rover_bringup)

find_package(ament_cmake REQUIRED)

# AJOUT DE "rviz" ICI 👇
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()