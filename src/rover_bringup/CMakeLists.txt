cmake_minimum_required(VERSION 3.5)
project(rover_bringup)

find_package(ament_cmake REQUIRED)

# Installe le dossier launch pour que ROS puisse le trouver
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()