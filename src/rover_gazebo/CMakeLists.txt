cmake_minimum_required(VERSION 3.5)
project(rover_gazebo)

find_package(ament_cmake REQUIRED)

# On installe le dossier launch
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()