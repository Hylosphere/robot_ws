cmake_minimum_required(VERSION 3.5)
project(rover_control)

# On trouve les outils de base
find_package(ament_cmake REQUIRED)

# ⚠️ LA LIGNE IMPORTANTE : On installe le dossier config pour que ROS le trouve
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()