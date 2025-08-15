cmake_minimum_required(VERSION 3.5)
project(rover_bringup)

find_package(ament_cmake REQUIRED)

# Installe les répertoires de lancement et params si tu les ajoutes ensuite
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_package()
