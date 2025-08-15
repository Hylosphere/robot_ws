cmake_minimum_required(VERSION 3.5)
project(waveshare_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# (Pas encore de sources, on posera la SystemInterface plus tard)
# add_library(${PROJECT_NAME} SHARED src/waveshare_system.cpp)
# ament_target_dependencies(${PROJECT_NAME} rclcpp hardware_interface pluginlib)
# pluginlib_export_plugin_description_file(hardware_interface waveshare_driver.xml)

install(DIRECTORY include/
  DESTINATION include
  OPTIONAL)

ament_package()
