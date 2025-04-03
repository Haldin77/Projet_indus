cmake_minimum_required(VERSION 3.8)
project(ur_control)

# Trouver ROS 2 et ses dépendances
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(omni_msgs REQUIRED)

# Ajouter les fichiers sources et les headers
add_executable(ur3e_node src/ur3e.cpp)
ament_target_dependencies(ur3e_node rclcpp std_msgs geometry_msgs sensor_msgs omni_msgs)

# Installer l'exécutable
install(TARGETS
  ur3e_node
  DESTINATION lib/${PROJECT_NAME}
)

# Installer les fichiers de lancement
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
