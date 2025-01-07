cmake_minimum_required(VERSION 3.5)  # ROS 2 nécessite au moins CMake 3.5
project(ur_to_touch_haptic_teleoperation)

# Utiliser C++17 ou plus récent
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Trouver les dépendances de ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(yaml-cpp REQUIRED)
find_package(omni_msgs REQUIRED)  # Assurez-vous d'ajouter omni_msgs si nécessaire

# Déclaration des exécutables
add_executable(${PROJECT_NAME}_teleop_node src/teleoperate.cpp)
add_executable(${PROJECT_NAME}_haptic_node src/haptic_feedback.cpp)

# Lier les bibliothèques et dépendances aux exécutables
ament_target_dependencies(${PROJECT_NAME}_teleop_node
    rclcpp
    std_msgs
    geometry_msgs
    yaml-cpp
    omni_msgs  # Ajout de omni_msgs si nécessaire
)

ament_target_dependencies(${PROJECT_NAME}_haptic_node
    rclcpp
    std_msgs
    geometry_msgs
    yaml-cpp
    omni_msgs  # Ajout de omni_msgs si nécessaire
)

# Ajouter des fichiers d'inclusion
include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
)

# Lier les bibliothèques aux exécutables
target_link_libraries(${PROJECT_NAME}_teleop_node
    ${rclcpp_LIBRARIES}
    ${Boost_LIBRARIES}
    yaml-cpp
)

target_link_libraries(${PROJECT_NAME}_haptic_node
    ${rclcpp_LIBRARIES}
    ${Boost_LIBRARIES}
    yaml-cpp
)

# Installer les exécutables
install(TARGETS
    ${PROJECT_NAME}_teleop_node
    ${PROJECT_NAME}_haptic_node
    DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Exporter les dépendances du paquet
ament_package()
