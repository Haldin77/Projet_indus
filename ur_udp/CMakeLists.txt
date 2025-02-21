cmake_minimum_required(VERSION 3.8)
project(ur_udp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(omni_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Ajouter l'exécutable
add_executable(ur_udp src/ur_udp.cpp)
add_executable(phantom_udp src/phantom_udp.cpp)

include_directories(include/ur_udp)

# Lier l'exécutable avec les bibliothèques ROS 2 nécessaires
ament_target_dependencies(ur_udp 
  rclcpp
  std_msgs
  geometry_msgs
  omni_msgs
)

ament_target_dependencies(phantom_udp 
  rclcpp
  std_msgs
  geometry_msgs
  omni_msgs
)

# Installer l'exécutable dans le bon dossier
install(TARGETS
  ur_udp
  phantom_udp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

