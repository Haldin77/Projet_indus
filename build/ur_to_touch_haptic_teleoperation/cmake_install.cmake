# Install script for directory: /home/dilhan/ros2_ws/src/ur_to_touch_haptic_teleoperation

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/dilhan/ros2_ws/install/ur_to_touch_haptic_teleoperation")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur_to_touch_haptic_teleoperation/ur_to_touch_haptic_teleoperation_teleop_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur_to_touch_haptic_teleoperation/ur_to_touch_haptic_teleoperation_teleop_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur_to_touch_haptic_teleoperation/ur_to_touch_haptic_teleoperation_teleop_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ur_to_touch_haptic_teleoperation" TYPE EXECUTABLE FILES "/home/dilhan/ros2_ws/build/ur_to_touch_haptic_teleoperation/ur_to_touch_haptic_teleoperation_teleop_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur_to_touch_haptic_teleoperation/ur_to_touch_haptic_teleoperation_teleop_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur_to_touch_haptic_teleoperation/ur_to_touch_haptic_teleoperation_teleop_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur_to_touch_haptic_teleoperation/ur_to_touch_haptic_teleoperation_teleop_node"
         OLD_RPATH "/opt/ros/humble/lib:/home/dilhan/ros2_ws/install/omni_msgs/lib:/usr/local/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur_to_touch_haptic_teleoperation/ur_to_touch_haptic_teleoperation_teleop_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur_to_touch_haptic_teleoperation/ur_to_touch_haptic_teleoperation_haptic_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur_to_touch_haptic_teleoperation/ur_to_touch_haptic_teleoperation_haptic_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur_to_touch_haptic_teleoperation/ur_to_touch_haptic_teleoperation_haptic_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ur_to_touch_haptic_teleoperation" TYPE EXECUTABLE FILES "/home/dilhan/ros2_ws/build/ur_to_touch_haptic_teleoperation/ur_to_touch_haptic_teleoperation_haptic_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur_to_touch_haptic_teleoperation/ur_to_touch_haptic_teleoperation_haptic_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur_to_touch_haptic_teleoperation/ur_to_touch_haptic_teleoperation_haptic_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur_to_touch_haptic_teleoperation/ur_to_touch_haptic_teleoperation_haptic_node"
         OLD_RPATH "/opt/ros/humble/lib:/home/dilhan/ros2_ws/install/omni_msgs/lib:/usr/local/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur_to_touch_haptic_teleoperation/ur_to_touch_haptic_teleoperation_haptic_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_to_touch_haptic_teleoperation" TYPE DIRECTORY FILES "/home/dilhan/ros2_ws/src/ur_to_touch_haptic_teleoperation/launch")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_to_touch_haptic_teleoperation" TYPE DIRECTORY FILES "/home/dilhan/ros2_ws/src/ur_to_touch_haptic_teleoperation/config")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/dilhan/ros2_ws/build/ur_to_touch_haptic_teleoperation/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/ur_to_touch_haptic_teleoperation")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/dilhan/ros2_ws/build/ur_to_touch_haptic_teleoperation/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/ur_to_touch_haptic_teleoperation")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_to_touch_haptic_teleoperation/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_to_touch_haptic_teleoperation/environment" TYPE FILE FILES "/home/dilhan/ros2_ws/build/ur_to_touch_haptic_teleoperation/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_to_touch_haptic_teleoperation/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_to_touch_haptic_teleoperation/environment" TYPE FILE FILES "/home/dilhan/ros2_ws/build/ur_to_touch_haptic_teleoperation/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_to_touch_haptic_teleoperation" TYPE FILE FILES "/home/dilhan/ros2_ws/build/ur_to_touch_haptic_teleoperation/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_to_touch_haptic_teleoperation" TYPE FILE FILES "/home/dilhan/ros2_ws/build/ur_to_touch_haptic_teleoperation/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_to_touch_haptic_teleoperation" TYPE FILE FILES "/home/dilhan/ros2_ws/build/ur_to_touch_haptic_teleoperation/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_to_touch_haptic_teleoperation" TYPE FILE FILES "/home/dilhan/ros2_ws/build/ur_to_touch_haptic_teleoperation/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_to_touch_haptic_teleoperation" TYPE FILE FILES "/home/dilhan/ros2_ws/build/ur_to_touch_haptic_teleoperation/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/dilhan/ros2_ws/build/ur_to_touch_haptic_teleoperation/ament_cmake_index/share/ament_index/resource_index/packages/ur_to_touch_haptic_teleoperation")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_to_touch_haptic_teleoperation/cmake" TYPE FILE FILES
    "/home/dilhan/ros2_ws/build/ur_to_touch_haptic_teleoperation/ament_cmake_core/ur_to_touch_haptic_teleoperationConfig.cmake"
    "/home/dilhan/ros2_ws/build/ur_to_touch_haptic_teleoperation/ament_cmake_core/ur_to_touch_haptic_teleoperationConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_to_touch_haptic_teleoperation" TYPE FILE FILES "/home/dilhan/ros2_ws/src/ur_to_touch_haptic_teleoperation/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/dilhan/ros2_ws/build/ur_to_touch_haptic_teleoperation/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
