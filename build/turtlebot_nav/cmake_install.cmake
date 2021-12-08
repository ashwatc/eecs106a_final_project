# Install script for directory: /home/cc/ee106a/fl21/class/ee106a-aes/Desktop/eecs106a_final_project/src/turtlebot_nav

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/cc/ee106a/fl21/class/ee106a-aes/Desktop/eecs106a_final_project/install")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot_nav/srv" TYPE FILE FILES
    "/home/cc/ee106a/fl21/class/ee106a-aes/Desktop/eecs106a_final_project/src/turtlebot_nav/srv/Hop.srv"
    "/home/cc/ee106a/fl21/class/ee106a-aes/Desktop/eecs106a_final_project/src/turtlebot_nav/srv/Rescue.srv"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot_nav/cmake" TYPE FILE FILES "/home/cc/ee106a/fl21/class/ee106a-aes/Desktop/eecs106a_final_project/build/turtlebot_nav/catkin_generated/installspace/turtlebot_nav-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/cc/ee106a/fl21/class/ee106a-aes/Desktop/eecs106a_final_project/devel/include/turtlebot_nav")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/cc/ee106a/fl21/class/ee106a-aes/Desktop/eecs106a_final_project/devel/share/roseus/ros/turtlebot_nav")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/cc/ee106a/fl21/class/ee106a-aes/Desktop/eecs106a_final_project/devel/share/common-lisp/ros/turtlebot_nav")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/cc/ee106a/fl21/class/ee106a-aes/Desktop/eecs106a_final_project/devel/share/gennodejs/ros/turtlebot_nav")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/cc/ee106a/fl21/class/ee106a-aes/Desktop/eecs106a_final_project/devel/lib/python2.7/dist-packages/turtlebot_nav")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/cc/ee106a/fl21/class/ee106a-aes/Desktop/eecs106a_final_project/devel/lib/python2.7/dist-packages/turtlebot_nav")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/cc/ee106a/fl21/class/ee106a-aes/Desktop/eecs106a_final_project/build/turtlebot_nav/catkin_generated/installspace/turtlebot_nav.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot_nav/cmake" TYPE FILE FILES "/home/cc/ee106a/fl21/class/ee106a-aes/Desktop/eecs106a_final_project/build/turtlebot_nav/catkin_generated/installspace/turtlebot_nav-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot_nav/cmake" TYPE FILE FILES
    "/home/cc/ee106a/fl21/class/ee106a-aes/Desktop/eecs106a_final_project/build/turtlebot_nav/catkin_generated/installspace/turtlebot_navConfig.cmake"
    "/home/cc/ee106a/fl21/class/ee106a-aes/Desktop/eecs106a_final_project/build/turtlebot_nav/catkin_generated/installspace/turtlebot_navConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot_nav" TYPE FILE FILES "/home/cc/ee106a/fl21/class/ee106a-aes/Desktop/eecs106a_final_project/src/turtlebot_nav/package.xml")
endif()

