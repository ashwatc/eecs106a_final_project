# Install script for directory: /home/cc/ee106a/fl21/class/ee106a-adg/eecs106a_final_project/turtlebot_navigation/src/process_usb_cam

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/cc/ee106a/fl21/class/ee106a-adg/eecs106a_final_project/turtlebot_navigation/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/cc/ee106a/fl21/class/ee106a-adg/eecs106a_final_project/turtlebot_navigation/build/process_usb_cam/catkin_generated/installspace/process_usb_cam.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/process_usb_cam/cmake" TYPE FILE FILES
    "/home/cc/ee106a/fl21/class/ee106a-adg/eecs106a_final_project/turtlebot_navigation/build/process_usb_cam/catkin_generated/installspace/process_usb_camConfig.cmake"
    "/home/cc/ee106a/fl21/class/ee106a-adg/eecs106a_final_project/turtlebot_navigation/build/process_usb_cam/catkin_generated/installspace/process_usb_camConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/process_usb_cam" TYPE FILE FILES "/home/cc/ee106a/fl21/class/ee106a-adg/eecs106a_final_project/turtlebot_navigation/src/process_usb_cam/package.xml")
endif()

