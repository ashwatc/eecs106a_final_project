# Install script for directory: /home/cc/ee106a/fl21/class/ee106a-aes/Desktop/eecs106a_final_project/src/baxter_search

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/baxter_search/srv" TYPE FILE FILES
    "/home/cc/ee106a/fl21/class/ee106a-aes/Desktop/eecs106a_final_project/src/baxter_search/srv/Guide.srv"
    "/home/cc/ee106a/fl21/class/ee106a-aes/Desktop/eecs106a_final_project/src/baxter_search/srv/SweepPose.srv"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/baxter_search/cmake" TYPE FILE FILES "/home/cc/ee106a/fl21/class/ee106a-aes/Desktop/eecs106a_final_project/build/baxter_search/catkin_generated/installspace/baxter_search-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/cc/ee106a/fl21/class/ee106a-aes/Desktop/eecs106a_final_project/devel/include/baxter_search")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/cc/ee106a/fl21/class/ee106a-aes/Desktop/eecs106a_final_project/devel/share/roseus/ros/baxter_search")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/cc/ee106a/fl21/class/ee106a-aes/Desktop/eecs106a_final_project/devel/share/common-lisp/ros/baxter_search")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/cc/ee106a/fl21/class/ee106a-aes/Desktop/eecs106a_final_project/devel/share/gennodejs/ros/baxter_search")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/cc/ee106a/fl21/class/ee106a-aes/Desktop/eecs106a_final_project/devel/lib/python2.7/dist-packages/baxter_search")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/cc/ee106a/fl21/class/ee106a-aes/Desktop/eecs106a_final_project/devel/lib/python2.7/dist-packages/baxter_search")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/cc/ee106a/fl21/class/ee106a-aes/Desktop/eecs106a_final_project/build/baxter_search/catkin_generated/installspace/baxter_search.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/baxter_search/cmake" TYPE FILE FILES "/home/cc/ee106a/fl21/class/ee106a-aes/Desktop/eecs106a_final_project/build/baxter_search/catkin_generated/installspace/baxter_search-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/baxter_search/cmake" TYPE FILE FILES
    "/home/cc/ee106a/fl21/class/ee106a-aes/Desktop/eecs106a_final_project/build/baxter_search/catkin_generated/installspace/baxter_searchConfig.cmake"
    "/home/cc/ee106a/fl21/class/ee106a-aes/Desktop/eecs106a_final_project/build/baxter_search/catkin_generated/installspace/baxter_searchConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/baxter_search" TYPE FILE FILES "/home/cc/ee106a/fl21/class/ee106a-aes/Desktop/eecs106a_final_project/src/baxter_search/package.xml")
endif()

