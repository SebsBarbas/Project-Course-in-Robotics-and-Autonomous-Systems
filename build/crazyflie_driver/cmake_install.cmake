# Install script for directory: /home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_driver

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/install")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/install" TYPE PROGRAM FILES "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/build/crazyflie_driver/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/install" TYPE PROGRAM FILES "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/build/crazyflie_driver/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/install/setup.bash;/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/install" TYPE FILE FILES
    "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/build/crazyflie_driver/catkin_generated/installspace/setup.bash"
    "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/build/crazyflie_driver/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/install/setup.sh;/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/install" TYPE FILE FILES
    "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/build/crazyflie_driver/catkin_generated/installspace/setup.sh"
    "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/build/crazyflie_driver/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/install/setup.zsh;/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/install" TYPE FILE FILES
    "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/build/crazyflie_driver/catkin_generated/installspace/setup.zsh"
    "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/build/crazyflie_driver/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/install" TYPE FILE FILES "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/build/crazyflie_driver/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/crazyflie_driver/srv" TYPE FILE FILES
    "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_driver/srv/AddCrazyflie.srv"
    "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_driver/srv/GoTo.srv"
    "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_driver/srv/Land.srv"
    "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_driver/srv/RemoveCrazyflie.srv"
    "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_driver/srv/SetGroupMask.srv"
    "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_driver/srv/StartTrajectory.srv"
    "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_driver/srv/Stop.srv"
    "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_driver/srv/Takeoff.srv"
    "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_driver/srv/UpdateParams.srv"
    "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_driver/srv/UploadTrajectory.srv"
    "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_driver/srv/sendPacket.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/crazyflie_driver/msg" TYPE FILE FILES
    "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_driver/msg/LogBlock.msg"
    "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_driver/msg/GenericLogData.msg"
    "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_driver/msg/FullState.msg"
    "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_driver/msg/TrajectoryPolynomialPiece.msg"
    "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_driver/msg/crtpPacket.msg"
    "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_driver/msg/Hover.msg"
    "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_driver/msg/Position.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/crazyflie_driver/cmake" TYPE FILE FILES "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/build/crazyflie_driver/catkin_generated/installspace/crazyflie_driver-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/devel/.private/crazyflie_driver/include/crazyflie_driver")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/devel/.private/crazyflie_driver/share/roseus/ros/crazyflie_driver")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/devel/.private/crazyflie_driver/share/common-lisp/ros/crazyflie_driver")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/devel/.private/crazyflie_driver/share/gennodejs/ros/crazyflie_driver")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/devel/.private/crazyflie_driver/lib/python2.7/dist-packages/crazyflie_driver")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/devel/.private/crazyflie_driver/lib/python2.7/dist-packages/crazyflie_driver")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/build/crazyflie_driver/catkin_generated/installspace/crazyflie_driver.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/crazyflie_driver/cmake" TYPE FILE FILES "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/build/crazyflie_driver/catkin_generated/installspace/crazyflie_driver-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/crazyflie_driver/cmake" TYPE FILE FILES
    "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/build/crazyflie_driver/catkin_generated/installspace/crazyflie_driverConfig.cmake"
    "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/build/crazyflie_driver/catkin_generated/installspace/crazyflie_driverConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/crazyflie_driver" TYPE FILE FILES "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/course_packages/crazyflie_ros/crazyflie_driver/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/build/crazyflie_driver/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/build/crazyflie_driver/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
