# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/course_packages/sim_cf/crazyflie_gazebo

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/build/crazyflie_gazebo

# Utility rule file for doxygen.

# Include the progress variables for this target.
include external/crazyflie_comm/CMakeFiles/doxygen.dir/progress.make

doxygen: external/crazyflie_comm/CMakeFiles/doxygen.dir/build.make

.PHONY : doxygen

# Rule to build all files generated by this target.
external/crazyflie_comm/CMakeFiles/doxygen.dir/build: doxygen

.PHONY : external/crazyflie_comm/CMakeFiles/doxygen.dir/build

external/crazyflie_comm/CMakeFiles/doxygen.dir/clean:
	cd /home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/build/crazyflie_gazebo/external/crazyflie_comm && $(CMAKE_COMMAND) -P CMakeFiles/doxygen.dir/cmake_clean.cmake
.PHONY : external/crazyflie_comm/CMakeFiles/doxygen.dir/clean

external/crazyflie_comm/CMakeFiles/doxygen.dir/depend:
	cd /home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/build/crazyflie_gazebo && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/course_packages/sim_cf/crazyflie_gazebo /home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/course_packages/sim_cf/crazyflie_gazebo/external/crazyflie_comm /home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/build/crazyflie_gazebo /home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/build/crazyflie_gazebo/external/crazyflie_comm /home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/build/crazyflie_gazebo/external/crazyflie_comm/CMakeFiles/doxygen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : external/crazyflie_comm/CMakeFiles/doxygen.dir/depend

