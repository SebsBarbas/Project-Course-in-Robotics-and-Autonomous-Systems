# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "milestone2: 0 messages, 1 services")

set(MSG_I_FLAGS "-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg;-Inav_msgs:/opt/ros/melodic/share/nav_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(milestone2_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/milestone2/srv/pathPlanning.srv" NAME_WE)
add_custom_target(_milestone2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "milestone2" "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/milestone2/srv/pathPlanning.srv" "nav_msgs/Path:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Point:geometry_msgs/PoseStamped:geometry_msgs/Quaternion"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(milestone2
  "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/milestone2/srv/pathPlanning.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/milestone2
)

### Generating Module File
_generate_module_cpp(milestone2
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/milestone2
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(milestone2_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(milestone2_generate_messages milestone2_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/milestone2/srv/pathPlanning.srv" NAME_WE)
add_dependencies(milestone2_generate_messages_cpp _milestone2_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(milestone2_gencpp)
add_dependencies(milestone2_gencpp milestone2_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS milestone2_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(milestone2
  "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/milestone2/srv/pathPlanning.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/milestone2
)

### Generating Module File
_generate_module_eus(milestone2
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/milestone2
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(milestone2_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(milestone2_generate_messages milestone2_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/milestone2/srv/pathPlanning.srv" NAME_WE)
add_dependencies(milestone2_generate_messages_eus _milestone2_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(milestone2_geneus)
add_dependencies(milestone2_geneus milestone2_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS milestone2_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(milestone2
  "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/milestone2/srv/pathPlanning.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/milestone2
)

### Generating Module File
_generate_module_lisp(milestone2
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/milestone2
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(milestone2_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(milestone2_generate_messages milestone2_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/milestone2/srv/pathPlanning.srv" NAME_WE)
add_dependencies(milestone2_generate_messages_lisp _milestone2_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(milestone2_genlisp)
add_dependencies(milestone2_genlisp milestone2_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS milestone2_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(milestone2
  "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/milestone2/srv/pathPlanning.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/milestone2
)

### Generating Module File
_generate_module_nodejs(milestone2
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/milestone2
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(milestone2_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(milestone2_generate_messages milestone2_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/milestone2/srv/pathPlanning.srv" NAME_WE)
add_dependencies(milestone2_generate_messages_nodejs _milestone2_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(milestone2_gennodejs)
add_dependencies(milestone2_gennodejs milestone2_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS milestone2_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(milestone2
  "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/milestone2/srv/pathPlanning.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/milestone2
)

### Generating Module File
_generate_module_py(milestone2
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/milestone2
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(milestone2_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(milestone2_generate_messages milestone2_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/milestone2/srv/pathPlanning.srv" NAME_WE)
add_dependencies(milestone2_generate_messages_py _milestone2_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(milestone2_genpy)
add_dependencies(milestone2_genpy milestone2_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS milestone2_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/milestone2)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/milestone2
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(milestone2_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(milestone2_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(milestone2_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(milestone2_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/milestone2)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/milestone2
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(milestone2_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(milestone2_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(milestone2_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(milestone2_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/milestone2)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/milestone2
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(milestone2_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(milestone2_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(milestone2_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(milestone2_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/milestone2)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/milestone2
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(milestone2_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(milestone2_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(milestone2_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(milestone2_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/milestone2)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/milestone2\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/milestone2
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(milestone2_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(milestone2_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(milestone2_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(milestone2_generate_messages_py std_msgs_generate_messages_py)
endif()
