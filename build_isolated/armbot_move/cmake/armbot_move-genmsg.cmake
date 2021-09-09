# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "armbot_move: 1 messages, 1 services")

set(MSG_I_FLAGS "-Iarmbot_move:/home/e/ROS/Armbot/src/armbot_move/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(armbot_move_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/e/ROS/Armbot/src/armbot_move/msg/move_position.msg" NAME_WE)
add_custom_target(_armbot_move_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "armbot_move" "/home/e/ROS/Armbot/src/armbot_move/msg/move_position.msg" ""
)

get_filename_component(_filename "/home/e/ROS/Armbot/src/armbot_move/srv/SetPosition.srv" NAME_WE)
add_custom_target(_armbot_move_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "armbot_move" "/home/e/ROS/Armbot/src/armbot_move/srv/SetPosition.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(armbot_move
  "/home/e/ROS/Armbot/src/armbot_move/msg/move_position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/armbot_move
)

### Generating Services
_generate_srv_cpp(armbot_move
  "/home/e/ROS/Armbot/src/armbot_move/srv/SetPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/armbot_move
)

### Generating Module File
_generate_module_cpp(armbot_move
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/armbot_move
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(armbot_move_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(armbot_move_generate_messages armbot_move_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/e/ROS/Armbot/src/armbot_move/msg/move_position.msg" NAME_WE)
add_dependencies(armbot_move_generate_messages_cpp _armbot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/e/ROS/Armbot/src/armbot_move/srv/SetPosition.srv" NAME_WE)
add_dependencies(armbot_move_generate_messages_cpp _armbot_move_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(armbot_move_gencpp)
add_dependencies(armbot_move_gencpp armbot_move_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS armbot_move_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(armbot_move
  "/home/e/ROS/Armbot/src/armbot_move/msg/move_position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/armbot_move
)

### Generating Services
_generate_srv_eus(armbot_move
  "/home/e/ROS/Armbot/src/armbot_move/srv/SetPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/armbot_move
)

### Generating Module File
_generate_module_eus(armbot_move
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/armbot_move
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(armbot_move_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(armbot_move_generate_messages armbot_move_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/e/ROS/Armbot/src/armbot_move/msg/move_position.msg" NAME_WE)
add_dependencies(armbot_move_generate_messages_eus _armbot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/e/ROS/Armbot/src/armbot_move/srv/SetPosition.srv" NAME_WE)
add_dependencies(armbot_move_generate_messages_eus _armbot_move_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(armbot_move_geneus)
add_dependencies(armbot_move_geneus armbot_move_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS armbot_move_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(armbot_move
  "/home/e/ROS/Armbot/src/armbot_move/msg/move_position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/armbot_move
)

### Generating Services
_generate_srv_lisp(armbot_move
  "/home/e/ROS/Armbot/src/armbot_move/srv/SetPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/armbot_move
)

### Generating Module File
_generate_module_lisp(armbot_move
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/armbot_move
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(armbot_move_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(armbot_move_generate_messages armbot_move_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/e/ROS/Armbot/src/armbot_move/msg/move_position.msg" NAME_WE)
add_dependencies(armbot_move_generate_messages_lisp _armbot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/e/ROS/Armbot/src/armbot_move/srv/SetPosition.srv" NAME_WE)
add_dependencies(armbot_move_generate_messages_lisp _armbot_move_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(armbot_move_genlisp)
add_dependencies(armbot_move_genlisp armbot_move_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS armbot_move_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(armbot_move
  "/home/e/ROS/Armbot/src/armbot_move/msg/move_position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/armbot_move
)

### Generating Services
_generate_srv_nodejs(armbot_move
  "/home/e/ROS/Armbot/src/armbot_move/srv/SetPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/armbot_move
)

### Generating Module File
_generate_module_nodejs(armbot_move
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/armbot_move
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(armbot_move_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(armbot_move_generate_messages armbot_move_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/e/ROS/Armbot/src/armbot_move/msg/move_position.msg" NAME_WE)
add_dependencies(armbot_move_generate_messages_nodejs _armbot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/e/ROS/Armbot/src/armbot_move/srv/SetPosition.srv" NAME_WE)
add_dependencies(armbot_move_generate_messages_nodejs _armbot_move_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(armbot_move_gennodejs)
add_dependencies(armbot_move_gennodejs armbot_move_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS armbot_move_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(armbot_move
  "/home/e/ROS/Armbot/src/armbot_move/msg/move_position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/armbot_move
)

### Generating Services
_generate_srv_py(armbot_move
  "/home/e/ROS/Armbot/src/armbot_move/srv/SetPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/armbot_move
)

### Generating Module File
_generate_module_py(armbot_move
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/armbot_move
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(armbot_move_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(armbot_move_generate_messages armbot_move_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/e/ROS/Armbot/src/armbot_move/msg/move_position.msg" NAME_WE)
add_dependencies(armbot_move_generate_messages_py _armbot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/e/ROS/Armbot/src/armbot_move/srv/SetPosition.srv" NAME_WE)
add_dependencies(armbot_move_generate_messages_py _armbot_move_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(armbot_move_genpy)
add_dependencies(armbot_move_genpy armbot_move_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS armbot_move_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/armbot_move)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/armbot_move
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(armbot_move_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/armbot_move)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/armbot_move
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(armbot_move_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/armbot_move)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/armbot_move
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(armbot_move_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/armbot_move)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/armbot_move
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(armbot_move_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/armbot_move)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/armbot_move\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/armbot_move
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(armbot_move_generate_messages_py std_msgs_generate_messages_py)
endif()
