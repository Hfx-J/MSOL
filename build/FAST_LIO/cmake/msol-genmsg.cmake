# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "msol: 1 messages, 0 services")

set(MSG_I_FLAGS "-Imsol:/home/beabbit/study_space/HL_BPR/MSOL/src/FAST_LIO/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(msol_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/beabbit/study_space/HL_BPR/MSOL/src/FAST_LIO/msg/Pose6D.msg" NAME_WE)
add_custom_target(_msol_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msol" "/home/beabbit/study_space/HL_BPR/MSOL/src/FAST_LIO/msg/Pose6D.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(msol
  "/home/beabbit/study_space/HL_BPR/MSOL/src/FAST_LIO/msg/Pose6D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msol
)

### Generating Services

### Generating Module File
_generate_module_cpp(msol
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msol
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(msol_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(msol_generate_messages msol_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/beabbit/study_space/HL_BPR/MSOL/src/FAST_LIO/msg/Pose6D.msg" NAME_WE)
add_dependencies(msol_generate_messages_cpp _msol_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(msol_gencpp)
add_dependencies(msol_gencpp msol_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS msol_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(msol
  "/home/beabbit/study_space/HL_BPR/MSOL/src/FAST_LIO/msg/Pose6D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msol
)

### Generating Services

### Generating Module File
_generate_module_eus(msol
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msol
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(msol_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(msol_generate_messages msol_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/beabbit/study_space/HL_BPR/MSOL/src/FAST_LIO/msg/Pose6D.msg" NAME_WE)
add_dependencies(msol_generate_messages_eus _msol_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(msol_geneus)
add_dependencies(msol_geneus msol_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS msol_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(msol
  "/home/beabbit/study_space/HL_BPR/MSOL/src/FAST_LIO/msg/Pose6D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msol
)

### Generating Services

### Generating Module File
_generate_module_lisp(msol
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msol
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(msol_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(msol_generate_messages msol_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/beabbit/study_space/HL_BPR/MSOL/src/FAST_LIO/msg/Pose6D.msg" NAME_WE)
add_dependencies(msol_generate_messages_lisp _msol_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(msol_genlisp)
add_dependencies(msol_genlisp msol_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS msol_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(msol
  "/home/beabbit/study_space/HL_BPR/MSOL/src/FAST_LIO/msg/Pose6D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msol
)

### Generating Services

### Generating Module File
_generate_module_nodejs(msol
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msol
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(msol_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(msol_generate_messages msol_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/beabbit/study_space/HL_BPR/MSOL/src/FAST_LIO/msg/Pose6D.msg" NAME_WE)
add_dependencies(msol_generate_messages_nodejs _msol_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(msol_gennodejs)
add_dependencies(msol_gennodejs msol_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS msol_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(msol
  "/home/beabbit/study_space/HL_BPR/MSOL/src/FAST_LIO/msg/Pose6D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msol
)

### Generating Services

### Generating Module File
_generate_module_py(msol
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msol
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(msol_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(msol_generate_messages msol_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/beabbit/study_space/HL_BPR/MSOL/src/FAST_LIO/msg/Pose6D.msg" NAME_WE)
add_dependencies(msol_generate_messages_py _msol_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(msol_genpy)
add_dependencies(msol_genpy msol_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS msol_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msol)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msol
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(msol_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msol)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msol
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(msol_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msol)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msol
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(msol_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msol)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msol
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(msol_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msol)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msol\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msol
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(msol_generate_messages_py geometry_msgs_generate_messages_py)
endif()
