# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "object_detection_msgs: 5 messages, 0 services")

set(MSG_I_FLAGS "-Iobject_detection_msgs:/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(object_detection_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/BoundingBox.msg" NAME_WE)
add_custom_target(_object_detection_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "object_detection_msgs" "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/BoundingBox.msg" ""
)

get_filename_component(_filename "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/BoundingBoxes.msg" NAME_WE)
add_custom_target(_object_detection_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "object_detection_msgs" "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/BoundingBoxes.msg" "std_msgs/Header:object_detection_msgs/BoundingBox"
)

get_filename_component(_filename "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/ObjectCount.msg" NAME_WE)
add_custom_target(_object_detection_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "object_detection_msgs" "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/ObjectCount.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/CarPosition.msg" NAME_WE)
add_custom_target(_object_detection_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "object_detection_msgs" "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/CarPosition.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/motorAngle.msg" NAME_WE)
add_custom_target(_object_detection_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "object_detection_msgs" "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/motorAngle.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(object_detection_msgs
  "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/object_detection_msgs
)
_generate_msg_cpp(object_detection_msgs
  "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/BoundingBoxes.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/BoundingBox.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/object_detection_msgs
)
_generate_msg_cpp(object_detection_msgs
  "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/ObjectCount.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/object_detection_msgs
)
_generate_msg_cpp(object_detection_msgs
  "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/CarPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/object_detection_msgs
)
_generate_msg_cpp(object_detection_msgs
  "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/motorAngle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/object_detection_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(object_detection_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/object_detection_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(object_detection_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(object_detection_msgs_generate_messages object_detection_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/BoundingBox.msg" NAME_WE)
add_dependencies(object_detection_msgs_generate_messages_cpp _object_detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/BoundingBoxes.msg" NAME_WE)
add_dependencies(object_detection_msgs_generate_messages_cpp _object_detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/ObjectCount.msg" NAME_WE)
add_dependencies(object_detection_msgs_generate_messages_cpp _object_detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/CarPosition.msg" NAME_WE)
add_dependencies(object_detection_msgs_generate_messages_cpp _object_detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/motorAngle.msg" NAME_WE)
add_dependencies(object_detection_msgs_generate_messages_cpp _object_detection_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(object_detection_msgs_gencpp)
add_dependencies(object_detection_msgs_gencpp object_detection_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS object_detection_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(object_detection_msgs
  "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/object_detection_msgs
)
_generate_msg_eus(object_detection_msgs
  "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/BoundingBoxes.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/BoundingBox.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/object_detection_msgs
)
_generate_msg_eus(object_detection_msgs
  "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/ObjectCount.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/object_detection_msgs
)
_generate_msg_eus(object_detection_msgs
  "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/CarPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/object_detection_msgs
)
_generate_msg_eus(object_detection_msgs
  "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/motorAngle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/object_detection_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(object_detection_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/object_detection_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(object_detection_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(object_detection_msgs_generate_messages object_detection_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/BoundingBox.msg" NAME_WE)
add_dependencies(object_detection_msgs_generate_messages_eus _object_detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/BoundingBoxes.msg" NAME_WE)
add_dependencies(object_detection_msgs_generate_messages_eus _object_detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/ObjectCount.msg" NAME_WE)
add_dependencies(object_detection_msgs_generate_messages_eus _object_detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/CarPosition.msg" NAME_WE)
add_dependencies(object_detection_msgs_generate_messages_eus _object_detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/motorAngle.msg" NAME_WE)
add_dependencies(object_detection_msgs_generate_messages_eus _object_detection_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(object_detection_msgs_geneus)
add_dependencies(object_detection_msgs_geneus object_detection_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS object_detection_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(object_detection_msgs
  "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/object_detection_msgs
)
_generate_msg_lisp(object_detection_msgs
  "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/BoundingBoxes.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/BoundingBox.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/object_detection_msgs
)
_generate_msg_lisp(object_detection_msgs
  "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/ObjectCount.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/object_detection_msgs
)
_generate_msg_lisp(object_detection_msgs
  "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/CarPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/object_detection_msgs
)
_generate_msg_lisp(object_detection_msgs
  "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/motorAngle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/object_detection_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(object_detection_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/object_detection_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(object_detection_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(object_detection_msgs_generate_messages object_detection_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/BoundingBox.msg" NAME_WE)
add_dependencies(object_detection_msgs_generate_messages_lisp _object_detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/BoundingBoxes.msg" NAME_WE)
add_dependencies(object_detection_msgs_generate_messages_lisp _object_detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/ObjectCount.msg" NAME_WE)
add_dependencies(object_detection_msgs_generate_messages_lisp _object_detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/CarPosition.msg" NAME_WE)
add_dependencies(object_detection_msgs_generate_messages_lisp _object_detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/motorAngle.msg" NAME_WE)
add_dependencies(object_detection_msgs_generate_messages_lisp _object_detection_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(object_detection_msgs_genlisp)
add_dependencies(object_detection_msgs_genlisp object_detection_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS object_detection_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(object_detection_msgs
  "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/object_detection_msgs
)
_generate_msg_nodejs(object_detection_msgs
  "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/BoundingBoxes.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/BoundingBox.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/object_detection_msgs
)
_generate_msg_nodejs(object_detection_msgs
  "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/ObjectCount.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/object_detection_msgs
)
_generate_msg_nodejs(object_detection_msgs
  "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/CarPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/object_detection_msgs
)
_generate_msg_nodejs(object_detection_msgs
  "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/motorAngle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/object_detection_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(object_detection_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/object_detection_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(object_detection_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(object_detection_msgs_generate_messages object_detection_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/BoundingBox.msg" NAME_WE)
add_dependencies(object_detection_msgs_generate_messages_nodejs _object_detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/BoundingBoxes.msg" NAME_WE)
add_dependencies(object_detection_msgs_generate_messages_nodejs _object_detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/ObjectCount.msg" NAME_WE)
add_dependencies(object_detection_msgs_generate_messages_nodejs _object_detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/CarPosition.msg" NAME_WE)
add_dependencies(object_detection_msgs_generate_messages_nodejs _object_detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/motorAngle.msg" NAME_WE)
add_dependencies(object_detection_msgs_generate_messages_nodejs _object_detection_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(object_detection_msgs_gennodejs)
add_dependencies(object_detection_msgs_gennodejs object_detection_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS object_detection_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(object_detection_msgs
  "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/object_detection_msgs
)
_generate_msg_py(object_detection_msgs
  "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/BoundingBoxes.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/BoundingBox.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/object_detection_msgs
)
_generate_msg_py(object_detection_msgs
  "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/ObjectCount.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/object_detection_msgs
)
_generate_msg_py(object_detection_msgs
  "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/CarPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/object_detection_msgs
)
_generate_msg_py(object_detection_msgs
  "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/motorAngle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/object_detection_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(object_detection_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/object_detection_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(object_detection_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(object_detection_msgs_generate_messages object_detection_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/BoundingBox.msg" NAME_WE)
add_dependencies(object_detection_msgs_generate_messages_py _object_detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/BoundingBoxes.msg" NAME_WE)
add_dependencies(object_detection_msgs_generate_messages_py _object_detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/ObjectCount.msg" NAME_WE)
add_dependencies(object_detection_msgs_generate_messages_py _object_detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/CarPosition.msg" NAME_WE)
add_dependencies(object_detection_msgs_generate_messages_py _object_detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xia_s1r/Documents/FormaitionVSTracker/Formation-Tracker/src/object_detection_msgs/msg/motorAngle.msg" NAME_WE)
add_dependencies(object_detection_msgs_generate_messages_py _object_detection_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(object_detection_msgs_genpy)
add_dependencies(object_detection_msgs_genpy object_detection_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS object_detection_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/object_detection_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/object_detection_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(object_detection_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/object_detection_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/object_detection_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(object_detection_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/object_detection_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/object_detection_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(object_detection_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/object_detection_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/object_detection_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(object_detection_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/object_detection_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/object_detection_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/object_detection_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(object_detection_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
