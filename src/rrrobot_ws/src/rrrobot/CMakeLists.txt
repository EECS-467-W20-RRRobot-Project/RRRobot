cmake_minimum_required(VERSION 2.8.3)
project(rrrobot)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  osrf_gear
  roscpp
  sensor_msgs
  std_srvs
  trajectory_msgs
  std_msgs
  geometry_msgs
  gazebo_ros
  message_generation
  moveit_ros_planning_interface
)
find_package(gazebo REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  model_insertion.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
#  LIBRARIES simulation_env
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs gazebo_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(insert_model SHARED
  src/model_insertion_plugin.cpp
)
add_dependencies(insert_model ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Declare a C++ executable
# add_executable(rrrobot_node src/rrrobot_node.cpp)
# add_dependencies(rrrobot_node ${catkin_EXPORTED_TARGETS})
# target_link_libraries(rrrobot_node ${catkin_LIBRARIES})

add_executable(arm_controller_node src/arm_controller_node.cpp)
add_dependencies(arm_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_controller_node ${catkin_LIBRARIES})

add_executable(test_insert_object test/test_insert_object.cpp)
add_executable(object_spawner_node src/object_spawner_node.cpp)

add_dependencies(test_insert_object insert_model)
add_dependencies(object_spawner_node insert_model)

target_link_libraries(test_insert_object
  ${catkin_LIBRARIES}
)
target_link_libraries(object_spawner_node
  ${catkin_LIBRARIES}
)

target_link_libraries(insert_model
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
# install(PROGRAMS
#   script/rrrobot_node.py
#   script/tf2_example.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS rrrobot_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(TARGETS arm_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#  config/sample_gear_conf.yaml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ariac_example.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)