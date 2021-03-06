cmake_minimum_required(VERSION 2.8.3)
project(ur_arm)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  std_srvs
  message_generation

  actionlib
  control_msgs
  sensor_msgs
  tf
  trac_ik_lib
  trajectory_msgs

  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface

  cv_bridge
  image_transport
)
find_package(OpenCV REQUIRED)
find_package(VISP REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

add_message_files(
  FILES
  Joints.msg
  PoseMatrix.msg
  AllAng.msg
  cartesianState.msg
)
add_service_files(
  FILES
  SetDigitalOut.srv
)

generate_messages(DEPENDENCIES std_msgs)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES ur_arm
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
   CATKIN_DEPENDS geometry_msgs roscpp std_msgs std_srvs message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ur_arm.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/ur_arm_node.cpp)


add_executable(
  node
  src/main.cpp
  src/ur_arm.cpp
  src/ur_arm_forkin.cpp
  src/ur_arm_node.cpp
  include/ur_arm/pose.h
  include/ur_arm/ur_arm.h
  include/ur_arm/ur_arm_forkin.h
  include/ur_arm/ur_arm_node.h
)
add_executable(
  vel_control
  src/vel_control.cpp
)
add_executable(
  collision_detect
  src/collision_detect.cpp
)
add_executable(
  grinder
  src/grinder.cpp
)
add_executable(
  dataRecorder
  src/dataRecorder.cpp
)
add_executable(
  parameterEstimate
  src/parameterEstimate.cpp
)
add_executable(
  collision_detect_simple
  src/collision_detect_simple.cpp
)
add_executable(
  grinder_newmethod
  src/grinder_newmethod.cpp
)
add_executable(
  zxw_learningNoCollision
  src/zxw_learningNoCollision.cpp
)
add_executable(
  getSensorData
  src/getSensorData.cpp
)
add_executable(
  grindBySensor
  src/grindBySensor.cpp
)
add_executable(
  fourPointsExploration
  src/fourPointsExploration.cpp
)
add_executable(
  dmpGrinding
  src/dmpGrinding.cpp
)
add_executable(
  circleGoalDetect
  src/circleGoalDetect.cpp
)
add_executable(
  minitest
  src/minitest.cpp
)
add_executable(
  dmpMoving
  src/dmpMoving.cpp
)
add_executable(
  dmp_pre_grinding
  src/dmp_pre_grinding.cpp
)

target_link_libraries(
  node
  ${catkin_LIBRARIES}
)
target_link_libraries(
  vel_control
  ${catkin_LIBRARIES}
)
target_link_libraries(
  collision_detect
  ${catkin_LIBRARIES}
)
target_link_libraries(
  grinder
  ${catkin_LIBRARIES}
)
target_link_libraries(
  dataRecorder
  ${catkin_LIBRARIES}
)
target_link_libraries(
  parameterEstimate
  ${catkin_LIBRARIES}
)
target_link_libraries(
  collision_detect_simple
  ${catkin_LIBRARIES}
)
target_link_libraries(
  grinder_newmethod
  ${catkin_LIBRARIES}
)
target_link_libraries(
  zxw_learningNoCollision
  ${catkin_LIBRARIES}
)
target_link_libraries(
  getSensorData
  ${catkin_LIBRARIES}
)
target_link_libraries(
  grindBySensor
  ${catkin_LIBRARIES}
)
target_link_libraries(
  fourPointsExploration
  ${catkin_LIBRARIES}
)
target_link_libraries(
  dmpGrinding
  ${catkin_LIBRARIES}
)
target_link_libraries(
  circleGoalDetect
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
target_link_libraries(
  minitest
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
target_link_libraries(
  dmpMoving
  ${catkin_LIBRARIES}
)
target_link_libraries(
  dmp_pre_grinding
  ${catkin_LIBRARIES}
)

add_dependencies(
  node
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(
  vel_control
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(
  collision_detect
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(
  grinder
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(
  dataRecorder
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(
  parameterEstimate
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(
  collision_detect_simple
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(
  grinder_newmethod
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(
  zxw_learningNoCollision
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(
  getSensorData
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(
  grindBySensor
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(
  fourPointsExploration
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(
  dmpGrinding
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(
  circleGoalDetect
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(
  minitest
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(
  dmpMoving
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(
  dmp_pre_grinding
  ${catkin_EXPORTED_TARGETS}
)
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ur_arm.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
