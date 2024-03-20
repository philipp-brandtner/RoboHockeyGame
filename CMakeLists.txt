cmake_minimum_required(VERSION 2.8.3)
project(gruppe6)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  image_transport
  sensor_msgs
  cv_bridge
  geometry_msgs
  pcl_ros
  message_generation
)

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
add_message_files(
   FILES
   Puck.msg
   Pole.msg	
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
#   TeamColor.srv
   CatchStart.srv
   CatchStop.srv
   CatchStatus.srv
   DropStart.srv
   DropStop.srv
   DropStatus.srv
   CheckPuck.srv
   srvPoleDetection.srv
   srvPuckDetectionBlue.srv
   srvPuckDetectionYellow.srv
   srvRectangleDetection.srv
   PuckFinderStart.srv
   PuckFinderStop.srv
   PuckFinderStatus.srv
   PuckMoverStart.srv
   PuckMoverStop.srv
   PuckMoverStatus.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

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
#  INCLUDE_DIRS include
#  LIBRARIES my_turtle
   CATKIN_DEPENDS message_runtime #roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(my_turtle
#   src/${PROJECT_NAME}/my_turtle.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(my_turtle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(PuckDetectionBlue src/ImageProcessing/DetectionBluePuck/run_PuckDetection.cpp src/ImageProcessing/DetectionBluePuck/PuckDetection.cpp)

add_executable(PuckDetectionYellow src/ImageProcessing/DetectionYellowPuck/run_PuckDetection.cpp src/ImageProcessing/DetectionYellowPuck/PuckDetection.cpp)

add_executable(PoleDetectionGreen src/ImageProcessing/DetectionGreenPole/run_PoleDetection.cpp src/ImageProcessing/DetectionGreenPole/PoleDetection.cpp)

add_executable(RectangleDetection src/ImageProcessing/DetectionRectangle/run_RectangleDetection.cpp src/ImageProcessing/DetectionRectangle/RectangleDetection.cpp)

add_executable(LocalNavigation src/Nav_Local/run_nav_local.cpp src/Nav_Local/nav_local.cpp)


#add_executable(pointcloud_processing src/ImageProcessing/Pointcloud/run_pointcloudProcessing.cpp #src/ImageProcessing/Pointcloud/pointcloud.cpp)
#add_executable(webcam_publisher src/ImageProcessing/webcam_publisher.cpp)
#add_executable(pointcloud src/pointcloud/pointcloud.cpp)

add_executable(puck_mover_node src/Nav_Global/puck_mover_node.cpp src/Nav_Global/puck_mover.cpp)
add_executable(puck_finder_node src/Nav_Global/puck_finder_node.cpp src/Nav_Global/puck_finder.cpp)
add_executable(finderStart_test src/Nav_Global/Tests/FinderStartTest.cpp)
add_executable(finderStop_test src/Nav_Global/Tests/FinderStopTest.cpp)
add_executable(finderStatus_test src/Nav_Global/Tests/FinderStatusTest.cpp)
add_executable(moverStart_test src/Nav_Global/Tests/MoverStartTest.cpp)
add_executable(moverStatus_test src/Nav_Global/Tests/MoverStatusTest.cpp)
add_executable(moverStop_test src/Nav_Global/Tests/MoverStopTest.cpp)
add_executable(findMove_test src/Nav_Global/Tests/FindMoveTest.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(my_turtle_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(PuckDetectionYellow gruppe6_generate_messages_cpp)
add_dependencies(PuckDetectionBlue gruppe6_generate_messages_cpp)
add_dependencies(PoleDetectionGreen gruppe6_generate_messages_cpp)
#add_dependencies(CatchPuck gruppe6_generate_messages_cpp)
add_dependencies(LocalNavigation gruppe6_gencpp)
add_dependencies(RectangleDetection gruppe6_gencpp)
add_dependencies(puck_finder_node gruppe6_gencpp)
add_dependencies(puck_mover_node gruppe6_gencpp)
## Specify libraries to link a library or executable target against
target_link_libraries(PuckDetectionBlue
  ${catkin_LIBRARIES}
)
target_link_libraries(PuckDetectionYellow
  ${catkin_LIBRARIES}
)

target_link_libraries(PoleDetectionGreen
  ${catkin_LIBRARIES}
)
target_link_libraries(RectangleDetection
  ${catkin_LIBRARIES}
)


target_link_libraries(LocalNavigation
  ${catkin_LIBRARIES}
)

#target_link_libraries(webcam_publisher
#  ${catkin_LIBRARIES}
#)
#target_link_libraries(pointcloud
# ${catkin_LIBRARIES}
#)

target_link_libraries(puck_mover_node ${catkin_LIBRARIES})
target_link_libraries(puck_finder_node ${catkin_LIBRARIES})
target_link_libraries(finderStart_test ${catkin_LIBRARIES})
target_link_libraries(finderStop_test ${catkin_LIBRARIES})
target_link_libraries(finderStatus_test ${catkin_LIBRARIES})
target_link_libraries(moverStart_test ${catkin_LIBRARIES})
target_link_libraries(moverStop_test ${catkin_LIBRARIES})
target_link_libraries(moverStatus_test ${catkin_LIBRARIES})
target_link_libraries(findMove_test ${catkin_LIBRARIES})

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
# install(TARGETS my_turtle my_turtle_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_my_turtle.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
