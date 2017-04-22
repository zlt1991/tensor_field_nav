cmake_minimum_required(VERSION 2.8.3)
project(pure_pursuit_controller)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  actionlib
  move_base_msgs
  moveit_msgs
  moveit_ros_planning_interface
  message_generation
)

# Generate services in the 'srv' folder
 add_service_files(
   FILES
   executePath.srv
   cancelPath.srv
   cutPath.srv
   recoverPath.srv
 )


 generate_messages(
   DEPENDENCIES
   geometry_msgs #    nav_msgs
   nav_msgs
   std_msgs
 )



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pure_pursuit_controller
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs tf
  DEPENDS system_lib
)

include_directories(
    include
  ${catkin_INCLUDE_DIRS}
)


add_library(purePursuitController
   src/purePursuitController.cpp include/pure_pursuit_controller/purePursuitController.h
)

add_executable(pure_pursuit_controller_node src/pure_pursuit_controller_node.cpp)

target_link_libraries(pure_pursuit_controller_node purePursuitController
   ${catkin_LIBRARIES}
 )
