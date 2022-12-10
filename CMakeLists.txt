cmake_minimum_required(VERSION 3.8)

project(irona)

## For msgs and srvs these dependencies are sufficient
find_package(catkin REQUIRED
            roscpp
            std_msgs
            geometry_msgs
            tf2_geometry_msgs
            move_base_msgs
            moveit_msgs
            moveit_ros_planning
            moveit_ros_planning_interface
            moveit_core
            actionlib
            actionlib_msgs
            message_generation
            gazebo_msgs
            )
find_package(Boost REQUIRED COMPONENTS system)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  DetectObject.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  CleanRoom.action
)

# generate_messages()

## If any messages, srvs, or actions have dependencies
## actions must have actionlib_msgs as dependency
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
CATKIN_DEPENDS message_runtime actionlib_msgs
)

include_directories(
include
${catkin_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
)

add_executable(ironagroup src/ironagroup.cpp src/ironamain.cpp)
target_link_libraries(ironagroup ${catkin_LIBRARIES})

add_dependencies(
  ironagroup
  ${irona_EXPORTED_TARGETS}
)

add_executable(detector  src/detectmain.cpp src/detectobject.cpp)
target_link_libraries(detector ${catkin_LIBRARIES})

add_dependencies(
  detector
  ${irona_EXPORTED_TARGETS}
)
