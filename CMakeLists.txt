cmake_minimum_required(VERSION 2.8.3)
project(p3at_tutorial)

find_package(catkin REQUIRED COMPONENTS
  amcl
  base_local_planner
  move_base
  navfn
  roslaunch
  tf
  urdf
  xacro
  rospy
  roscpp
  sensor_msgs
)
catkin_package(
 INCLUDE_DIRS include
 # LIBRARIES radom_walk
 CATKIN_DEPENDS amcl base_local_planner base_local_planner move_base navfn roslaunch tf urdf xacro
 DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(random_walk src/random_walk.cpp)
add_executable(random_walk src/random_walk.cpp src/random_walk_node.cpp)
target_link_libraries(random_walk ${catkin_LIBRARIES})

add_executable(mapping src/mapping.cpp)
target_link_libraries(mapping ${catkin_LIBRARIES})

add_executable(exploration src/exploration.cpp)
target_link_libraries(exploration ${catkin_LIBRARIES})

# Declare a C++ executable
add_executable(pose_tf src/pose_tf.cpp)
target_link_libraries(pose_tf ${catkin_LIBRARIES})
