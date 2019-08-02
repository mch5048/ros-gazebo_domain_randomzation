cmake_minimum_required(VERSION 2.8.3)
project(domain_randomization)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# Load catkin and all dependencies required for this package

# Depend on system install of Gazebo
find_package(gazebo REQUIRED
)


find_package(catkin REQUIRED COMPONENTS 
  geometry_msgs
  sensor_msgs
  std_msgs
  gazebo_msgs
  roscpp 
  gazebo_ros 
)

add_message_files( DIRECTORY msg
FILES
Tactile.msg
Contact.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_generation
  message_runtime
)


link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
include_directories(${ADDITIONAL_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS}/Paging)

# For dynamic_objects_simple_world_plugin plugin
# add_library(dynamic_objects_simple_world_plugin src/simple_world_plugin.cpp)
# target_link_libraries(dynamic_objects_simple_world_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# For dynamic_world_plugin plugin
add_library(dynamic_world_plugin src/dynamic_world_plugin.cpp)
add_library(pressure_plugin_ros src/pressure_plugin_ros.cc)
add_library(contact_plugin_ros src/ContactPlugin.cc)
target_link_libraries(dynamic_world_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(pressure_plugin_ros ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(contact_plugin_ros ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})




catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
)