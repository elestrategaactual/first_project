cmake_minimum_required(VERSION 3.0.2)
project(first_project)



find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs nav_msgs message_generation tf)


## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Odom.msg
 )


## Generate services in the 'srv' folder

  add_service_files(
    FILES
    reset_odom.srv
  )

## Declare a catkin package


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  first_project
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs) 


## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odom_node src/odom_node.cpp)
add_dependencies(odom_node ${odom_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(odom_node custom_messages_generate_messages_cpp)
target_link_libraries(odom_node ${catkin_LIBRARIES})


