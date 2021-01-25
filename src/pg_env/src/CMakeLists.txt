cmake_minimum_required(VERSION 3.0.2)
project(my_pkg)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs std_srvs message_generation)

include_directories(include ${catkin_INCLUDE_DIRS})

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs std_srvs)

## Declare a catkin package
catkin_package()

## listener is the node name
add_executable(client src/client.cpp)
target_link_libraries(client ${catkin_LIBRARIES})
