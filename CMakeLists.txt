# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(localization)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg tf)

## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES xy_vector.msg scan_vector.msg scan_point.msg xy_point.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

##add executables

add_executable(locate src/locate.cpp)
target_link_libraries(locate ${catkin_LIBRARIES})
add_dependencies(locate locate_gencpp)

add_executable(fake_scan src/fake_scan.cpp)
target_link_libraries(fake_scan ${catkin_LIBRARIES})
add_dependencies(fake_scan fake_scan_gencpp)

## Declare a catkin package
catkin_package()

# %EndTag(FULLTEXT)%