# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(laser_loc)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES xy_vector.msg scan_vector.msg scan_point.msg xy_cords.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

##add executables

add_executable(bot_loc src/bot_loc.cpp)
target_link_libraries(bot_loc ${catkin_LIBRARIES})
add_dependencies(bot_loc laser_loc_gencpp)

add_executable(get_poles_test src/get_poles_test.cpp)
target_link_libraries(get_poles_test ${catkin_LIBRARIES})
add_dependencies(get_poles_test laser_loc_gencpp)

add_executable(get_poles src/get_poles.cpp)
target_link_libraries(get_poles ${catkin_LIBRARIES})
add_dependencies(get_poles laser_loc_gencpp)

add_executable(fake_scan src/fake_scan.cpp)
target_link_libraries(fake_scan ${catkin_LIBRARIES})
add_dependencies(fake_scan laser_loc_gencpp)

add_executable(init_poles src/init_poles.cpp)
target_link_libraries(init_poles ${catkin_LIBRARIES})
add_dependencies(init_poles laser_loc_gencpp)

## Declare a catkin package
catkin_package()

# %EndTag(FULLTEXT)%