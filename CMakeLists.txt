# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(localization)

## Find catkin and any catkin packages
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs genmsg tf)
include_directories(${Eigen_INCLUDE_DIRS})

## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES xy_vector.msg scan_vector.msg scan_point.msg xy_point.msg beach_map.msg)
add_service_files(DIRECTORY srv FILES InitLocalization.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

##add executables

add_executable(locate src/locate.cpp)
target_link_libraries(locate ${catkin_LIBRARIES})
add_dependencies(locate locate_gencpp)

add_executable(fake_scan src/fake_scan.cpp)
target_link_libraries(fake_scan ${catkin_LIBRARIES})
add_dependencies(fake_scan fake_scan_gencpp)

add_executable(error_checker src/error_checker.cpp)
target_link_libraries(error_checker ${catkin_LIBRARIES})
add_dependencies(error_checker error_checker_gencpp)

add_executable(intensity_test src/intensity_test.cpp)
target_link_libraries(intensity_test ${catkin_LIBRARIES})
add_dependencies(intensity_test intensity_test_gencpp)

add_executable(laser_angle src/laser_angle.cpp)
target_link_libraries(laser_angle ${catkin_LIBRARIES})
add_dependencies(laser_angle laser_angle_gencpp)

## Declare a catkin package
catkin_package()

# %EndTag(FULLTEXT)%