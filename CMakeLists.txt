# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(localization)

## Find catkin and any catkin packages
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs genmsg tf cmake_modules pluginlib laser_geometry)
find_package(TinyXML REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})

## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES xy_vector.msg scan_vector.msg scan_point.msg xy_point.msg beach_map.msg)
add_message_files(DIRECTORY include FILES IOFromBoard.msg)
add_service_files(DIRECTORY srv FILES InitLocalization.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

##add executables

add_executable(locate src/locate.cpp)
target_link_libraries(locate ${catkin_LIBRARIES} serial)
add_dependencies(locate locate_gencpp)

add_executable(fake_scan src/fake_scan.cpp)
target_link_libraries(fake_scan ${catkin_LIBRARIES})
add_dependencies(fake_scan locate_gencpp)

add_executable(error_checker src/error_checker.cpp)
target_link_libraries(error_checker ${catkin_LIBRARIES})
add_dependencies(error_checker locate_gencpp)

add_executable(intensity_test src/intensity_test.cpp)
target_link_libraries(intensity_test ${catkin_LIBRARIES})
add_dependencies(intensity_test locate_gencpp)

add_executable(laser_angle src/laser_angle.cpp)
target_link_libraries(laser_angle ${catkin_LIBRARIES})
add_dependencies(laser_angle locate_gencpp)

add_executable(output_simulator src/output_simulator.cpp)
target_link_libraries(output_simulator ${catkin_LIBRARIES})
add_dependencies(output_simulator locate_gencpp)

add_executable(laser_filter src/laser_filter.cpp)
target_link_libraries(laser_filter ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})
add_dependencies(laser_filter testing_gencpp)

## Declare a catkin package
catkin_package()

# %EndTag(FULLTEXT)%