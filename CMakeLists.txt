cmake_minimum_required(VERSION 2.8.3)
project(rover_control)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs)
catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
endif()

include_directories(${catkin_INCLUDE_DIRS})

install(DIRECTORY config meshes models launch src scripts urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(rover_keyboard_control src/rover_keyboard_control.cpp)
target_link_libraries(rover_keyboard_control ${catkin_LIBRARIES})
