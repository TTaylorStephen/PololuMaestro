cmake_minimum_required(VERSION 2.8.3)
project(science_node)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  std_msgs
  roscpp
)

catkin_package(

)
include_directories(${catkin_INCLUDE_DIRS})
add_executable(science_node src/armcode.cpp src/armcode.h)
add_dependencies(science_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(science_node ${catkin_LIBRARIES})



