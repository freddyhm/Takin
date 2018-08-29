cmake_minimum_required(VERSION 2.8.3)
project(capra_co2_detector)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ../../include/
)

add_executable(capra_co2_detector src/capra_co2_value_publisher.cpp)
target_link_libraries(capra_co2_detector ${catkin_LIBRARIES})
