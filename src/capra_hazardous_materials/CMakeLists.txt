
cmake_minimum_required(VERSION 3.5)
project(capra_hazardous_materials_detection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
 cv_bridge
 image_geometry
 roscpp
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
# include
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
add_executable(${PROJECT_NAME}
   src/DisplayImage.cpp
)

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)