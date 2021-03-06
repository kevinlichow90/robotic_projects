cmake_minimum_required(VERSION 2.8.3)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${geometry_msgs_INCLUDE_DIRS})

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(DifferentialDrivePlugin SHARED src/DifferentialDrivePlugin.cpp)
target_link_libraries(DifferentialDrivePlugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})
add_library(SpheroDrivePlugin SHARED src/SpheroDrivePlugin.cpp)
target_link_libraries(SpheroDrivePlugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

install(TARGETS DifferentialDrivePlugin SpheroDrivePlugin DESTINATION ~/catkin_ws/src/gazebo_plugins/lib)
