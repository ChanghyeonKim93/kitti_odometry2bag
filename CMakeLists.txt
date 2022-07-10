cmake_minimum_required(VERSION 3.1.0)
project(kitti_odometry2bag)

find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  std_msgs  
  
  sensor_msgs
  geometry_msgs
)


catkin_package(
  DEPENDS Eigen
  CATKIN_DEPENDS roscpp message_runtime message_filters
)

LIST(APPEND LINK_LIBS
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES} 
  ${Eigen_LIBRARIES}  
)

set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR})
set(includeDirs ${includeDirs} include)

set(SRC ${SRC} 
    src/kitti_odometry2bag.cpp
    src/geometry_library.cpp)

include_directories( ${includeDirs} )
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories( ${Eigen_INCLUDE_DIRS} )
include_directories( ${Boost_INCLUDE_DIRS} )

set(CMAKE_build_type RELEASE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -pg -O2")

add_executable (kitti_odometry2bag src/main.cpp ${SRC})
target_link_libraries (kitti_odometry2bag ${LINK_LIBS})