cmake_minimum_required(VERSION 2.8.3)
project(laserScan_Similarity)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  tf
  tf_conversions
  libpointmatcher_ros
)

find_package(libpointmatcher REQUIRED)

add_definitions(-std=c++11 -O2 -g -DNDEBUG)

catkin_package (
#	CATKIN_DEPENDS roscpp
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${libpointmatcher_INCLUDE_DIRS}
)

add_executable(laserScan_Similarity src/laserScan_Similarity.cpp)
target_link_libraries(laserScan_Similarity 
${catkin_LIBRARIES}
${libpointmatcher_LIBRARIES}
)

add_executable(laserScan_Similarity_Continuous src/laserScan_Similarity_Continuous.cpp)
target_link_libraries(laserScan_Similarity_Continuous 
${catkin_LIBRARIES}
${libpointmatcher_LIBRARIES}
)

add_executable(laserScan_Similarity_Rings_Continuous src/laserScan_Similarity_Rings_Continuous.cpp)
target_link_libraries(laserScan_Similarity_Rings_Continuous 
${catkin_LIBRARIES}
${libpointmatcher_LIBRARIES}
)
