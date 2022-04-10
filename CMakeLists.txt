cmake_minimum_required(VERSION 3.0.2)
project(apa_planner)


set(CMAKE_CXX_STANDARD 11)

# 不调试的时候打开下面
# set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ") # -g
set (CMAKE_VERBOSE_MAKEFILE ON)

# 调试的时候打开下面
set(CMAKE_BUILD_TYPE "Debug")
IF(CMAKE_COMPILER_IS_GNUCC)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
ENDIF(CMAKE_COMPILER_IS_GNUCC)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  visualization_msgs
  cv_bridge
  tf
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

aux_source_directory(./src SRCS)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR})


catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS  PCL 
  INCLUDE_DIRS include
)

add_executable(${PROJECT_NAME}_node)
target_sources(${PROJECT_NAME}_node PUBLIC ${SRCS})
target_link_libraries( ${PROJECT_NAME}_node     ${catkin_LIBRARIES})


