cmake_minimum_required(VERSION 3.10)
project(centerized_form)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_frontier_exploration
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs  
  tf
  
)
find_package(Eigen3 REQUIRED )

LINK_DIRECTORIES( ${ipopt_LIBRARY_DIRS} )
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

SET(source_list
	src/main.cpp
	src/batch_solver.cpp
	
)
#
ADD_EXECUTABLE(centerized_form ${source_list})
#
# libraries to be linked into the specified target,
# as determined by pkg-config for ipopt
TARGET_LINK_LIBRARIES(centerized_form
    ${cppad_lib}
    /usr/local/lib/libipopt.so
    ${colpack_libs}
    ${catkin_LIBRARIES}
)


