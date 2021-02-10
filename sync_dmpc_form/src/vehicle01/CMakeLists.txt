cmake_minimum_required(VERSION 3.10)
project(vehicle01)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs  
  tf
  message_generation
  mymsg
)
find_package(Eigen3 REQUIRED )




catkin_package(

CATKIN_DEPENDS roscpp message_runtime
)

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
ADD_EXECUTABLE(vehicle01 ${source_list})
add_dependencies(vehicle01 mymsg_gencpp)
#
# libraries to be linked into the specified target,
# as determined by pkg-config for ipopt
TARGET_LINK_LIBRARIES(vehicle01
    ${cppad_lib}
    /usr/local/lib/libipopt.so
    ${colpack_libs}
    ${catkin_LIBRARIES}
)


