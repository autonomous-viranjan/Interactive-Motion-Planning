cmake_minimum_required(VERSION 3.0.2)
project(expt1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  genmsg
)

## Gurobi deps
set(GUROBI_FOLDER "/home/emc2/vb/gurobi10.0.3_linux64/gurobi1003/linux64")
set(GUROBI_LIB_DIRS ${GUROBI_FOLDER}/lib)  
set(GUROBI_LIBS gurobi_g++5.2 gurobi100)
set(GUROBI_INCLUDE_DIRS ${GUROBI_FOLDER}/include)
# Required sources
set(planner1_SOURCES src/mpc.cpp)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
 CATKIN_DEPENDS geometry_msgs rospy roscpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS} include)
include_directories(${GUROBI_INCLUDE_DIRS})
link_directories(${GUROBI_LIB_DIRS})

## Declare a C++ library
add_library(planner1_LIB ${planner1_SOURCES})

## C++
add_executable(planner1 src/planner1.cpp)
target_link_libraries(planner1 ${catkin_LIBRARIES} planner1_LIB ${GUROBI_LIBS})
add_dependencies(planner1 expt1_generate_messages_cpp)

## Python
catkin_install_python(PROGRAMS
  src/ego_vehicle.py  
  src/NV.py
  src/stopped_vehicle.py
  src/NV_.py
  src/g920.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
