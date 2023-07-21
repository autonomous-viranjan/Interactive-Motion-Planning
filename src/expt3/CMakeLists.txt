cmake_minimum_required(VERSION 3.0.2)
project(expt3)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
)

# Gurobi deps
set(GUROBI_FOLDER "/home/optimal-student/vb/gurobi10.0.1_linux64/gurobi1001/linux64")
set(GUROBI_LIB_DIRS ${GUROBI_FOLDER}/lib)  
set(GUROBI_LIBS gurobi_g++5.2 gurobi100)
set(GUROBI_INCLUDE_DIRS ${GUROBI_FOLDER}/include)

# Required sources
set(planner3_SOURCES src/jointmpc.cpp)

# generate_messages(
#   DEPENDENCIES
#   geometry_msgs
# )

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
add_library(planner3_LIB ${planner3_SOURCES})

## C++
add_executable(planner3 src/planner3.cpp)
target_link_libraries(planner3 ${catkin_LIBRARIES} planner3_LIB ${GUROBI_LIBS})
# add_dependencies(planner3 expt3_generate_messages_cpp)

## Python
catkin_install_python(PROGRAMS
  src/ego_vehicle.py  
  src/NV.py
  src/stopped_vehicle.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)