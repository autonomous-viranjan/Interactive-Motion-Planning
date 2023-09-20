cmake_minimum_required(VERSION 3.0.2)
project(expt4)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
)

# Gurobi deps
set(GUROBI_FOLDER "/home/emc2/vb/gurobi10.0.3_linux64/gurobi1003/linux64")
set(GUROBI_LIB_DIRS ${GUROBI_FOLDER}/lib)  
set(GUROBI_LIBS gurobi_g++5.2 gurobi100)
set(GUROBI_INCLUDE_DIRS ${GUROBI_FOLDER}/include)

# Required sources
set(planner4_SOURCES src/jointmpc.cpp src/imputer.cpp)

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
add_library(planner4_LIB ${planner4_SOURCES})

## C++
add_executable(planner4 src/planner4.cpp)
target_link_libraries(planner4 ${catkin_LIBRARIES} planner4_LIB ${GUROBI_LIBS})
# add_dependencies(planner3 expt3_generate_messages_cpp)

## Python
catkin_install_python(PROGRAMS
  src/ego_vehicle.py  
  src/NV.py
  src/stopped_vehicle.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)