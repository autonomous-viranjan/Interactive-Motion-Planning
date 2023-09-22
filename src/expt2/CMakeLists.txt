cmake_minimum_required(VERSION 3.2.2)
project(expt2)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  genmsg
)

set(GUROBI_FOLDER "/home/emc2/vb/gurobi10.0.3_linux64/gurobi1003/linux64")
set(GUROBI_LIB_DIRS ${GUROBI_FOLDER}/lib)  
set(GUROBI_LIBS gurobi_g++5.2 gurobi100)
set(GUROBI_INCLUDE_DIRS ${GUROBI_FOLDER}/include)

set(planner2_SOURCES src/jointmpc.cpp)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
 CATKIN_DEPENDS geometry_msgs rospy roscpp
)

# C++ files
include_directories(${catkin_INCLUDE_DIRS} include)
include_directories(${GUROBI_INCLUDE_DIRS})
link_directories(${GUROBI_LIB_DIRS})

add_library(planner2_LIB ${planner2_SOURCES})

add_executable(planner2 src/planner2.cpp)
target_link_libraries(planner2 ${catkin_LIBRARIES} planner2_LIB ${GUROBI_LIBS})
add_dependencies(planner2 expt2_generate_messages_cpp)

# Python files
catkin_install_python(PROGRAMS
  src/ego_vehicle.py
  src/NV.py
  src/stopped_vehicle.py
  src/NV_.py
  src/g920.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)