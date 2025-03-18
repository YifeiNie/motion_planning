cmake_minimum_required(VERSION 3.0.2)
project(plan_manage)
set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_BUILD_TYPE "Debug")	
# set(CMAKE_CXX_FLAGS "-g -O0")

set(CMAKE_BUILD_TYPE "Release")	
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  grid_path_searcher
  quad_msgs
  roscpp
  std_msgs
  trajectory_optimization
  mapping
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include/plan_manage
)

include_directories(
  include/plan_manage
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

file(GLOB SRC_FILES "src/*cpp")

# add_executable ( ${PROJECT_NAME} 
#                 ${SRC_FILES} )

add_executable ( ${PROJECT_NAME} 
                src/planner_FSM.cpp 
                src/simple_planner_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${PCL_INCLUDE_DIRS}
  grid_path_searcher
  trajectory_optimization
  mapping
)