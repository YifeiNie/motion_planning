cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Debug")	
project(trajectory_optimization)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  grid_path_searcher
)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include/trajectory_optimization
  LIBRARIES trajectory_optimization
  CATKIN_DEPENDS roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/trajectory_optimization
  
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

file(GLOB SRC_FILES "src/*cpp")

add_library(${PROJECT_NAME}
               ${SRC_FILES}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)