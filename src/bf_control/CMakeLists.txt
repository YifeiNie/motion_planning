set(CMAKE_CXX_STANDARD 14)
cmake_minimum_required(VERSION 3.0.2)
project(bf_control)
set(CMAKE_BUILD_TYPE "Release")	
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  message_runtime
  nav_msgs
  roscpp
  std_msgs
  quad_msgs
)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)


catkin_package(
  INCLUDE_DIRS include/bf_control
  CATKIN_DEPENDS geometry_msgs mavros_msgs message_runtime nav_msgs roscpp std_msgs
)

file(GLOB SRC_FILES "src/*cpp")

include_directories(
  include/bf_control
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
               ${SRC_FILES}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)