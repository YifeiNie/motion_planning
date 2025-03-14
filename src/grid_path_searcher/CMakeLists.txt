cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 14)
project(grid_path_searcher)
set(CMAKE_BUILD_TYPE "Debug")	
set(CMAKE_CXX_FLAGS "-g -O0")
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs
    nav_msgs
    visualization_msgs 
    trajectory_optimization
    quad_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
    include 
    SYSTEM 
    third_party
    ${catkin_INCLUDE_DIRS} 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

add_library( ${PROJECT_NAME} 
    src/A_star.cpp)

target_link_libraries(${PROJECT_NAME}  
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} 
)

add_executable ( random_complex 
    src/random_complex_generator.cpp )

target_link_libraries( random_complex
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} )  
