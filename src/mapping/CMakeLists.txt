cmake_minimum_required(VERSION 2.8.3)
project(mapping)

ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_BUILD_TYPE "Debug")	
set(CMAKE_CXX_FLAGS "-g -O0")

find_package(OpenCV REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs
  cv_bridge
  message_filters
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES mapping
 CATKIN_DEPENDS roscpp std_msgs
 DEPENDS OpenCV

)

include_directories( 
    SYSTEM 
    include/mapping
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

file(GLOB SRC_FILES "src/*cpp")

add_library(${PROJECT_NAME}
               ${SRC_FILES}
)

target_link_libraries( mapping
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    )  

