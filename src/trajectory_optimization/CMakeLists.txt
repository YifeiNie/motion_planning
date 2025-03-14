cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Debug")	
project(trajectory_optimization)
set(CMAKE_CXX_FLAGS "-g -O0")

find_package(catkin REQUIRED COMPONENTS  # 解析xml文件
  roscpp
  quad_msgs
)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# 用于指导其他功能包使用这个包
catkin_package(                                 
  INCLUDE_DIRS include/trajectory_optimization  # 指定该包的 头文件路径，让其他包在 #include 你的头文件时能找到它。
  LIBRARIES trajectory_optimization             # 指定该功能包生成的 库的名称，让其他包可以链接到它
  CATKIN_DEPENDS roscpp                         # 指定该功能包依赖于 roscpp
)

# 用于指导当前功能包的编译
include_directories(
  ${catkin_INCLUDE_DIRS}
  include/trajectory_optimization
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

file(GLOB SRC_FILES "src/*cpp")


# 生成一个叫做${PROJECT_NAME}的库，包含了${SRC_FILES}这些文件
add_library(${PROJECT_NAME}
               ${SRC_FILES}
)

# 生成的${PROJECT_NAME}库依赖于${catkin_LIBRARIES}和${YAML_CPP_LIBRARIES}
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} quad_msgs_generate_messages_cpp)