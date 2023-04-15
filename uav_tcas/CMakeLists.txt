cmake_minimum_required(VERSION 3.0.2)
project(uav_tcas)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES uav_tcas
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(node src/tcas.cpp)
target_link_libraries(node ${catkin_LIBRARIES})
