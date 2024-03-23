cmake_minimum_required(VERSION 3.8)
# cmake_minimum_required(VERSION 3.0.2)
project(path_finding_util)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
add_compile_options(-std=c++17)

set(ROS_LIBRARIES
  grid_map
)

find_package(catkin REQUIRED COMPONENTS
  ${ROS_LIBRARIES}
)
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES path_finding_util # Exported libraries
 CATKIN_DEPENDS grid_map # Export grid_map as part of this package
#  DEPENDS system_lib
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS} 
)

add_library(path_finding_util src/path_tools.cpp src/graph_search.cpp src/global_planner.cpp src/prm.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(path_finding_util
  ${catkin_LIBRARIES}
)
