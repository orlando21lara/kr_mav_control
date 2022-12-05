cmake_minimum_required(VERSION 3.10)
project(kr_uzh_interface)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  autopilot
  kr_mav_msgs
  geometry_msgs
  kr_mav_msgs
  nav_msgs
  roscpp
  std_msgs
  tf2
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS autopilot kr_mav_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mpc_input_bridge_node src/mpc_input_bridge_node.cpp src/mpc_input_bridge.cpp)
target_link_libraries(mpc_input_bridge_node ${catkin_LIBRARIES})

add_executable(mpc_output_bridge_node src/mpc_output_bridge_node.cpp src/mpc_output_bridge.cpp)
target_link_libraries(mpc_output_bridge_node ${catkin_LIBRARIES})


