cmake_minimum_required(VERSION 2.8.3)
project(pcl_to_grid_map)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  grid_map_ros
  grid_map_cv
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_rviz_plugin
  grid_map_visualization
  geometry_msgs
  grid_map_core
  pcl_ros
  sensor_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    grid_map_core
    pcl_ros
  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(PclToGridMap src/PclToGridMapNode.cpp)

##Declare a cpp library
add_library(${PROJECT_NAME}
  src/PclToGridMapNode.cpp
)

target_link_libraries(PclToGridMap
 ${catkin_LIBRARIES}
)



add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME} PclToGridMap
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

#############
## Testing ##
#############
