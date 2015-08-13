cmake_minimum_required(VERSION 2.8.3)
project(kin_dyn_model)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  visualization_msgs
  urdf
  cmake_modules
)

find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)

link_directories(${orocos_kdl_LIBRARY_DIRS})

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

### Orocos Dependencies ###
# Note that orocos_use_package() does not need to be called for any dependency
# listed in the package.xml file

include_directories(${USE_OROCOS_INCLUDE_DIRS} include)
ADD_DEFINITIONS(-DEIGEN_RUNTIME_NO_MALLOC)
### Orocos Targets ###

add_library(kin_dyn_model src/kin_model.cpp src/dyn_model.cpp)

target_link_libraries(kin_dyn_model ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES} ${orocos_kdl_LIBRARIES} ${kdl_parser_LIBRARIES})
add_dependencies(kin_dyn_model ${PROJECT_NAME}_gencpp)

### Orocos Package Exports and Install Targets ###

# Export package information (replaces catkin_package() macro) 
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kin_dyn_model
  DEPENDS roscpp geometry_msgs visualization_msgs message_runtime urdf Eigen orocos_kdl kdl_parser
)

