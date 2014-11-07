cmake_minimum_required(VERSION 2.8.3)
project(self_collision)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  geometry_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  visualization_msgs
  kdl_parser
  urdf
)

#find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

link_directories(${orocos_kdl_LIBRARY_DIRS})

include_directories(${catkin_INCLUDE_DIRS})# ${Eigen_INCLUDE_DIRS})

### Orocos Dependencies ###
# Note that orocos_use_package() does not need to be called for any dependency
# listed in the package.xml file

include_directories(${USE_OROCOS_INCLUDE_DIRS} include)
ADD_DEFINITIONS(-DEIGEN_RUNTIME_NO_MALLOC)
### Orocos Targets ###

#orocos_component(self_collision src/qhull_test.cpp src/urdf_collision_parser.cpp src/marker_publisher.cpp src/qhull_interface.cpp src/gjk.cpp src/transform.cpp src/intersect.cpp src/narrowphase.cpp src/geometric_shapes.cpp)
#orocos_component(self_collision src/self_collision_avoidance.cpp src/urdf_collision_parser.cpp src/marker_publisher.cpp src/qhull_interface.cpp src/gjk.cpp src/transform.cpp src/intersect.cpp src/narrowphase.cpp src/geometric_shapes.cpp)
orocos_component(self_collision src/self_collision_avoidance.cpp src/urdf_collision_parser.cpp src/gjk.cpp src/transform.cpp src/intersect.cpp src/narrowphase.cpp src/geometric_shapes.cpp)

target_link_libraries(self_collision ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES} ${orocos_kdl_LIBRARIES})
add_dependencies(self_collision ${PROJECT_NAME}_gencpp)

### Orocos Package Exports and Install Targets ###

# Export package information (replaces catkin_package() macro) 
orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS rtt_ros geometry_msgs std_msgs message_runtime rtt_roscomm
)

orocos_install_headers(
  DIRECTORY include/${PROJECT_NAME}/
)

