cmake_minimum_required(VERSION 3.0.2)
project(localization)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_ros
)



add_executable(pcd_to_pointcloud_node src/pcd_to_pointcloud.cpp)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES localization
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(pcd_to_pointcloud_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Install executables and libraries
install(TARGETS pcd_to_pointcloud_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

