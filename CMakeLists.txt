cmake_minimum_required(VERSION 3.0.2)
project(lazy_mapping_moveit)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED
  COMPONENTS
    # moveit_core
    # moveit_ros_planning
    moveit_ros_planning_interface
    # moveit_ros_perception
    moveit_visual_tools
    # pluginlib
    # geometric_shapes
    # pcl_ros
    # pcl_conversions
    # rosbag
    # tf2_ros
    # tf2_eigen
    # tf2_geometry_msgs
    tf
    visp_bridge
    message_generation
    # std_msgs
    # sensor_msgs
    # roscpp
    # rospy
)

add_message_files(
  FILES
  Marker.msg
  Markers.msg
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs lazy_mapping_moveit)


find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(VISP REQUIRED)

set(THIS_PACKAGE_INCLUDE_DIRS
  doc/interactivity/include
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    # moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    # interactive_markers
    # tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(
    include
    ${VISP_INCLUDE_DIRS}
)

add_executable(obstacles_to_moveit src/obstacles_to_moveit.cpp)
target_link_libraries(obstacles_to_moveit ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(ar_detector_node src/ar_detector_node.cpp)
target_link_libraries(ar_detector_node ${catkin_LIBRARIES} ${VISP_LIBRARIES})
add_dependencies(ar_detector_node ${catkin_EXPORTED_TARGETS})

