cmake_minimum_required(VERSION 3.0.2)
project(lidar_localization)

set(CMAKE_CXX_FLAGS "-std=c++11 -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")

find_package(
  catkin
  REQUIRED
  COMPONENTS 
  roscpp 
  roslaunch 
  rviz
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  laser_geometry
  nodelet
  obstacle_detector
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_nodelets
  CATKIN_DEPENDS roscpp rviz std_msgs geometry_msgs sensor_msgs tf2 tf2_ros laser_geometry nodelet obstacle_detector
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
)

#
# Build libs
#
add_library(lidar_localization src/lidar_localization.cpp)
target_link_libraries(lidar_localization ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})
add_dependencies(lidar_localization ${catkin_EXPORTED_TARGETS})

add_library(area_obstacles_extractor src/area_obstacles_extractor.cpp)
target_link_libraries(area_obstacles_extractor ${catkin_LIBRARIES})
add_dependencies(area_obstacles_extractor ${catkin_EXPORTED_TARGETS})

#
# Build nodes
#
add_executable(rx_to_odom src/rx_to_odom.cpp)
target_link_libraries(rx_to_odom ${PROJECT_NAME})

add_executable(scan_rosbag src/scan_rosbag.cpp)
target_link_libraries(scan_rosbag ${PROJECT_NAME})

add_executable(lidar_localization_node src/nodes/lidar_localization_node.cpp)
target_link_libraries(lidar_localization_node lidar_localization)

add_executable(area_obstacles_extractor_node src/nodes/area_obstacles_extractor_node.cpp)
target_link_libraries(area_obstacles_extractor_node area_obstacles_extractor)

#
# Build nodelets
#
add_library(${PROJECT_NAME}_nodelets src/nodelets/${PROJECT_NAME}_nodelet.cpp)
target_link_libraries(${PROJECT_NAME}_nodelets ${PROJECT_NAME})


install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_nodelets
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
