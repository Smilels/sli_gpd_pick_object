cmake_minimum_required(VERSION 2.8.3)
project(sli_gpd_pick_object)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  moveit_core
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  sli_gpd
  tf
  tf_conversions
  eigen_conversions
  pcl_ros
  message_filters
  message_generation
)
find_package(PCL REQUIRED)

add_service_files(
 FILES
 depth_GQCNNPredict.srv
)
#include_directories(
# ${catkin_INCLUDE_DIRS}
#)
generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs # Or other packages containing msgs
)
catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS pcl_conversions message_runtime DEPENDS
PCL
)
include_directories(include ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_pick_object_demo src/pick_object_demo.cpp)
target_link_libraries(${PROJECT_NAME}_pick_object_demo ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_plan_grasps_service src/plan_grasps_service.cpp
                                   src/examine_grasps.cpp)
target_link_libraries(${PROJECT_NAME}_plan_grasps_service  ${catkin_LIBRARIES} )
add_dependencies(${PROJECT_NAME}_plan_grasps_service ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_bounding_box_preprocess src/bounding_box_preprocess.cpp)
target_link_libraries(${PROJECT_NAME}_bounding_box_preprocess ${catkin_LIBRARIES} ${PCL_LIBRARIES})
