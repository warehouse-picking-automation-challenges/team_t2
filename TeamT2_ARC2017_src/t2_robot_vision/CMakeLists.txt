cmake_minimum_required(VERSION 2.8.3)
project(T2_robot_vision)
set(CMAKE_BUILD_TYPE Debug)

# darknet by nishiyama
include_directories(
  #/home/dl-box/catkin_ws/src/T2_robot_vision/src/darknet
  /usr/local/cuda/include
  #/usr/local/cuda-8.0/include
)
link_directories(
  #/home/dl-box/catkin_ws/src/T2_robot_vision/src/darknet
  /usr/local/cuda/lib64
  #/usr/local/cuda-8.0/lib64
)

#find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  nodelet
  eigen_conversions
  pcl_conversions
  std_msgs
  sensor_msgs
  t2_msgs
  message_generation
  octomap_msgs
  rosbag
)

link_directories (
  /opt/pcl1.8.0/lib
  /opt/ros/indigo/lib
  /opt/opencv2.4.13_vtk/lib
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  CalibratedData.msg
  CancelCaptureReq.msg
  CancelCaptureRes.msg
  CapturedData.msg
  ItemizedData.msg
  RecognitionTarget.msg
  RecognizedItem.msg
  RecognizedResult.msg
  RecognizedSegment.msg
  SegmentedData.msg
  # OccupancyReq.msg
  OccupancyTarget.msg
  # OccupancyRes.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  #AcquireData.srv
  #AcquireImage.srv
  Capture.srv
  Conv.srv
  ItemCad.srv
  CadItem.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  catch_robot_msgs
  T2_robot_vision
  octomap_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES T2_robot_vision 
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs nodelet
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /opt/opencv2.4.13_vtk/include
  /opt/pcl1.8.0/include/pcl-1.8
  #${OpenCV_INCLUDE_DIRS}
  #${PCL_INCLUDE_DIRS}
)

add_library(
  robot_vision_manager
  src/robot_vision_nodelet.cpp
  src/robot_vision_manager.cpp
)
target_link_libraries(
  robot_vision_manager
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  #${PCL_COMMON_LIBRARIES}
  #${PCL_IO_LIBRARIES}
)
add_dependencies(
  robot_vision_manager
  ${PROJECT_NAME}_generate_messages_cpp
)

add_library(
  robot_vision_capture_realsense
  src/robot_vision_nodelet.cpp
  src/robot_vision_capture_realsense.cpp
)
target_link_libraries(
  robot_vision_capture_realsense
  ${catkin_LIBRARIES}
)
add_dependencies(
  robot_vision_capture_realsense
  ${PROJECT_NAME}_generate_messages_cpp
)
add_library(
  robot_vision_capture_ensenso
  src/robot_vision_nodelet.cpp
  src/robot_vision_capture_ensenso.cpp
)
find_library(NX_LIB NxLib64 /usr/lib)
target_link_libraries(
  robot_vision_capture_ensenso
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES} 
  ${NX_LIB}
)
add_dependencies(
  robot_vision_capture_ensenso
  ${PROJECT_NAME}_generate_messages_cpp
)


add_library(
  robot_vision_calibration
  src/robot_vision_nodelet.cpp
  src/robot_vision_calibration.cpp
)
target_link_libraries(
  robot_vision_calibration
  ${catkin_LIBRARIES}
)
add_dependencies(
  robot_vision_calibration
  ${PROJECT_NAME}_generate_messages_cpp
)

add_library(
  robot_vision_segment
  src/robot_vision_nodelet.cpp
  src/robot_vision_segment.cpp
  src/seg_common.cpp
  src/normal_vector.cpp
  src/plane.cpp
  src/cluster.cpp
)
add_definitions(
 -std=c++11
)
target_link_libraries(
  robot_vision_segment
  ${catkin_LIBRARIES}
)
add_dependencies(
  robot_vision_segment
  ${PROJECT_NAME}_generate_messages_cpp
)

add_library(
  robot_vision_recognition
  src/robot_vision_nodelet.cpp
  src/robot_vision_recognition.cpp
)
target_link_libraries(
  robot_vision_recognition
  ${catkin_LIBRARIES}
)
add_dependencies(
  robot_vision_recognition
  ${PROJECT_NAME}_generate_messages_cpp
)

add_library(
  robot_vision_recognition2
  src/robot_vision_nodelet.cpp
  src/robot_vision_recognition2.cpp
)
target_link_libraries(
  robot_vision_recognition2
  ${catkin_LIBRARIES}
)
add_dependencies(
  robot_vision_recognition2
  ${PROJECT_NAME}_generate_messages_cpp
)

add_library(
  robot_vision_findplane
  src/robot_vision_nodelet.cpp
  src/robot_vision_findplane.cpp
)
target_link_libraries(
  robot_vision_findplane
  ${catkin_LIBRARIES}
  pcl_io
  pcl_common
  pcl_search
  pcl_segmentation
  pcl_sample_consensus
)
add_dependencies(
  robot_vision_findplane
  ${PROJECT_NAME}_generate_messages_cpp
  pcl_io
  pcl_common
  pcl_search
  pcl_segmentation
  pcl_sample_consensus
)

add_definitions(
  #-DGPU
  #-DCUDNN
)
add_library(
  robot_vision_tottori
  src/robot_vision_nodelet.cpp
  src/robot_vision_tottori.cpp
)
target_link_libraries(
  robot_vision_tottori
  ${catkin_LIBRARIES}
  pcl_common
  #cuda
  #cudart
  #cublas
  #curand
  #cudnn
  m
  pthread
  darknet_cpu
  #darknet_gpu
)
add_dependencies(
  robot_vision_tottori
  ${PROJECT_NAME}_generate_messages_cpp
)

add_library(
  robot_vision_unification
  src/robot_vision_nodelet.cpp
  src/robot_vision_unification.cpp
  src/PCA.cpp
)
target_link_libraries(
  robot_vision_unification
  ${catkin_LIBRARIES}
  #${PCL_COMMON_LIBRARIES}
  #${PCL_IO_LIBRARIES}
  #${PCL_SEARCH_LIBRARIES}
  pcl_io
  pcl_common
  pcl_search
)
add_dependencies(
  robot_vision_unification
  ${PROJECT_NAME}_generate_messages_cpp
)

add_executable(
  sample_msg_publisher
  src/sample_msg_publisher.cpp
)
target_link_libraries(
  sample_msg_publisher
  ${catkin_LIBRARIES}
)
add_dependencies(
  sample_msg_publisher
  ${PROJECT_NAME}_generate_messages_cpp
)

add_library(
  robot_vision_occupancy
  src/robot_vision_nodelet.cpp
  src/robot_vision_occupancy.cpp
)
target_link_libraries(
  robot_vision_occupancy
  ${catkin_LIBRARIES}
  pcl_common
  pcl_io
  pcl_search
  pcl_filters
  octomap
)
add_dependencies(
  robot_vision_occupancy
  ${PROJECT_NAME}_generate_messages_cpp
)

add_executable(
  test_occupancy_publisher
  src/test_occupancy_publisher.cpp
)
target_link_libraries(
  test_occupancy_publisher
  ${catkin_LIBRARIES}
  pcl_common
  pcl_io
  pcl_search
  octomap
  octomath
)
add_dependencies(
  test_occupancy_publisher
  ${PROJECT_NAME}_generate_messages_cpp
)

add_executable(
  recognition_dumy_input_node
  src/recognition_dumy_input_node.cpp
)
target_link_libraries(
  recognition_dumy_input_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  #${PCL_COMMON_LIBRARIES}
  #${PCL_IO_LIBRARIES}
  pcl_common
  pcl_io
)
add_dependencies(
  recognition_dumy_input_node
  ${PROJECT_NAME}_generate_messages_cpp
)


