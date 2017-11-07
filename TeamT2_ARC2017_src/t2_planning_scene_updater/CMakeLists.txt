cmake_minimum_required(VERSION 2.8.3)
project(t2_planning_scene_updater)

add_definitions(-std=c++11)

set(CMAKE_BUILD_TYPE Debug)

set(PACKAGE_DEPENDS
  roscpp
  pcl_ros
  moveit_core
  moveit_ros_planning
  moveit_ros_move_group
  moveit_ros_perception
  moveit_ros_planning_interface
  geometric_shapes
  t2_msgs
  catch_robot_msgs
  roslint
)

find_package(catkin REQUIRED COMPONENTS
  ${PACKAGE_DEPENDS}
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS message_runtime ${PACKAGE_DEPENDS}
  DEPENDS system_lib
  INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})

file(GLOB SOURCE_FILES src/*.cpp)
add_executable(${PROJECT_NAME} ${SOURCE_FILES})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)

set(ROSLINT_CPP_OPTS "--filter=-legal/copyright,-readability/todo,-build/include_what_you_use,-whitespace/braces,-runtime/references")
roslint_cpp()
