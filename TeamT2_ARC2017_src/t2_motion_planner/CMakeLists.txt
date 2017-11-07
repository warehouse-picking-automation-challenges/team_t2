cmake_minimum_required(VERSION 2.8.3)

project(t2_motion_planner)

add_compile_options(-Wall -std=c++11)

set(CMAKE_BUILD_TYPE Debug)

set(PACKAGE_DEPENDS
  cmake_modules
  roscpp
  actionlib
  actionlib_msgs
  moveit_core
  eigen_conversions
  tf_conversions
  pluginlib
  std_msgs
  control_msgs
  t2_msgs
  t2_pinching_gripper
  t2_planning_scene_updater
  roslint
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDS})

find_package(Boost REQUIRED)

find_package(Eigen REQUIRED)

link_directories(${catkin_LIBRARY_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS message_runtime ${PACKAGE_DEPENDS}
  DEPENDS Boost Eigen
  INCLUDE_DIRS include
)

file(GLOB SOURCE_FILES src/*.cpp)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
  
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

set(ROSLINT_CPP_OPTS "--filter=-legal/copyright,-readability/todo,-build/include_what_you_use,-whitespace/braces,-runtime/references")
roslint_cpp()
