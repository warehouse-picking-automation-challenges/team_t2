cmake_minimum_required(VERSION 2.8.3)

project(t2_task_planner)

add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  pcl_ros
  std_msgs
  sensor_msgs
  t2_robot_msgs
  t2_msgs
  t2_planning_scene_updater
  t2_motion_planner
  t2_pinching_gripper
  roslint
  octomap_msgs
  eigen_conversions
  geometric_shapes
)

find_package(Eigen REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

link_directories(${catkin_LIBRARY_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

include_directories(${EIGEN_INCLUDE_DIRS})

add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
  CATKIN_DEPENDS message_runtime ${PACKAGE_DEPENDS}
  DEPENDS Eigen
)

file(GLOB SOURCE_FILES src/*.cpp)
add_executable(
  ${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
  src/t2_box_info.cpp
  src/t2_task_strategy_common.cpp
  src/t2_task_strategy_grasp_pack.cpp
  src/t2_task_strategy_grasp.cpp
  src/t2_task_strategy_pack.cpp
  src/t2_task_strategy_component.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${JSONCPP_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(
  t2_task_strategy_viewer
  src/t2_task_strategy_viewer.cpp
  src/t2_task_strategy_common.cpp
  src/t2_task_strategy_grasp_pack.cpp
  src/t2_task_strategy_grasp.cpp
  src/t2_task_strategy_pack.cpp
  src/t2_task_strategy_component.cpp
)
target_link_libraries(t2_task_strategy_viewer ${catkin_LIBRARIES})
add_dependencies(t2_task_strategy_viewer ${catkin_EXPORTED_TARGETS})

#-----------------------

execute_process(COMMAND chmod +x ${CMAKE_SOURCE_DIR}/t2_robot/${PROJECT_NAME}/scripts/gptest_update.py)


#-----------------------
#add_executable(
#  plan_arm_sim
#  src/plan_stub_arm.cpp
#)
#
#target_link_libraries(plan_arm_sim ${catkin_LIBRARIES})
#
#add_dependencies(plan_arm_sim ${catkin_EXPORTED_TARGETS})

#-----------------------
#add_executable(
#  t2_recoPC
#  src/recoPC.cpp
#)

#target_link_libraries(t2_recoPC ${catkin_LIBRARIES})

#add_dependencies(t2_recoPC ${catkin_EXPORTED_TARGETS})
  
#-----------------------
#add_executable(
#  plan_vr_ctrl
#  src/plan_vr_ctrl.cpp
#  )
#target_link_libraries(plan_vr_ctrl ${catkin_LIBRARIES})
#add_dependencies(plan_vr_ctrl ${catkin_EXPORTED_TARGETS})

set(ROSLINT_CPP_OPTS "--filter=-legal/copyright,-readability/todo,-build/include_what_you_use,-whitespace/braces,-runtime/references")
roslint_cpp()
