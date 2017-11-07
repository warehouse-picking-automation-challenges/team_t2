cmake_minimum_required(VERSION 2.8.3)

project(t2_ui)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  std_msgs
  t2_msgs
  roslint
)

link_directories(${catkin_LIBRARY_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS message_runtime ${PACKAGE_DEPENDS})

#-----------------------
add_executable(
  ${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
  src/hitkey.c
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

set(ROSLINT_CPP_OPTS "--filter=-legal/copyright,-readability/todo,-build/include_what_you_use,-whitespace/braces,-runtime/references")
roslint_cpp()
