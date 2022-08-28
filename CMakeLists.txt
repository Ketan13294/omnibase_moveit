cmake_minimum_required(VERSION 3.1.3)
project(omnibase_moveit)

add_definitions(-std=c++11)
find_package(catkin REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread serialization)
find_package( catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  roscpp
  rosconsole
  pluginlib
  tf
  dynamic_reconfigure
  eigen_conversions
  omnibase_description)


catkin_package(LIBRARIES RMLPlanner_context RMLPlanner_plugin)

include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(SYSTEM ${EIGEN_INCLUDE_DIR})
include_directories( ${catkin_INCLUDE_DIRS} )

link_directories(${Boost_LIBRARY_DIRS})

add_library( RMLPlanner_context src/RMLPlanner_context.cpp )
target_link_libraries( RMLPlanner_context ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library( RMLPlanner_plugin src/RMLPlanner_plugin.cpp)
target_link_libraries( RMLPlanner_plugin RMLPlanner_context ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
