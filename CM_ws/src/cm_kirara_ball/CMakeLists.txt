cmake_minimum_required(VERSION 3.5)
project(cm_kirara_ball)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  moveit_msgs
  roscpp
  rospy
  std_msgs
  trajectory_msgs
  urdf
)
find_package(Boost REQUIRED)
find_package(fcl REQUIRED)
find_package(hpp-fcl REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(pinocchio REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.5)

catkin_package(INCLUDE_DIRS include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
  ${HPP_FCL_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
)

link_directories(
  ${FCL_LIBRARY_DIRS}
  ${HPP_FCL_LIBRARY_DIRS}
  ${PINOCCHIO_LIBRARY_DIRS}
  ${YAMLCPP_LIBRARY_DIRS}
)

set(CM_KIRARA_BALL_LIBRARIES
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${FCL_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${PINOCCHIO_LIBRARIES}
  hpp-fcl::hpp-fcl
  yaml-cpp
)

add_executable(CM_SOCKET_BALL_VALIDATOR
  src/CM_SOCKET_BALL_VALIDATOR.cpp)
target_link_libraries(CM_SOCKET_BALL_VALIDATOR
  ${CM_KIRARA_BALL_LIBRARIES})
add_dependencies(CM_SOCKET_BALL_VALIDATOR ${catkin_EXPORTED_TARGETS})

add_executable(CM_SOCKET_BALL_GOAL
  src/CM_SOCKET_BALL_GOAL.cpp)
target_link_libraries(CM_SOCKET_BALL_GOAL
  ${CM_KIRARA_BALL_LIBRARIES})
add_dependencies(CM_SOCKET_BALL_GOAL ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS
  scripts/sample_kirara.py
  scripts/generate_socket_ball_plan.py
  scripts/validate_latest.py
  scripts/validate_obstacle_clearance.py
  scripts/cm_socket_planner_node.py
  scripts/run_socket_planning_pair.py
  scripts/cm_distributed_goal_node.py
  scripts/run_distributed_goal_pair.py
  scripts/cm_ball_sim_left.py
  scripts/cm_ball_sim_right.py
  scripts/cm_ball_real_left.py
  scripts/cm_ball_real_right.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  CM_SOCKET_BALL_VALIDATOR
  CM_SOCKET_BALL_GOAL
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
