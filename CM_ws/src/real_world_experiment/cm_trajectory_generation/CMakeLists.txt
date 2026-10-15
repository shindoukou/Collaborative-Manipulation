cmake_minimum_required(VERSION 3.5)
project(cm_trajectory_generation)

add_compile_options(-std=c++14)
find_package(catkin REQUIRED COMPONENTS kdl_parser moveit_msgs roscpp rospy std_msgs trajectory_msgs urdf)
find_package(Boost REQUIRED)
find_package(fcl REQUIRED)
find_package(hpp-fcl REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(pinocchio REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.5)

catkin_package(CATKIN_DEPENDS kdl_parser moveit_msgs roscpp rospy std_msgs trajectory_msgs urdf)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${FCL_INCLUDE_DIRS}
  ${HPP_FCL_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${PINOCCHIO_INCLUDE_DIRS} ${YAMLCPP_INCLUDE_DIRS})
link_directories(${FCL_LIBRARY_DIRS} ${HPP_FCL_LIBRARY_DIRS} ${PINOCCHIO_LIBRARY_DIRS} ${YAMLCPP_LIBRARY_DIRS})

function(add_planning_target target family)
  add_executable(${target} src/${target}.cpp)
  target_include_directories(${target} PRIVATE include/${family})
  target_link_libraries(${target} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${FCL_LIBRARIES}
    ${orocos_kdl_LIBRARIES} ${PINOCCHIO_LIBRARIES} hpp-fcl::hpp-fcl yaml-cpp)
  add_dependencies(${target} ${catkin_EXPORTED_TARGETS})
endfunction()

add_planning_target(CM_SOCKET_BALL_GOAL ball)
add_planning_target(CM_SOCKET_BALL_VALIDATOR ball)
add_planning_target(CM_KIRARA_BOX_GOAL box)
add_planning_target(CM_SOCKET_BOX_VALIDATOR box)

catkin_install_python(PROGRAMS
  scripts/plan.py
  scripts/real_planning.py
  scripts/generate_socket_ball_plan.py
  scripts/generate_socket_box_plan.py
  scripts/run_socket_planning_pair.py
  scripts/cm_socket_planner_node.py
  scripts/run_distributed_goal_pair.py
  scripts/cm_distributed_goal_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS CM_SOCKET_BALL_GOAL CM_SOCKET_BALL_VALIDATOR CM_KIRARA_BOX_GOAL CM_SOCKET_BOX_VALIDATOR
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch models generated DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
