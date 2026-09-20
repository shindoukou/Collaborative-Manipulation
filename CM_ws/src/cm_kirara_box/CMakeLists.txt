cmake_minimum_required(VERSION 3.5)
project(cm_kirara_box)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  kdl_parser moveit_msgs roscpp rospy std_msgs trajectory_msgs urdf)
find_package(Boost REQUIRED)
find_package(fcl REQUIRED)
find_package(hpp-fcl REQUIRED)
find_package(ompl REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(pinocchio REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.5)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS kdl_parser moveit_msgs roscpp rospy std_msgs trajectory_msgs urdf)

include_directories(
  include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${FCL_INCLUDE_DIRS}
  ${HPP_FCL_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDE_DIRS} ${YAMLCPP_INCLUDE_DIRS})

link_directories(
  ${FCL_LIBRARY_DIRS} ${HPP_FCL_LIBRARY_DIRS} ${OMPL_LIBRARY_DIRS}
  ${PINOCCHIO_LIBRARY_DIRS} ${YAMLCPP_LIBRARY_DIRS})

set(CM_KIRARA_BOX_LIBRARIES
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${FCL_LIBRARIES}
  ${OMPL_LIBRARIES} ${orocos_kdl_LIBRARIES} ${PINOCCHIO_LIBRARIES}
  hpp-fcl::hpp-fcl yaml-cpp)

function(add_kirara_box_executable target source)
  add_executable(${target} ${source})
  target_link_libraries(${target} ${CM_KIRARA_BOX_LIBRARIES})
  add_dependencies(${target} ${catkin_EXPORTED_TARGETS})
endfunction()

add_kirara_box_executable(CM_KIRARA_BOX_CATCH src/CM_KIRARA_BOX_CATCH.cpp)
add_kirara_box_executable(CM_KIRARA_BOX_MOVE src/CM_KIRARA_BOX_MOVE.cpp)
add_kirara_box_executable(CM_KIRARA_BOX_GOAL test/CM_KIRARA_BOX_GOAL.cpp)
add_kirara_box_executable(CM_SOCKET_BOX_VALIDATOR src/CM_SOCKET_BOX_VALIDATOR.cpp)

catkin_install_python(PROGRAMS
  scripts/cm_box_attachment.py
  scripts/plan_kirara_box.py
  scripts/generate_socket_box_plan.py
  scripts/validate_side_approach_kirara_box.py
  scripts/sample_kirara.py
  scripts/safe_planning_fallback_kirara_box.py
  scripts/cm_box_sim_left.py
  scripts/cm_box_sim_right.py
  scripts/cm_box_real_left.py
  scripts/cm_box_real_right.py
  scripts/search_move_fcl_boundary.py
  scripts/validate_obstacle_clearance.py
  scripts/prepare_move_reference.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS CM_KIRARA_BOX_CATCH CM_KIRARA_BOX_MOVE CM_KIRARA_BOX_GOAL
  CM_SOCKET_BOX_VALIDATOR
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
