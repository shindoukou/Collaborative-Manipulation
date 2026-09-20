cmake_minimum_required(VERSION 3.0.2)
project(cm_trajectory_optimization)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/cm_trajopt_arm_node.py
  scripts/cm_trajopt_common.py
  scripts/cm_trajopt_protocol.py
  scripts/cm_trajopt_transport.py
  scripts/run_distributed_trajopt_pair.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
