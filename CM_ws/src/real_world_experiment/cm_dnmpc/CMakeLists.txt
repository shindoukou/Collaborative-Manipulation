cmake_minimum_required(VERSION 3.5)
project(cm_dnmpc)

find_package(catkin REQUIRED COMPONENTS geometry_msgs rospy sensor_msgs trajectory_msgs)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/controller_node.py
  scripts/controller_core.py
  scripts/state_protocol.py
  scripts/socket_transport.py
  scripts/object_profiles.py
  scripts/real_controller.py
  scripts/internal_force_projection.py
  scripts/adaptive_impedance.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch simulation_runtime
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
