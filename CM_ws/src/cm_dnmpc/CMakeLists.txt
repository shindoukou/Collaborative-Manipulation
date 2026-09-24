cmake_minimum_required(VERSION 3.5)
project(cm_dnmpc)

find_package(catkin REQUIRED COMPONENTS geometry_msgs rospy sensor_msgs trajectory_msgs)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/cm_export_selected_q.py
  scripts/cm_dnmpc_arm_node.py
  scripts/cm_dnmpc_common.py
  scripts/cm_dnmpc_left_node.py
  scripts/cm_dnmpc_protocol.py
  scripts/cm_dnmpc_right_node.py
  scripts/cm_dnmpc_transport.py
  scripts/cm_real_object_config.py
  scripts/cm_real_object_node.py
  scripts/internal_force_projection.py
  scripts/paper_elastic3.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
