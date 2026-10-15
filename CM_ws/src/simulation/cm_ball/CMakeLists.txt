cmake_minimum_required(VERSION 3.5)
project(cm_ball)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/sim_left.py
  scripts/sim_right.py
  scripts/real_left.py
  scripts/real_right.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES scripts/cm_short_entry.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
