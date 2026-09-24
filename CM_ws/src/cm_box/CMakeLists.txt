cmake_minimum_required(VERSION 3.5)
project(cm_box)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/plan.py
  scripts/goal.py
  scripts/sample.py
  scripts/validate.py
  scripts/clearance.py
  scripts/prepare.py
  scripts/boundary.py
  scripts/fallback.py
  scripts/sim_left.py
  scripts/sim_right.py
  scripts/real_left.py
  scripts/real_right.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES scripts/cm_short_entry.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
