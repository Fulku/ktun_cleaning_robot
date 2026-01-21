cmake_minimum_required(VERSION 3.0.2)
project(ktun_cleaning_robot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  move_base_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch config maps worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  src/qr_reader.py
  src/task_manager.py
  src/room_cleaner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
