cmake_minimum_required(VERSION 2.8.3)
project(python_natnet)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

install(DIRECTORY python_natnet/test_data DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.bin")

