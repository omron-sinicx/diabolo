cmake_minimum_required(VERSION 2.8.3)
project(natnet_ros)

find_package(catkin REQUIRED COMPONENTS rospy python_natnet)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS rospy natnet_msgs)

catkin_install_python(PROGRAMS scripts/client DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS test/logtest DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  # Bash uses 0 for true, CMake uses 1 for true
  execute_process(COMMAND "test $(rosversion -d) != indigo" RESULT_VARIABLE INDIGO)

  find_package(rostest REQUIRED)
  add_rostest(test/client.test)
  add_rostest(test/logging.test)
  if(NOT INDIGO)
    add_rostest(test/client_publishtest.test)
  endif()
endif()
