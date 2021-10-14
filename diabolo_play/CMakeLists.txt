cmake_minimum_required(VERSION 2.8.3)
project(diabolo_play)

## Add support for C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  tf_conversions
  moveit_core 
  moveit_commander
  moveit_ros_planning
  moveit_ros_planning_interface
  message_generation
  bio_ik
  std_msgs
  diabolo_gazebo
)
set(IGN_MATH_VER 6)
find_package(ignition-math${IGN_MATH_VER} REQUIRED)

add_message_files(DIRECTORY msg)
add_service_files(DIRECTORY srv)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()
generate_messages(DEPENDENCIES geometry_msgs std_msgs diabolo_gazebo trajectory_msgs)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES diabolo_motion_generator
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

# Declare a C++ executable
# With catkin_make all packages are built within a single CMake context
# The recommended prefix ensures that target names across packages don't collide
add_executable(diabolo_playback src/diabolo_playback.cpp)
add_dependencies(diabolo_playback 
	${${PROJECT_NAME}_EXPORTED_TARGETS}
  	${catkin_EXPORTED_TARGETS})
target_link_libraries(diabolo_playback ${catkin_LIBRARIES})

add_executable(stick_target_to_joint_target_converter src/stick_target_to_joint_target_converter.cpp)
add_dependencies(stick_target_to_joint_target_converter 
	${${PROJECT_NAME}_EXPORTED_TARGETS}
  	${catkin_EXPORTED_TARGETS})
target_link_libraries(stick_target_to_joint_target_converter ${catkin_LIBRARIES})

add_executable(diabolo_motion_generator src/diabolo_motion_generator.cpp)
add_dependencies(diabolo_motion_generator 
	${${PROJECT_NAME}_EXPORTED_TARGETS}
  	${catkin_EXPORTED_TARGETS})
target_link_libraries(diabolo_motion_generator ${catkin_LIBRARIES} ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER})
include_directories(include ${catkin_INCLUDE_DIRS} )

# add_library(${PROJECT_NAME} src/diabolo_motion_generator.cpp)
# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER})

# add_library(diabolo_motion_generator src/diabolo_motion_generator.cpp)
# target_link_libraries(diabolo_motion_generator ${catkin_LIBRARIES} ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER})

# catkin_install_python(PROGRAMS scripts/transformations.py
#                       DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# === Different attempts at getting the cross-package linking to work

# https://answers.ros.org/question/54173/install-destination-error-with-my-catkin-package/
# install(TARGETS diabolo_motion_generator
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# # https://roboticsbackend.com/ros-include-cpp-header-from-another-package/
# install(
#   TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

install(
  TARGETS diabolo_motion_generator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# ## Install project namespaced headers
# ## https://answers.ros.org/question/201977/include-header-file-from-another-package-indigo/
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE)

#############
## Testing ##
#############

