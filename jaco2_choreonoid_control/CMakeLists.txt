cmake_minimum_required(VERSION 2.8.3)
project(jaco2_choreonoid_control)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  control_msgs
  roscpp
  roslaunch
  sensor_msgs
  trajectory_msgs
  )

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
  )

# Build
add_definitions("-std=c++11")

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(jaco2_choreonoid_control
  src/jaco2_choreonoid_control.cpp
)

target_link_libraries(jaco2_choreonoid_control
  ${catkin_LIBRARIES}
)

# # Install
# foreach(modules jaco2_choreonoid_control_node)
#   install(TARGETS ${modules}
#     ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#   )
# endforeach(modules)

# foreach(dir config launch)
#   install(DIRECTORY ${dir}/
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)

# # Test
# if (CATKIN_ENABLE_TESTING)
#   find_package(roslaunch REQUIRED)
#   roslaunch_add_file_check(launch)
# endif()
