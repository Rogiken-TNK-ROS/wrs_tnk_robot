cmake_minimum_required(VERSION 2.8.3)
project(tps_base_link)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tps_base_link
   CATKIN_DEPENDS roscpp tf2 tf2_ros geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tps_base_link src/tps_base_link.cpp)
target_link_libraries(tps_base_link ${catkin_LIBRARIES})

#############
## Install ##
#############
foreach(dir launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)


#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
