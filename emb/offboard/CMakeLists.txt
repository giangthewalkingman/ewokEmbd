cmake_minimum_required(VERSION 3.0.2)
project(offboard)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  roslaunch
  rospy
  std_msgs
  nav_msgs
  mav_trajectory_generation 
  mav_trajectory_generation_ros
  message_generation
  controller_msgs
  geometric_controller
)

find_package(controller_msgs REQUIRED)

add_message_files(
  FILES
  FlatTarget.msg
  # PositionCommand.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES offboard
   CATKIN_DEPENDS geometry_msgs mavros_msgs geometric_controller roscpp rospy std_msgs nav_msgs mav_trajectory_generation mav_trajectory_generation_ros message_runtime controller_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CATKIN_WORKSPACE}/src/emb/px4_controllers/geometric_controller/include/
  ${CATKIN_WORKSPACE}/src/emb/px4_controllers/controller_msgs/include/
)

roslaunch_add_file_check(launch)

add_library(offboard_lib
  src/offboard_lib.cpp
)
target_link_libraries(offboard_lib
  ${catkin_LIBRARIES}
  # geometric_controller
  
)

add_executable(offboard_node src/offboard_node.cpp)
target_link_libraries(offboard_node
  offboard_lib
)

add_executable(setmode_offb src/setmode_offb.cpp)
target_link_libraries(setmode_offb
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/MarkerDetection.py
  scripts/real_cam.py
  scripts/transform.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /home/giang/Desktop/duong_ws_test/src/emb/px4_controllers/geometric_controller)

# include_directories(
#   ${catkin_INCLUDE_DIRS}
#   ${CATKIN_WORKSPACE}/src/emb/px4_controllers/geometric_controller/include/
# )

# add_library(offboard_lib src/offboard_lib.cpp)
# target_link_libraries(offboard_lib geometric_controller)
