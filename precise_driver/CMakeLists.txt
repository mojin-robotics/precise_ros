cmake_minimum_required(VERSION 3.0.2)
project(precise_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cob_srvs
  controller_manager_msgs
  geometry_msgs
  message_generation
  realtime_tools
  ros_control_boilerplate
  roscpp
  std_srvs
  tf
)

add_service_files(
  FILES
  Gripper.srv
  MoveCartesian.srv
  Plate.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS cob_srvs controller_manager_msgs geometry_msgs message_runtime roscpp std_srvs tf
)

## Build
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(precise_hw_interface src/device/device.cpp src/device/tcp_client.cpp src/precise_hw_interface.cpp)
add_dependencies(precise_hw_interface ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(precise_hw_interface ${catkin_LIBRARIES})

add_executable(precise_hw_node src/precise_hw_node.cpp)
add_dependencies(precise_hw_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(precise_hw_node precise_hw_interface ${catkin_LIBRARIES})

## Install
install(TARGETS precise_hw_interface precise_hw_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/precise_hardware.launch)
endif()

