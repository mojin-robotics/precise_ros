cmake_minimum_required(VERSION 2.8.3)
project(precise_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  angles
  cob_srvs
  geometry_msgs
  realtime_tools
  ros_control_boilerplate
  std_srvs
)

catkin_package(
  CATKIN_DEPENDS cob_srvs geometry_msgs std_srvs
)

## Build
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(precise_hw_interface src/pflex_device.cpp src/precise_hw_interface.cpp src/precise_tcp_interface.cpp)
add_dependencies(precise_hw_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(precise_hw_interface ${catkin_LIBRARIES})

add_executable(precise_hw_node src/precise_hw_node.cpp)
add_dependencies(precise_hw_node ${catkin_EXPORTED_TARGETS})
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
