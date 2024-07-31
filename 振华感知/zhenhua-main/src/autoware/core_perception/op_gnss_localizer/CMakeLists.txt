cmake_minimum_required(VERSION 2.8.3)
project(op_gnss_localizer)

find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  nav_msgs
  autoware_msgs
  gnss
  tf
  op_planner
)

catkin_package()

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(op_fix2tfpose
  nodes/op_fix2tfpose/fix2tfpose.cpp
)
target_link_libraries(op_fix2tfpose ${catkin_LIBRARIES})
add_dependencies(op_fix2tfpose ${catkin_EXPORTED_TARGETS})

install(TARGETS op_fix2tfpose
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
