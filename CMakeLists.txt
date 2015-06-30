cmake_minimum_required(VERSION 2.8.3)
project(lar_comau)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  tf
  kdl_parser
  orocos_kdl
  message_generation
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/cmake/Modules/")
set(CMAKE_INCLUDE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/include/")

## Generate messages in the 'msg' folder
add_message_files(
 FILES
 ComauState.msg
 ComauCommand.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  geometry_msgs
)

catkin_package(
   CATKIN_DEPENDS
   message_runtime
   geometry_msgs
   std_msgs
)


#C4GOPEN Library
set(C4GOPEN_PATH $ENV{C4GOPEN_PATH})
MESSAGE( WOWOWOWOWO)
MESSAGE(${C4GOPEN_PATH})
include_directories(
  ${C4GOPEN_PATH}/include
)
link_directories(
  ${C4GOPEN_PATH}/lib/
)
set(C4GOPEN_LIBS -lC4gOpen)

#REALTIME Library
include_directories(
  /usr/realtime/include
  /usr/local/rtnet/include
  /usr/realtime/include/
  /usr/realtime/include/rtdm
  /usr/include/netinet
  /usr/include/net
)

#Commons Includes
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_INCLUDE_PATH}/commons
)

#LAR COMAU COMMONS LIBRARY
add_library(lar_comau_commons
              src/commons/ComauSmartSix.cpp
              src/commons/FIROrder2.cpp
)
target_link_libraries (lar_comau_commons ${catkin_LIBRARIES})


#NODE - Comau Driver
add_executable(comau_driver src/comau_driver.cpp )
target_link_libraries(
  comau_driver
  -lpthread
  ${C4GOPEN_LIBS}
  ${catkin_LIBRARIES}
)

#NODE - Comau Cartesian Controller
add_executable(comau_cartesian_controller src/comau_cartesian_controller.cpp)
target_link_libraries(
  comau_cartesian_controller
  ${catkin_LIBRARIES}
  lar_comau_commons
)
add_dependencies(comau_cartesian_controller ${PROJECT_NAME}_generate_messages_cpp)

#NODE - Comau Cartesian Controller
add_executable(dan_tester src/dan_tester.cpp)
target_link_libraries(
  dan_tester
  ${catkin_LIBRARIES}
  lar_comau_commons
)
add_dependencies(dan_tester ${PROJECT_NAME}_generate_messages_cpp)
