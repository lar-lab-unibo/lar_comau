cmake_minimum_required(VERSION 2.8.3)
project(lar_comau)

if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(ColourBold  "${Esc}[1m")
  set(Red         "${Esc}[31m")
  set(Green       "${Esc}[32m")
  set(Yellow      "${Esc}[33m")
  set(Blue        "${Esc}[34m")
  set(Magenta     "${Esc}[35m")
  set(Cyan        "${Esc}[36m")
  set(White       "${Esc}[37m")
  set(BoldRed     "${Esc}[1;31m")
  set(BoldGreen   "${Esc}[1;32m")
  set(BoldYellow  "${Esc}[1;33m")
  set(BoldBlue    "${Esc}[1;34m")
  set(BoldMagenta "${Esc}[1;35m")
  set(BoldCyan    "${Esc}[1;36m")
  set(BoldWhite   "${Esc}[1;37m")
endif()

find_package(orocos_kdl REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  tf
  tf_conversions
  kdl_parser
  message_generation
)


#Lar Tools
MESSAGE("###### LAR_TOOLS ########")
find_package(lar_tools)
include_directories(${lar_tools_INCLUDE_DIRS})

#Boost
FIND_PACKAGE( Boost )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
endif()



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
MESSAGE("${Green}=== C4G & REALTIME LIBRARIES ===${ColourReset}\n")
if(C4GOPEN_PATH)
  set(USE_C4G true)
  MESSAGE("${Green} C4G LIBRARY OK! ${ColourReset}")
else(C4GOPEN_PATH)
  MESSAGE("${Red} NO C4G FOUND! ${ColourReset}")
endif(C4GOPEN_PATH)
MESSAGE("\n")
MESSAGE("${Green}================================${ColourReset}")

if(USE_C4G)
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
endif(USE_C4G)

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
if(USE_C4G)
  add_executable(comau_driver src/comau_driver.cpp )
  target_link_libraries(
    comau_driver
    -lpthread
    ${C4GOPEN_LIBS}
    ${catkin_LIBRARIES}
  )
endif(USE_C4G)

#NODE - Comau Cartesian Controller
add_executable(comau_cartesian_controller src/comau_cartesian_controller.cpp)
target_link_libraries(
  comau_cartesian_controller
  ${catkin_LIBRARIES}
  lar_comau_commons
)
add_dependencies(comau_cartesian_controller ${PROJECT_NAME}_generate_messages_cpp)




#NODE - Comau Testing Bench
add_executable(dan_tester src/dan_tester.cpp)
target_link_libraries(
  dan_tester
  ${catkin_LIBRARIES}
  lar_comau_commons
)
add_dependencies(dan_tester ${PROJECT_NAME}_generate_messages_cpp)

#NODE - Gripper Test Novembre
add_executable(gripper_test_novembre src/gripper_test_novembre.cpp)
target_link_libraries(
  gripper_test_novembre
  ${catkin_LIBRARIES}
  lar_comau_commons
)
add_dependencies(gripper_test_novembre ${PROJECT_NAME}_generate_messages_cpp)


#NODE - Camera On Robot Calibration
add_executable(camera_on_robot_calibration src/camera_on_robot_calibration.cpp)
target_link_libraries(
  camera_on_robot_calibration
  ${catkin_LIBRARIES}
  lar_tools
  lar_comau_commons
)
add_dependencies(camera_on_robot_calibration ${PROJECT_NAME}_generate_messages_cpp)


if(Boost_FOUND)
  #NODE - Comau Bridge UDP
  add_executable(comau_bridge_udp src/comau_bridge_udp.cpp)
  target_link_libraries(
    comau_bridge_udp
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    lar_tools
    lar_comau_commons
  )
endif()
