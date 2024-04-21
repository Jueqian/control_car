cmake_minimum_required(VERSION 3.0.2)
project(control_car)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

list(APPEND CMAKE_PREFIX_PATH "/opt/ros/noetic/share/rosserial")



find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  geometry_msgs
  serial
)



###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES control_car
#  CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(control_car src/control_car.cpp src/canusb.cpp)
target_link_libraries(control_car ${catkin_LIBRARIES})

add_executable(game_controller_simulate src/joy_controller_simulate.cpp)
target_link_libraries(game_controller_simulate ${catkin_LIBRARIES})

add_executable(usart_RemoteSend src/usart_RemoteSend.cpp)
target_link_libraries(usart_RemoteSend ${catkin_LIBRARIES})

add_executable(usart_RemoteReceive src/usart_RemoteReceive.cpp)
target_link_libraries(usart_RemoteReceive ${catkin_LIBRARIES})


# add_executable(send_ceshi src/send_ceshi.cpp)
# target_link_libraries(send_ceshi ${catkin_LIBRARIES})

