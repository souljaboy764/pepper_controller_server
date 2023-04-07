cmake_minimum_required(VERSION 3.0.2)
project(pepper_controller_server)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-g)

set(CATKIN_COMPONENTS
  controller_interface
  hardware_interface
  roscpp
  pluginlib
  trajectory_msgs
  message_generation
  trajectory_msgs
)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_COMPONENTS})

add_service_files(
  FILES
  JointTarget.srv
)

generate_messages(
  DEPENDENCIES
  trajectory_msgs
)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_COMPONENTS}
  # DEPENDS system_lib
)

include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(position_stiffness_controller src/position_stiffness_controller.cpp)
target_link_libraries(position_stiffness_controller ${catkin_LIBRARIES})

add_library(compliant_controller src/compliant_controller.cpp src/position_stiffness_controller.cpp)
target_link_libraries(compliant_controller ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS position_stiffness_controller compliant_controller
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS compliant_controller
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugins/position_stiffness_controller_plugins.xml plugins/compliant_controller_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/include)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
