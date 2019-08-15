cmake_minimum_required(VERSION 2.8.3)
project(wamv_thruster_manager)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs)

catkin_python_setup()

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

catkin_install_python(PROGRAMS src/thrust_allocator.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
