cmake_minimum_required(VERSION 3.0.2)
project(aubo_i5_planning_interface)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS rospy 
)

catkin_install_python(
    PROGRAMS 
        scripts/move_line
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
