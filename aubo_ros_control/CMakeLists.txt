cmake_minimum_required(VERSION 3.0.2)
project(aubo_ros_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_manager
  hardware_interface
  joint_limits_interface
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES aubo_ros_control
 CATKIN_DEPENDS 
  roscpp
  controller_manager
  hardware_interface
  joint_limits_interface
 DEPENDS
)

include_directories(include dependent/robotsdk/inc dependent/servojsdk/inc  ${catkin_INCLUDE_DIRS})

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        link_directories(${catkin_LIB_DIRS}  dependent/robotsdk/lib/linux_x64 dependent/servojsdk/lib/linux_x64)
else()
        link_directories(${catkin_LIB_DIRS}  dependent/robotsdk/lib/linux_x32 dependent/servojsdk/lib/linux_x32)
endif()

add_executable(test_aubo_driver
    src/test_aubo_driver.cpp
    src/aubo_driver.cpp
)
add_dependencies(test_aubo_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_aubo_driver ${catkin_LIBRARIES} -laubo_sdk -lservojsdk -laral)

add_executable(aubo_hardware_interface_node
    src/aubo_hardware_interface_node.cpp
    src/aubo_hardware_interface.cpp
    src/aubo_driver.cpp
)
add_dependencies(aubo_hardware_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(aubo_hardware_interface_node ${catkin_LIBRARIES} -laubo_sdk -lservojsdk -laral)