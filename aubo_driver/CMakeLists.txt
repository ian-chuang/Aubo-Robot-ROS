cmake_minimum_required(VERSION 3.8)
project(aubo_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ruckig REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_manager
  controller_manager_msgs
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  std_srvs
  tf2_geometry_msgs
  ruckig
)

add_library(
  aubo_driver_plugin
  SHARED
  src/aubo_hardware_interface.cpp
  src/aubo_driver.cpp
)
# specify aubo sdk libraries directories
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  target_link_directories(
    aubo_driver_plugin 
    PUBLIC
    dependent/robotsdk/lib/linux_x64 
    dependent/servojsdk/lib/linux_x64
  )
else()
  target_link_directories(
    aubo_driver_plugin 
    PUBLIC
    dependent/robotsdk/lib/linux_x32 
    dependent/servojsdk/lib/linux_x32
  )
endif()
# link libraries, including ruckig and aubo sdk
target_link_libraries(
  aubo_driver_plugin
  -laral
  -laubo_sdk
  -lservojsdk
  ruckig::ruckig
)
# include directories, also include aubo sdk headers
target_include_directories(
  aubo_driver_plugin
  PRIVATE
  include
  dependent/robotsdk/inc 
  dependent/servojsdk/inc
)
ament_target_dependencies(
  aubo_driver_plugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

# INSTALL
install(
  TARGETS aubo_driver_plugin
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

# add dependent libraries to install
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  install(
    FILES 
    dependent/robotsdk/lib/linux_x64/libaral.so
    dependent/robotsdk/lib/linux_x64/libaubo_sdk.so
    dependent/robotsdk/lib/linux_x64/libaubo_sdk.so.2
    dependent/robotsdk/lib/linux_x64/libaubo_sdk.so.2.5
    dependent/robotsdk/lib/linux_x64/libaubo_sdk.so.2.5.3
    dependent/servojsdk/lib/linux_x64/libservojsdk.so
    dependent/servojsdk/lib/linux_x64/libservojsdk.so.1
    dependent/servojsdk/lib/linux_x64/libservojsdk.so.1.0
    dependent/servojsdk/lib/linux_x64/libservojsdk.so.1.0.6
    DESTINATION lib
  )
else()
  install(
    FILES 
    dependent/robotsdk/lib/linux_x32/libaral.so
    dependent/robotsdk/lib/linux_x32/libaubo_sdk.so
    dependent/robotsdk/lib/linux_x32/libaubo_sdk.so.2
    dependent/robotsdk/lib/linux_x32/libaubo_sdk.so.2.5
    dependent/robotsdk/lib/linux_x32/libaubo_sdk.so.2.5.3
    dependent/servojsdk/lib/linux_x32/libservojsdk.so
    dependent/servojsdk/lib/linux_x32/libservojsdk.so.1
    dependent/servojsdk/lib/linux_x32/libservojsdk.so.1.0
    dependent/servojsdk/lib/linux_x32/libservojsdk.so.1.0.6
    DESTINATION lib
  )
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  aubo_driver_plugin
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
 
install(DIRECTORY config launch urdf rviz dependent
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
