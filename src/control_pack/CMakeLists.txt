cmake_minimum_required(VERSION 3.8)
project(control_pack)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(control_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)

pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(controller_interface controller_plugin.xml)
pluginlib_export_plugin_description_file(controller_interface arm_pid_controller_plugin.xml)
pluginlib_export_plugin_description_file(hardware_interface real_arm_hw_plugin.xml)

add_library(trajectory_controller SHARED
  src/controller/controller.cpp
)

add_library(arm_pid_controller SHARED
  src/arm_pid_controller/arm_pid_controller.cpp
)

target_include_directories(trajectory_controller
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(arm_pid_controller
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(trajectory_controller
  Eigen3::Eigen
  orocos-kdl
)

ament_target_dependencies(arm_pid_controller
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_target_dependencies(trajectory_controller
  control_msgs
  controller_interface
  hardware_interface
  joint_trajectory_controller
  kdl_parser
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  trajectory_msgs
)

add_library(real_arm_hw SHARED
  src/real_arm_hw/cdc_trans.cpp
  src/real_arm_hw/real_arm_hw.cpp
)

target_include_directories(real_arm_hw
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LIBUSB_INCLUDE_DIRS}
)

target_link_directories(real_arm_hw
  PUBLIC
  ${LIBUSB_LIBRARY_DIRS}
)

target_link_libraries(real_arm_hw
  ${LIBUSB_LIBRARIES}
)

ament_target_dependencies(real_arm_hw
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  robot_interfaces
  std_srvs
)

install(
  TARGETS trajectory_controller arm_pid_controller real_arm_hw
  LIBRARY DESTINATION lib
)

install(
  FILES controller_plugin.xml arm_pid_controller_plugin.xml real_arm_hw_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
