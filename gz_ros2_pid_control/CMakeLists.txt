cmake_minimum_required(VERSION 3.8)
project(gz_ros2_pid_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(gz_ros2_control REQUIRED)

# Select Gazebo dependency set
if("$ENV{GZ_VERSION}" STREQUAL "harmonic")
  find_package(gz-sim8 REQUIRED)
  set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo harmonic")
  find_package(gz-plugin2 REQUIRED)
  set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
  set(GZ_PLUGIN gz-plugin${GZ_PLUGIN_VER}::register)
  set(GZ_SIM gz-sim${GZ_SIM_VER}::core)
  add_definitions(-DGZ_HEADERS)
else()
  find_package(ignition-gazebo6 REQUIRED)
  set(GZ_SIM_VER ${ignition-gazebo6_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo fortress")
  find_package(ignition-plugin1 REQUIRED)
  set(GZ_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
  set(GZ_PLUGIN ignition-plugin${GZ_PLUGIN_VER}::register)
  set(GZ_SIM ignition-gazebo${GZ_SIM_VER}::core)
endif()

add_library(${PROJECT_NAME} SHARED
  src/gz_pid_system.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pluginlib
  hardware_interface
  gz_ros2_control
)

target_link_libraries(${PROJECT_NAME}
  ${GZ_SIM}
  ${GZ_PLUGIN}
)

pluginlib_export_plugin_description_file(
  gz_ros2_control
  gz_hardware_plugins.xml
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES gz_hardware_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
