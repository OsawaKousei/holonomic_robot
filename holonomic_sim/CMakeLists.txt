cmake_minimum_required(VERSION 3.8)
project(holonomic_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-gazebo6 REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(SYSTEM
  ${IGNITION-COMMON_INCLUDE_DIRS}
  ${IGNITION-GUI_INCLUDE_DIRS}
)

# system plugin
add_library(HolonomicPlugin SHARED
  src/holonomic_plugin/holonomic_plugin.cpp
)
target_link_libraries(HolonomicPlugin
  ${IGNITION-PLUGIN_LIBRARIES}
  ${IGNITION-GAZEBO_LIBRARIES}
)

install(
  TARGETS HolonomicPlugin
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY
    launch
    models
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_auto_add_library(robot_vel src/robot_vel.cpp)
rclcpp_components_register_nodes(robot_vel "holonomic_sim::robotVel")
ament_auto_add_executable(
ign_ros_node src/ign_ros_node.cpp
  src/robot_vel.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
