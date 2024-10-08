cmake_minimum_required(VERSION 3.8)
project(canalystii_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

## executable
ament_auto_add_library(can_bridge_node SHARED
  src/canalystii.cpp
  src/can_bridge_node.cpp
)

find_library(CONTROL_CAN_LIB NAMES controlcan PATHS lib)
# message("find ${CONTROL_CAN_LIB}")

target_link_libraries(can_bridge_node
  ${CONTROL_CAN_LIB}
)

# 指定外部共享库的安装
install(FILES ${CONTROL_CAN_LIB}
  DESTINATION lib
)

rclcpp_components_register_node(can_bridge_node
  PLUGIN "drivers::can_driver::CanBridgeNode"
  EXECUTABLE can_bridge_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  lib
  launch
  config
)
