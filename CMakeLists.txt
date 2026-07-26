cmake_minimum_required(VERSION 3.8)
project(jetpilot_vesc_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(control_cmd_to_vesc_node src/control_cmd_to_vesc_node.cpp)
target_compile_features(control_cmd_to_vesc_node PUBLIC cxx_std_17)

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
