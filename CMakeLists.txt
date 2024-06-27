cmake_minimum_required(VERSION 3.8)
project(voskros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetGrammar.srv"
  DEPENDENCIES builtin_interfaces std_msgs 
)

ament_export_dependencies(rosidl_default_runtime)

# Create Cpp executable
#include_directories(include)
#add_executable(cpp_executable src/cpp_node.cpp)
#ament_target_dependencies(cpp_executable rclcpp)

# Install Cpp executables
#install(TARGETS
#  cpp_executable
#  DESTINATION lib/${PROJECT_NAME}
#)

# Install Python modules
#ament_python_install_package(${PROJECT_NAME})
# Install executables
install(PROGRAMS
  scripts/vosk
  scripts/prompter
  scripts/set_grammar.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files.
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
