cmake_minimum_required(VERSION 3.8)
project(moveit2_scripts)

# Set compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(grasping_msgs REQUIRED)

# Skip linters if testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Function to create and configure executables
function(create_moveit_executable target_name)
  add_executable(${target_name} src/${target_name}.cpp)
  target_include_directories(${target_name} PUBLIC include)
  ament_target_dependencies(${target_name}
    ament_cmake rclcpp rclcpp_action moveit_core moveit_ros_planning_interface
    interactive_markers moveit_ros_planning grasping_msgs control_msgs)
  install(TARGETS ${target_name} DESTINATION lib/${PROJECT_NAME})
endfunction()

# Create and configure executables
create_moveit_executable(pick_and_place)
create_moveit_executable(pick_and_place_perception)
create_moveit_executable(pick_and_place_perception_real)

# Install the launch file
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
