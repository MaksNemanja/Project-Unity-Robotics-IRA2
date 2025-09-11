cmake_minimum_required(VERSION 3.8)
project(unity_robotics_ira2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(ur_rtde REQUIRED)
find_package(Eigen3 REQUIRED)

# Install message files

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HapticInfo.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# define executables
add_executable(unity_control src/unity_control.cpp)
add_executable(unity_control_v2 src/unity_control_v2.cpp)
add_executable(robot_control src/robot_control.cpp)
add_executable(joints_sub src/joints_sub.cpp)
add_executable(haptic_control src/haptic_control.cpp)
add_executable(get_force src/get_force.cpp)

# Define dependencies for each executable
ament_target_dependencies(unity_control
  rclcpp 
  geometry_msgs
  std_msgs
  tf2
)

ament_target_dependencies(unity_control_v2
  rclcpp 
  geometry_msgs
  std_msgs
  tf2
)


ament_target_dependencies(robot_control 
  rclcpp 
  std_msgs
)


ament_target_dependencies(joints_sub 
  rclcpp 
  sensor_msgs
  ur_rtde
)

ament_target_dependencies(haptic_control
  rclcpp 
  geometry_msgs
  std_msgs
  tf2
  Boost
  ur_rtde
  std_msgs
)
add_dependencies(haptic_control ${PROJECT_NAME}__rosidl_generator_cpp)



ament_target_dependencies(get_force 
  rclcpp 
  geometry_msgs
  tf2
  Eigen3
)

# Install executables
install(TARGETS
  unity_control
  unity_control_v2
  robot_control
  joints_sub
  haptic_control
  get_force
  DESTINATION lib/${PROJECT_NAME}
)

rosidl_target_interfaces(haptic_control
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
target_link_libraries(haptic_control ur_rtde::rtde)
target_link_libraries(joints_sub ur_rtde::rtde)
target_include_directories(get_force PUBLIC ${EIGEN3_INCLUDE_DIRS})



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
