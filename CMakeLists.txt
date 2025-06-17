cmake_minimum_required(VERSION 3.8)
project(hurocore)

find_package(ament_cmake REQUIRED)

# ROS2 Dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Unitree ROS2 messages
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)

set(
  DEPENDENCY_LIST
  unitree_go
  unitree_hg
  unitree_api
  rclcpp
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
)

# Custom message types
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/MotorMode.msg"
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# include_directories(include)
set(
  SRCS
  src/root_node.cc
  src/motor_crc_hg.cc
)

add_executable(root src/root.cc ${SRCS})
target_include_directories(root PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(root "${cpp_typesupport_target}")
ament_target_dependencies(root ${DEPENDENCY_LIST})
target_compile_options(root PUBLIC "-Wall;-Wextra;-O3;")
target_compile_features(root PUBLIC c_std_99 cxx_std_17)

install(TARGETS
  root
  DESTINATION lib/${PROJECT_NAME}
)

# Install the config directory
install(
  DIRECTORY config
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
