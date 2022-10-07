cmake_minimum_required(VERSION 3.5)
project(conix_demo)

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros2_unitree_legged_msgs REQUIRED)

# message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
else()
  set(ARCH arm64)
endif()
set(EXTRA_LIBS -pthread libunitree_legged_sdk_${ARCH}.so lcm)

set(CMAKE_CXX_FLAGS "-O3")

include_directories(
    include
    ${CMAKE_SOURCE_DIR}/unitree_legged_sdk-3.4.2/include
)

link_directories(${CMAKE_SOURCE_DIR}/unitree_legged_sdk-3.4.2/lib)

add_executable(state_publisher src/state_publisher.cpp)
ament_target_dependencies(state_publisher
  rclcpp ros2_unitree_legged_msgs)

add_executable(joint_sim src/joint_sim.cpp)
ament_target_dependencies(joint_sim
  rclcpp geometry_msgs sensor_msgs tf2 tf2_ros ros2_unitree_legged_msgs)

add_executable(ros2_udp src/ros2_udp.cpp)
target_link_libraries(ros2_udp ${EXTRA_LIBS})
ament_target_dependencies(ros2_udp rclcpp ros2_unitree_legged_msgs)

add_executable(ros2_walk_example src/ros2_walk_example.cpp)
target_link_libraries(ros2_walk_example ${EXTRA_LIBS})
ament_target_dependencies(ros2_walk_example rclcpp ros2_unitree_legged_msgs)

add_executable(cmd_interface src/cmd_interface.cpp)
target_link_libraries(cmd_interface ${EXTRA_LIBS})
ament_target_dependencies(cmd_interface rclcpp ros2_unitree_legged_msgs)

add_executable(state_to_joint_converter src/state_to_joint_converter.cpp)
target_link_libraries(state_to_joint_converter ${EXTRA_LIBS})
ament_target_dependencies(state_to_joint_converter sensor_msgs rclcpp ros2_unitree_legged_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  go1_description
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY 
    ${CMAKE_SOURCE_DIR}/unitree_legged_sdk-3.4.2/lib/
    DESTINATION lib
)

install(TARGETS
  state_publisher
  joint_sim
  ros2_udp
  ros2_walk_example
  cmd_interface
  state_to_joint_converter
  DESTINATION lib/${PROJECT_NAME})

ament_package()
