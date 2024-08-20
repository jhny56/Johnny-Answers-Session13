cmake_minimum_required(VERSION 3.8)
project(ball_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(ball_follower_node src/ball_follower_node.cpp)
add_executable(initNode src/initNode.cpp)

target_include_directories(initNode PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(initNode PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS initNode
  DESTINATION lib/${PROJECT_NAME})

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

# Link against ROS2 packages
ament_target_dependencies(ball_follower_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
)
# Link libraries
target_link_libraries(ball_follower_node
  ${OpenCV_LIBRARIES}
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)
# Install the executable
install(TARGETS ball_follower_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files or other files if needed
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
