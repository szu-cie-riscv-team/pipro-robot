cmake_minimum_required(VERSION 3.8)
project(k1muse_communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(k1muse_interfaces REQUIRED)

add_library(${PROJECT_NAME}_protocol src/protocol.cpp)
target_include_directories(${PROJECT_NAME}_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(${PROJECT_NAME}_node src/stm32_communication_node.cpp)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_protocol)
ament_target_dependencies(
  ${PROJECT_NAME}_node rclcpp rclcpp_action std_msgs std_srvs geometry_msgs k1muse_interfaces)

add_executable(stm32_communication src/stm32_communication_main.cpp)
target_link_libraries(stm32_communication ${PROJECT_NAME}_node)
ament_target_dependencies(stm32_communication rclcpp)

add_library(${PROJECT_NAME}_odometry src/chassis_odometry.cpp)
target_include_directories(${PROJECT_NAME}_odometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  ${PROJECT_NAME}_odometry geometry_msgs nav_msgs tf2 tf2_geometry_msgs k1muse_interfaces)

add_executable(chassis_odometry src/chassis_odometry_node.cpp)
target_link_libraries(chassis_odometry ${PROJECT_NAME}_odometry)
ament_target_dependencies(chassis_odometry rclcpp nav_msgs tf2_ros k1muse_interfaces)

install(TARGETS ${PROJECT_NAME}_protocol ${PROJECT_NAME}_node ${PROJECT_NAME}_odometry
  stm32_communication chassis_odometry
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

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

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol ${PROJECT_NAME}_protocol)

  ament_add_gtest(test_fake_stm32 test/test_fake_stm32.cpp TIMEOUT 30)
  target_link_libraries(test_fake_stm32 ${PROJECT_NAME}_node)
  ament_target_dependencies(test_fake_stm32 rclcpp rclcpp_action geometry_msgs k1muse_interfaces)
  find_library(UTIL_LIBRARY util REQUIRED)
  target_link_libraries(test_fake_stm32 ${UTIL_LIBRARY})

  ament_add_gtest(test_chassis_odometry test/test_chassis_odometry.cpp)
  target_link_libraries(test_chassis_odometry ${PROJECT_NAME}_odometry)
  ament_target_dependencies(test_chassis_odometry geometry_msgs nav_msgs tf2 tf2_geometry_msgs k1muse_interfaces)
endif()

ament_package()
