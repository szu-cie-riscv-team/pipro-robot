cmake_minimum_required(VERSION 3.8)
project(k1muse_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(CURL REQUIRED IMPORTED_TARGET libcurl)
find_package(Threads REQUIRED)
find_package(k1muse_interfaces REQUIRED)
find_package(k1muse_voice REQUIRED)
find_package(k1muse_perception REQUIRED)

add_library(${PROJECT_NAME}_drug_catalog src/drug_catalog.cpp)
target_compile_features(${PROJECT_NAME}_drug_catalog PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_drug_catalog PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_drug_catalog nlohmann_json::nlohmann_json yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}_drug_catalog ament_index_cpp)

add_library(${PROJECT_NAME}_node
  src/task_manager_node.cpp src/task_manager_navigation.cpp src/task_manager_fetch.cpp
  src/task_manager_motion.cpp src/task_manager_vision.cpp src/front_grasp_plan.cpp src/top_grasp_plan.cpp src/inference_control.cpp
  src/task_manager_survey.cpp src/survey_counter.cpp src/inventory_store.cpp
  src/warehouse_config.cpp src/grasp_lift_ik.cpp vendor/so101-ik/so101_algorithm.c)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_node nlohmann_json::nlohmann_json yaml-cpp ${PROJECT_NAME}_drug_catalog)
target_include_directories(${PROJECT_NAME}_node PRIVATE vendor/so101-ik)
target_link_libraries(${PROJECT_NAME}_node m)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp rcl_interfaces rclcpp_action action_msgs geometry_msgs nav_msgs sensor_msgs vision_msgs nav2_msgs k1muse_interfaces std_srvs std_msgs tf2 tf2_ros)

add_executable(task_manager_node src/task_manager_main.cpp)
target_link_libraries(task_manager_node ${PROJECT_NAME}_node)
ament_target_dependencies(task_manager_node rclcpp)

add_library(${PROJECT_NAME}_voice_local src/voice_local.cpp)
target_compile_features(${PROJECT_NAME}_voice_local PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_voice_local PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
find_path(SPACEMIT_LLAMA_INCLUDE_DIR llama.h)
find_library(SPACEMIT_LLAMA_LIBRARY NAMES llama)
find_library(SPACEMIT_LLAMA_COMMON_LIBRARY NAMES llama-common)
find_library(SPACEMIT_GGML_CPU_LIBRARY NAMES ggml-cpu)
find_library(SPACEMIT_GGML_LIBRARY NAMES ggml)
find_library(SPACEMIT_GGML_BASE_LIBRARY NAMES ggml-base)
if(SPACEMIT_LLAMA_INCLUDE_DIR AND SPACEMIT_LLAMA_LIBRARY AND
  SPACEMIT_LLAMA_COMMON_LIBRARY AND SPACEMIT_GGML_CPU_LIBRARY AND
  SPACEMIT_GGML_LIBRARY AND SPACEMIT_GGML_BASE_LIBRARY)
  target_compile_definitions(${PROJECT_NAME}_voice_local PRIVATE K1MUSE_HAS_LOCAL_LLM)
  # common is built against JSON 3.12. Keep its headers/types private to this TU,
  # ahead of the system JSON headers used by the existing business parser.
  set(_llama_headers "${CMAKE_CURRENT_SOURCE_DIR}/vendor/spacemit-llama-0.2.2")
  target_include_directories(${PROJECT_NAME}_voice_local SYSTEM PRIVATE
    "${_llama_headers}/vendor" "${_llama_headers}/common" ${SPACEMIT_LLAMA_INCLUDE_DIR})
  target_link_libraries(${PROJECT_NAME}_voice_local PRIVATE
    ${SPACEMIT_LLAMA_LIBRARY} ${SPACEMIT_LLAMA_COMMON_LIBRARY}
    ${SPACEMIT_GGML_CPU_LIBRARY} ${SPACEMIT_GGML_LIBRARY} ${SPACEMIT_GGML_BASE_LIBRARY}
    Threads::Threads)
elseif(CMAKE_CROSSCOMPILING OR CMAKE_SYSTEM_PROCESSOR MATCHES "riscv")
  message(FATAL_ERROR "Local intent requires the matching SpaceMIT llama/ggml/common SDK.")
else()
  message(WARNING "Local LLM unavailable: SpaceMIT llama/ggml/common SDK was not found.")
endif()

add_library(${PROJECT_NAME}_voice_config src/voice_intent.cpp)
target_compile_features(${PROJECT_NAME}_voice_config PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_voice_config PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_voice_config nlohmann_json::nlohmann_json yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}_voice_config ament_index_cpp)
add_executable(voice_config src/voice_config_main.cpp)
target_link_libraries(voice_config ${PROJECT_NAME}_voice_config)
install(TARGETS voice_config RUNTIME DESTINATION lib/${PROJECT_NAME})

add_library(${PROJECT_NAME}_voice_intent src/voice_cloud.cpp)
target_compile_features(${PROJECT_NAME}_voice_intent PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_voice_intent PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_voice_intent
  PUBLIC nlohmann_json::nlohmann_json ${PROJECT_NAME}_voice_config
  PRIVATE yaml-cpp PkgConfig::CURL Threads::Threads ${PROJECT_NAME}_voice_local)

add_library(${PROJECT_NAME}_voice_command src/voice_command_node.cpp)
target_compile_features(${PROJECT_NAME}_voice_command PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_voice_command PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_voice_command yaml-cpp ${PROJECT_NAME}_voice_intent ${PROJECT_NAME}_drug_catalog)
ament_target_dependencies(${PROJECT_NAME}_voice_command
  rclcpp rclcpp_action std_msgs std_srvs k1muse_interfaces)
add_executable(voice_command src/voice_command_main.cpp)
target_link_libraries(voice_command ${PROJECT_NAME}_voice_command)
ament_target_dependencies(voice_command rclcpp)

if(TARGET k1muse_voice::k1muse_voice_node AND
  TARGET k1muse_perception::k1muse_perception_yolo_node)
  add_executable(ai_inference src/ai_inference_main.cpp)
  target_compile_features(ai_inference PRIVATE cxx_std_17)
  target_link_libraries(ai_inference
    ${PROJECT_NAME}_voice_command
    k1muse_voice::k1muse_voice_node
    k1muse_perception::k1muse_perception_yolo_node)
  ament_target_dependencies(ai_inference rclcpp)
  install(TARGETS ai_inference RUNTIME DESTINATION lib/${PROJECT_NAME})
elseif(CMAKE_CROSSCOMPILING OR CMAKE_SYSTEM_PROCESSOR MATCHES "riscv")
  message(FATAL_ERROR
    "ai_inference requires the real exported voice_input and YOLO node libraries.")
else()
  message(WARNING "ai_inference is not built: target model node libraries are unavailable.")
endif()

install(TARGETS ${PROJECT_NAME}_voice_command ${PROJECT_NAME}_voice_intent
  ${PROJECT_NAME}_voice_local ${PROJECT_NAME}_drug_catalog ${PROJECT_NAME}_voice_config
  EXPORT export_${PROJECT_NAME}_voice
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(TARGETS ${PROJECT_NAME}_node task_manager_node
  voice_command
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config
  PATTERN "drug_catalog.yaml" EXCLUDE)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(PROGRAMS scripts/task_debug.py DESTINATION lib/${PROJECT_NAME})

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
  ament_add_gtest(test_drug_catalog test/test_drug_catalog.cpp)
  target_link_libraries(test_drug_catalog ${PROJECT_NAME}_drug_catalog)
  target_compile_definitions(test_drug_catalog PRIVATE
    K1MUSE_CATALOG_FIXTURE="${CMAKE_CURRENT_SOURCE_DIR}/test/data/drug_catalog_cases.json")
  ament_add_gtest(test_survey_inventory test/test_survey_inventory.cpp
    src/survey_counter.cpp src/inventory_store.cpp)
  target_compile_features(test_survey_inventory PRIVATE cxx_std_17)
  target_include_directories(test_survey_inventory PRIVATE include)
  target_link_libraries(test_survey_inventory nlohmann_json::nlohmann_json)
  ament_add_gtest(test_survey_task test/test_survey_task.cpp TIMEOUT 240)
  target_compile_definitions(test_survey_task PRIVATE K1MUSE_MANAGER_TEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  target_link_libraries(test_survey_task ${PROJECT_NAME}_node)
  ament_target_dependencies(test_survey_task rclcpp rclcpp_action tf2_ros nav2_msgs k1muse_interfaces)
  ament_add_gtest(test_warehouse_config test/test_warehouse_config.cpp src/warehouse_config.cpp)
  target_compile_features(test_warehouse_config PRIVATE cxx_std_17)
  target_include_directories(test_warehouse_config PRIVATE include)
  target_link_libraries(test_warehouse_config nlohmann_json::nlohmann_json)
  target_compile_definitions(test_warehouse_config PRIVATE
    K1MUSE_WAREHOUSE_FIXTURE="${CMAKE_CURRENT_SOURCE_DIR}/test/data/warehouse_cases.json")
  find_package(action_msgs REQUIRED)
  ament_add_gtest(test_task_manager test/test_task_manager.cpp TIMEOUT 30)
  target_link_libraries(test_task_manager ${PROJECT_NAME}_node)
  ament_target_dependencies(test_task_manager rclcpp rclcpp_action nav2_msgs k1muse_interfaces)
  ament_add_gtest(test_inference_control test/test_inference_control.cpp TIMEOUT 30)
  target_compile_definitions(test_inference_control PRIVATE
    K1MUSE_MANAGER_TEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  target_link_libraries(test_inference_control ${PROJECT_NAME}_node ${PROJECT_NAME}_voice_command)
  ament_target_dependencies(test_inference_control rclcpp rclcpp_action std_msgs std_srvs k1muse_interfaces)
  ament_add_gtest(test_voice_command test/test_voice_command.cpp TIMEOUT 30)
  target_compile_definitions(test_voice_command PRIVATE
    K1MUSE_MANAGER_TEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  target_link_libraries(test_voice_command ${PROJECT_NAME}_voice_command)
  ament_target_dependencies(test_voice_command
    rclcpp rclcpp_action std_msgs std_srvs action_msgs k1muse_interfaces)
  ament_add_gtest(test_voice_intent test/test_voice_intent.cpp TIMEOUT 30)
  target_link_libraries(test_voice_intent ${PROJECT_NAME}_voice_intent)
  ament_add_gtest(test_voice_cloud_node test/test_voice_cloud_node.cpp TIMEOUT 60)
  target_compile_definitions(test_voice_cloud_node PRIVATE
    K1MUSE_MANAGER_TEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  target_link_libraries(test_voice_cloud_node ${PROJECT_NAME}_voice_command)
  ament_target_dependencies(test_voice_cloud_node rclcpp rclcpp_action std_msgs k1muse_interfaces)
  ament_add_gtest(test_voice_restart test/test_voice_restart.cpp TIMEOUT 30)
  target_link_libraries(test_voice_restart ${PROJECT_NAME}_voice_command)
  ament_target_dependencies(test_voice_restart rclcpp k1muse_interfaces std_msgs)
  ament_add_gtest(test_grasp_plan test/test_grasp_plan.cpp)
  target_link_libraries(test_grasp_plan ${PROJECT_NAME}_node)
  ament_add_gtest(test_fetch_task test/test_fetch_task.cpp TIMEOUT 90)
  target_compile_definitions(test_fetch_task PRIVATE
    K1MUSE_WAREHOUSE_FIXTURE="${CMAKE_CURRENT_SOURCE_DIR}/test/data/warehouse_cases.json"
    K1MUSE_MANAGER_TEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  target_link_libraries(test_fetch_task ${PROJECT_NAME}_node)
  ament_target_dependencies(test_fetch_task rclcpp rclcpp_action nav_msgs vision_msgs k1muse_interfaces)

endif()

ament_export_targets(export_${PROJECT_NAME}_voice HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_action std_msgs std_srvs k1muse_interfaces
  nlohmann_json yaml-cpp ament_index_cpp)
ament_package(CONFIG_EXTRAS cmake/k1muse_manager-extras.cmake)
