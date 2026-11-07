cmake_minimum_required(VERSION 3.8)
project(k1muse_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(k1muse_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d objdetect)
find_package(Eigen3 REQUIRED)

# cv_bridge selects the system OpenCV ABI for existing nodes. Keep its targets
# unchanged; the independent CV process uses the installed RVV build on K1.
set(_cv_opencv_includes ${OpenCV_INCLUDE_DIRS})
set(_cv_opencv_libraries opencv_core opencv_imgproc opencv_objdetect)
# Before OpenCV 4.7 ArUco lives in the contrib aruco library; K1 uses 4.14 objdetect.
if(NOT CMAKE_CROSSCOMPILING AND OpenCV_VERSION VERSION_LESS "4.7")
  find_library(K1MUSE_HOST_ARUCO opencv_aruco REQUIRED)
  list(APPEND _cv_opencv_libraries ${K1MUSE_HOST_ARUCO})
endif()
if(CMAKE_CROSSCOMPILING)
  find_path(K1MUSE_CV_OPENCV_INCLUDE opencv2/objdetect.hpp
    PATHS "${CMAKE_SYSROOT}/opt/opencv-spacemit/include/opencv4" NO_DEFAULT_PATH)
  if(NOT K1MUSE_CV_OPENCV_INCLUDE)
    message(FATAL_ERROR "CV requires the existing /opt/opencv-spacemit RVV headers in the sysroot")
  endif()
  set(_cv_opencv_includes ${K1MUSE_CV_OPENCV_INCLUDE})
  set(_cv_opencv_libraries "")
  foreach(_component core imgproc objdetect)
    find_library(K1MUSE_CV_OPENCV_${_component} NAMES opencv_${_component}
      PATHS "${CMAKE_SYSROOT}/opt/opencv-spacemit/lib" NO_DEFAULT_PATH)
    if(NOT K1MUSE_CV_OPENCV_${_component})
      message(FATAL_ERROR "CV requires the existing RVV opencv_${_component} in the sysroot")
    endif()
    list(APPEND _cv_opencv_libraries ${K1MUSE_CV_OPENCV_${_component}})
  endforeach()
endif()

# Pure CV is independent of the optional YOLO/ORT/SpaceMIT runtime targets.
add_library(${PROJECT_NAME}_cv
  src/qr_reader.cpp
  src/cv_perception.cpp
)
target_include_directories(${PROJECT_NAME}_cv PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${_cv_opencv_includes}
)
target_link_libraries(${PROJECT_NAME}_cv
  nlohmann_json::nlohmann_json ${_cv_opencv_libraries})
add_executable(cv_perception src/cv_perception_node.cpp)
target_link_libraries(cv_perception ${PROJECT_NAME}_cv)
ament_target_dependencies(cv_perception rclcpp rcutils rcl_interfaces sensor_msgs std_msgs std_srvs)
if(CMAKE_CROSSCOMPILING)
  set_target_properties(cv_perception PROPERTIES INSTALL_RPATH "/opt/opencv-spacemit/lib")
endif()
install(TARGETS ${PROJECT_NAME}_cv cv_perception
  EXPORT export_${PROJECT_NAME}_cv
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_library(${PROJECT_NAME}_target_transform src/target_transform_node.cpp)
target_include_directories(${PROJECT_NAME}_target_transform PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_target_transform
  rclcpp rcl_interfaces vision_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)
add_executable(target_transform src/target_transform_main.cpp)
target_link_libraries(target_transform ${PROJECT_NAME}_target_transform)
install(TARGETS ${PROJECT_NAME}_target_transform target_transform
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_library(${PROJECT_NAME}_depth_support
  src/depth_image_view.cpp
  src/depth_geometry.cpp
  src/depth_center_mean.cpp
  src/depth_yolo_weighted.cpp
)
target_include_directories(${PROJECT_NAME}_depth_support PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_depth_support sensor_msgs vision_msgs geometry_msgs)
target_link_libraries(${PROJECT_NAME}_depth_support opencv_core opencv_calib3d)

add_library(${PROJECT_NAME}_depth_node src/depth_fusion_node.cpp)
target_link_libraries(${PROJECT_NAME}_depth_node ${PROJECT_NAME}_depth_support)
ament_target_dependencies(${PROJECT_NAME}_depth_node
  rclcpp rclcpp_lifecycle rcl_interfaces sensor_msgs vision_msgs message_filters k1muse_interfaces nlohmann_json)

add_executable(depth_fusion src/depth_fusion_main.cpp)
target_link_libraries(depth_fusion ${PROJECT_NAME}_depth_node)
ament_target_dependencies(depth_fusion rclcpp)
install(TARGETS ${PROJECT_NAME}_depth_support ${PROJECT_NAME}_depth_node depth_fusion
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_library(${PROJECT_NAME}_yolo_support
  src/yolo_preprocess.cpp
  src/yolo_postprocess.cpp
)
target_include_directories(${PROJECT_NAME}_yolo_support PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME}_yolo_support PUBLIC ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}_yolo_support vision_msgs)
target_link_libraries(${PROJECT_NAME}_yolo_support opencv_core opencv_imgproc)

add_library(${PROJECT_NAME}_bytetrack_core STATIC
  src/bytetrack/BYTETracker.cpp
  src/bytetrack/STrack.cpp
  src/bytetrack/kalmanFilter.cpp
  src/bytetrack/lapjv.cpp
  src/bytetrack/utils.cpp
)
target_include_directories(${PROJECT_NAME}_bytetrack_core PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_bytetrack_core PUBLIC Eigen3::Eigen opencv_core)

add_library(${PROJECT_NAME}_byte_tracker src/byte_tracker.cpp)
target_include_directories(${PROJECT_NAME}_byte_tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_byte_tracker vision_msgs)
target_include_directories(${PROJECT_NAME}_byte_tracker PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_byte_tracker ${PROJECT_NAME}_bytetrack_core)

# Compile ROS integration even on a host without the target inference libraries.
# Linking the runnable detector still requires the official ORT and SpaceMIT EP.
add_library(${PROJECT_NAME}_yolo_node STATIC src/yolo_detector_node.cpp)
target_link_libraries(${PROJECT_NAME}_yolo_node
  nlohmann_json::nlohmann_json
  ${PROJECT_NAME}_yolo_support
  ${PROJECT_NAME}_byte_tracker
)
ament_target_dependencies(${PROJECT_NAME}_yolo_node
  rclcpp rcutils rcl_interfaces rclcpp_lifecycle lifecycle_msgs sensor_msgs std_srvs vision_msgs cv_bridge
  k1muse_interfaces)

# The generic preprocessing/NMS tests can run without K1's proprietary runtime.
# The real detector is only meaningful with both official target-side libraries.
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  PATH_SUFFIXES include onnxruntime/core/session)
find_path(SPACEMIT_EP_INCLUDE_DIR spacemit_ort_env.h
  PATH_SUFFIXES include onnxruntime/core/session)
find_library(ONNXRUNTIME_LIBRARY NAMES onnxruntime)
find_library(SPACEMIT_EP_LIBRARY NAMES spacemit_ep)

if(ONNXRUNTIME_INCLUDE_DIR AND SPACEMIT_EP_INCLUDE_DIR AND
  ONNXRUNTIME_LIBRARY AND SPACEMIT_EP_LIBRARY)
  add_library(${PROJECT_NAME}_yolo_engine src/yolo_engine.cpp)
  target_include_directories(${PROJECT_NAME}_yolo_engine PRIVATE
    ${ONNXRUNTIME_INCLUDE_DIR}
    ${SPACEMIT_EP_INCLUDE_DIR}
  )
  target_link_libraries(${PROJECT_NAME}_yolo_engine
    ${PROJECT_NAME}_yolo_support
    ${ONNXRUNTIME_LIBRARY}
    ${SPACEMIT_EP_LIBRARY}
  )

  add_library(${PROJECT_NAME}_ocr_engine src/ocr_engine.cpp)
  target_include_directories(${PROJECT_NAME}_ocr_engine PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS})
  target_include_directories(${PROJECT_NAME}_ocr_engine PRIVATE
    ${ONNXRUNTIME_INCLUDE_DIR} ${SPACEMIT_EP_INCLUDE_DIR})
  target_compile_options(${PROJECT_NAME}_ocr_engine PRIVATE -ffp-contract=off)
  target_link_libraries(${PROJECT_NAME}_ocr_engine
    opencv_core opencv_imgproc ${ONNXRUNTIME_LIBRARY} ${SPACEMIT_EP_LIBRARY})
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "riscv")
    # The general toolchain intentionally targets scalar RV64GC. Only this
    # K1 OCR kernel opts into RVV; preserve the rest of the workspace policy.
    target_compile_options(${PROJECT_NAME}_ocr_engine PRIVATE -march=rv64gcv_zba_zbb_zbc_zbs)
  endif()

  add_library(${PROJECT_NAME}_ocr_node src/ocr_node.cpp)
  target_link_libraries(${PROJECT_NAME}_ocr_node
    ${PROJECT_NAME}_ocr_engine nlohmann_json::nlohmann_json Threads::Threads)
  ament_target_dependencies(${PROJECT_NAME}_ocr_node
    rclcpp rcutils rcl_interfaces sensor_msgs std_msgs k1muse_interfaces)

  target_link_libraries(${PROJECT_NAME}_yolo_node
    ${PROJECT_NAME}_yolo_engine ${PROJECT_NAME}_ocr_node)

  add_executable(yolo_detector src/yolo_detector_main.cpp)
  target_link_libraries(yolo_detector ${PROJECT_NAME}_yolo_node)
  ament_target_dependencies(yolo_detector rclcpp)

  install(TARGETS
    ${PROJECT_NAME}_yolo_engine
    ${PROJECT_NAME}_ocr_engine
    ${PROJECT_NAME}_ocr_node
    ${PROJECT_NAME}_yolo_node
    yolo_detector
    EXPORT export_${PROJECT_NAME}_yolo
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING
    "K1 ONNX Runtime and SpaceMIT EP were not found: yolo_detector is not built. "
    "Install the official target runtime before building C08 on K1.")
endif()

install(TARGETS ${PROJECT_NAME}_yolo_support ${PROJECT_NAME}_byte_tracker ${PROJECT_NAME}_bytetrack_core
  EXPORT export_${PROJECT_NAME}_yolo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
# Incremental installs otherwise retain the removed QR geometry API headers.
install(CODE [[
  file(REMOVE
    "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/include/k1muse_perception/qr_pose_estimator.hpp"
    "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/include/k1muse_perception/qr_depth.hpp"
    "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/include/k1muse_perception/iou_tracker.hpp"
    "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libk1muse_perception_iou_tracker.a")
]])
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(DIRECTORY LICENSES DESTINATION share/${PROJECT_NAME})
install(FILES DEPTH_ATTRIBUTION.md OCR_ATTRIBUTION.md TRACKER_ATTRIBUTION.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_yolo_preprocess test/test_yolo_preprocess.cpp)
  target_link_libraries(test_yolo_preprocess ${PROJECT_NAME}_yolo_support)
  target_include_directories(test_yolo_preprocess PRIVATE ${OpenCV_INCLUDE_DIRS})

  ament_add_gtest(test_yolo_postprocess test/test_yolo_postprocess.cpp)
  target_link_libraries(test_yolo_postprocess ${PROJECT_NAME}_yolo_support)

  ament_add_gtest(test_byte_tracker test/test_byte_tracker.cpp)
  target_link_libraries(test_byte_tracker ${PROJECT_NAME}_byte_tracker)

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_yolo_launch test/test_yolo_launch.py)
  if(TARGET ${PROJECT_NAME}_yolo_engine)
    # Uses the actual runtime linkage; no fake engine or EP. No model is loaded by these tests.
    ament_add_gtest(test_yolo_perf test/test_yolo_perf.cpp)
    target_link_libraries(test_yolo_perf ${PROJECT_NAME}_yolo_node)
  endif()

  ament_add_gtest(test_qr test/test_qr.cpp)
  target_link_libraries(test_qr ${PROJECT_NAME}_cv)
  ament_target_dependencies(test_qr sensor_msgs vision_msgs geometry_msgs)

  ament_add_gtest(test_depth_estimation test/test_depth_estimation.cpp)
  target_link_libraries(test_depth_estimation ${PROJECT_NAME}_depth_support
    ${PROJECT_NAME}_yolo_support)
  ament_add_gtest(test_depth_fusion_node test/test_depth_fusion_node.cpp)
  target_link_libraries(test_depth_fusion_node ${PROJECT_NAME}_depth_node)
  ament_add_pytest_test(test_depth_launch test/test_depth_launch.py)
  ament_add_pytest_test(test_depth_startup test/test_depth_startup.py
    ENV DEPTH_FUSION_EXECUTABLE=$<TARGET_FILE:depth_fusion>)
  ament_add_gtest(test_target_transform test/test_target_transform.cpp)
  target_link_libraries(test_target_transform ${PROJECT_NAME}_target_transform
    ${PROJECT_NAME}_depth_node)
  ament_add_pytest_test(test_target_transform_launch test/test_target_transform_launch.py)
  ament_add_pytest_test(test_target_transform_startup test/test_target_transform_startup.py
    ENV TARGET_TRANSFORM_EXECUTABLE=$<TARGET_FILE:target_transform>)
endif()

ament_export_targets(export_${PROJECT_NAME}_yolo HAS_LIBRARY_TARGET)
ament_export_targets(export_${PROJECT_NAME}_cv HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp rcutils rcl_interfaces rclcpp_lifecycle lifecycle_msgs sensor_msgs std_msgs std_srvs
  vision_msgs cv_bridge k1muse_interfaces nlohmann_json OpenCV Eigen3 Threads)
ament_package()
