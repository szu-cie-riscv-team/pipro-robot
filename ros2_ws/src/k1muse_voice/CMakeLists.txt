cmake_minimum_required(VERSION 3.8)
project(k1muse_voice)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(k1muse_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(${PROJECT_NAME}_core src/voice_input_core.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# The runnable node uses the real target SDK. Core buffering/text tests remain
# available when the RISC-V SpaceMIT libraries are absent on the host.
set(SPACEMIT_AI_SDK_ROOT "" CACHE PATH
  "SpaceMIT AI SDK install root containing include/ and lib/")
set(SPACEMIT_VOICE_EXTRA_LIBRARIES "" CACHE STRING
  "Additional target-side libraries required by the installed SpaceMIT SDK")
set(_spacemit_hints)
if(SPACEMIT_AI_SDK_ROOT)
  list(APPEND _spacemit_hints "${SPACEMIT_AI_SDK_ROOT}")
endif()
list(APPEND _spacemit_hints
  "/home/bianbu/AI_SDK/ai-sdk/output/staging"
  "/usr" "/usr/local")

find_path(SPACEMIT_VOICE_INCLUDE_DIR asr_service.h
  HINTS ${_spacemit_hints} PATH_SUFFIXES include)
find_path(SPACEMIT_VAD_INCLUDE_DIR vad_service.h
  HINTS ${_spacemit_hints} PATH_SUFFIXES include)
find_library(SPACEMIT_ASR_LIBRARY NAMES asr stt
  HINTS ${_spacemit_hints} PATH_SUFFIXES lib lib64)
find_library(SPACEMIT_SENSEVOICE_LIBRARY NAMES sensevoice
  HINTS ${_spacemit_hints} PATH_SUFFIXES lib lib64)
find_library(SPACEMIT_VAD_LIBRARY NAMES vad
  HINTS ${_spacemit_hints} PATH_SUFFIXES lib lib64)
find_library(SPACEMIT_SILERO_LIBRARY NAMES silero_vad
  HINTS ${_spacemit_hints} PATH_SUFFIXES lib lib64)
find_library(ONNXRUNTIME_LIBRARY NAMES onnxruntime
  HINTS ${_spacemit_hints} PATH_SUFFIXES lib lib64)

# The current official libasr archive links both compiled ASR backends, so its
# installed Zipformer dependencies are required even when SenseVoice is chosen.
foreach(_name zipformer kaldi-native-fbank-core kissfft-float)
  string(MAKE_C_IDENTIFIER "${_name}" _id)
  string(TOUPPER "${_id}" _id)
  find_library(SPACEMIT_${_id}_LIBRARY NAMES ${_name}
    HINTS ${_spacemit_hints} PATH_SUFFIXES lib lib64)
  list(APPEND _spacemit_asr_libraries ${SPACEMIT_${_id}_LIBRARY})
endforeach()
find_library(SPACEMIT_EP_LIBRARY NAMES spacemit_ep
  HINTS ${_spacemit_hints} PATH_SUFFIXES lib lib64)
if(SPACEMIT_EP_LIBRARY)
  list(APPEND _spacemit_optional_libraries ${SPACEMIT_EP_LIBRARY})
endif()

find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(ALSA QUIET IMPORTED_TARGET alsa)
  pkg_check_modules(CURL QUIET IMPORTED_TARGET libcurl)
  pkg_check_modules(FFTW3F QUIET IMPORTED_TARGET fftw3f)
  pkg_check_modules(SNDFILE QUIET IMPORTED_TARGET sndfile)
endif()
find_package(Threads QUIET)
find_package(ament_index_cpp REQUIRED)
find_package(K1MuseSherpa QUIET CONFIG
  PATHS "${CMAKE_SYSROOT}/opt/k1muse-deps/sherpa-onnx/1.13.1/lib/cmake/K1MuseSherpa"
  NO_DEFAULT_PATH)

if(SPACEMIT_VOICE_INCLUDE_DIR AND SPACEMIT_VAD_INCLUDE_DIR AND
  SPACEMIT_ASR_LIBRARY AND SPACEMIT_SENSEVOICE_LIBRARY AND
  SPACEMIT_ZIPFORMER_LIBRARY AND SPACEMIT_KALDI_NATIVE_FBANK_CORE_LIBRARY AND
  SPACEMIT_KISSFFT_FLOAT_LIBRARY AND
  SPACEMIT_VAD_LIBRARY AND SPACEMIT_SILERO_LIBRARY AND
  ONNXRUNTIME_LIBRARY AND ALSA_FOUND AND CURL_FOUND AND FFTW3F_FOUND AND
  SNDFILE_FOUND AND Threads_FOUND AND K1MuseSherpa_FOUND)
  add_library(${PROJECT_NAME}_node src/voice_input_node.cpp src/voice_kws.cpp)
  target_link_options(${PROJECT_NAME}_node INTERFACE -Wl,--gc-sections)
  target_include_directories(${PROJECT_NAME}_node SYSTEM PRIVATE
    ${SPACEMIT_VOICE_INCLUDE_DIR} ${SPACEMIT_VAD_INCLUDE_DIR})
  target_link_libraries(${PROJECT_NAME}_node
    PUBLIC ${PROJECT_NAME}_core
    PRIVATE nlohmann_json::nlohmann_json K1Muse::SherpaKws
    PRIVATE PkgConfig::ALSA
    "-Wl,--start-group"
    ${SPACEMIT_ASR_LIBRARY}
    ${SPACEMIT_SENSEVOICE_LIBRARY}
    ${_spacemit_asr_libraries}
    ${SPACEMIT_VAD_LIBRARY}
    ${SPACEMIT_SILERO_LIBRARY}
    "-Wl,--end-group"
    ${ONNXRUNTIME_LIBRARY}
    ${_spacemit_optional_libraries}
    ${SPACEMIT_VOICE_EXTRA_LIBRARIES}
    PkgConfig::CURL PkgConfig::FFTW3F PkgConfig::SNDFILE
    Threads::Threads ${CMAKE_DL_LIBS} m)
  ament_target_dependencies(${PROJECT_NAME}_node PUBLIC rclcpp rcl_interfaces std_msgs std_srvs k1muse_interfaces ament_index_cpp)

  add_executable(voice_input src/voice_input_main.cpp)
  target_link_libraries(voice_input ${PROJECT_NAME}_node)
  ament_target_dependencies(voice_input rclcpp)
  install(TARGETS ${PROJECT_NAME}_node voice_input
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING
    "C14 voice_input is not built: ALSA or the complete SpaceMIT VAD/ASR "
    "SDK or fixed sherpa KWS dependency was not found. Set SPACEMIT_AI_SDK_ROOT on the K1 target.")
endif()

install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "voice_intent.yaml" EXCLUDE
  PATTERN "cloud_api.yaml" EXCLUDE
  PATTERN "cloud_prompt.yaml" EXCLUDE
  PATTERN "local_llm.yaml" EXCLUDE
  PATTERN "local_prompt.yaml" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_voice_input_core test/test_voice_input_core.cpp)
  target_link_libraries(test_voice_input_core ${PROJECT_NAME}_core)

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_voice_input_launch test/test_voice_input_launch.py)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_index_cpp rclcpp rcl_interfaces std_msgs std_srvs k1muse_interfaces nlohmann_json)
ament_package(CONFIG_EXTRAS cmake/k1muse_voice-extras.cmake)
