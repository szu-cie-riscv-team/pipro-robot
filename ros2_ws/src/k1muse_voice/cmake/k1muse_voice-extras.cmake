# Static SDK consumers need these imported targets before loading our exports.
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
pkg_check_modules(ALSA REQUIRED IMPORTED_TARGET alsa)
pkg_check_modules(CURL REQUIRED IMPORTED_TARGET libcurl)
pkg_check_modules(FFTW3F REQUIRED IMPORTED_TARGET fftw3f)
pkg_check_modules(SNDFILE REQUIRED IMPORTED_TARGET sndfile)

# The optional target exists only when the fixed target SDK was installed.
if(EXISTS "${CMAKE_SYSROOT}/opt/k1muse-deps/sherpa-onnx/1.13.1/lib/cmake/K1MuseSherpa/K1MuseSherpaConfig.cmake")
  find_package(K1MuseSherpa REQUIRED CONFIG
    PATHS "${CMAKE_SYSROOT}/opt/k1muse-deps/sherpa-onnx/1.13.1/lib/cmake/K1MuseSherpa"
    NO_DEFAULT_PATH)
endif()
