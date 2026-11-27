cmake_minimum_required(VERSION 3.16)
project(uav_vio CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra)
# Tune for the Pi 5 when building on it; the same code builds on x86 for replay and metrics.
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  add_compile_options(-mcpu=cortex-a76)
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc video imgcodecs calib3d)

add_library(vio_core
  src/session.cpp
  src/imu_propagation.cpp
  src/camera_model.cpp
  src/frontend.cpp
  src/filter.cpp
  src/config.cpp
  src/msckf.cpp
  src/quality.cpp
  src/inject.cpp
)
target_include_directories(vio_core PUBLIC include)
target_link_libraries(vio_core PUBLIC Eigen3::Eigen yaml-cpp opencv_core opencv_imgproc opencv_video)

add_executable(vio_replay src/tools/vio_replay.cpp)
target_link_libraries(vio_replay PRIVATE vio_core opencv_imgcodecs)

add_executable(vio_track src/tools/vio_track.cpp)
target_link_libraries(vio_track PRIVATE vio_core opencv_imgcodecs)

enable_testing()
add_executable(test_imu_propagation tests/test_imu_propagation.cpp)
target_link_libraries(test_imu_propagation PRIVATE vio_core)
add_test(NAME imu_propagation COMMAND test_imu_propagation)
# calib3d only for the comparison with OpenCV's camera models.
add_executable(test_camera_models tests/test_camera_models.cpp)
target_link_libraries(test_camera_models PRIVATE vio_core opencv_core opencv_calib3d)
add_test(NAME camera_models COMMAND test_camera_models)
add_executable(test_frontend tests/test_frontend.cpp)
target_link_libraries(test_frontend PRIVATE vio_core)
add_test(NAME frontend COMMAND test_frontend)
add_executable(test_filter tests/test_filter.cpp)
target_link_libraries(test_filter PRIVATE vio_core)
add_test(NAME filter COMMAND test_filter)
add_executable(test_quality tests/test_quality.cpp)
target_link_libraries(test_quality PRIVATE vio_core)
add_test(NAME quality COMMAND test_quality)
add_executable(test_inject tests/test_inject.cpp)
target_link_libraries(test_inject PRIVATE vio_core)
add_test(NAME inject COMMAND test_inject)
add_test(NAME metrics COMMAND python3 ${CMAKE_SOURCE_DIR}/tools/test_metrics.py)
