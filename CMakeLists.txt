cmake_minimum_required(VERSION 3.21)
project(AeroForge VERSION 0.1.0 LANGUAGES CXX)

# ──────────────────────────────────────────────────────────────────────────
# Project options
# ──────────────────────────────────────────────────────────────────────────

option(AEROFORGE_WITH_DJI "Enable DJI SDK integration" OFF)
option(AEROFORGE_WITH_IMGUI "Enable ImGui HUD" ON)
option(AEROFORGE_ENABLE_SANITIZERS "Enable AddressSanitizer and UBSan (Debug only)" OFF)
option(AEROFORGE_BUILD_TESTS "Build unit tests" ON)
option(AEROFORGE_BUILD_APPS "Build applications" ON)

# ──────────────────────────────────────────────────────────────────────────
# C++ standard
# ──────────────────────────────────────────────────────────────────────────

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile commands for clangd/clang-tidy
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ──────────────────────────────────────────────────────────────────────────
# CMake modules
# ──────────────────────────────────────────────────────────────────────────

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# ──────────────────────────────────────────────────────────────────────────
# Compiler warnings
# ──────────────────────────────────────────────────────────────────────────

if(MSVC)
  add_compile_options(/W4 /WX)
  # Disable specific warnings that are too noisy
  add_compile_options(/wd4127) # conditional expression is constant
  add_compile_options(/utf-8)  # UTF-8 source and execution
else()
  add_compile_options(-Wall -Wextra -Werror -pedantic)
  add_compile_options(-Wno-unused-parameter)
endif()

# ──────────────────────────────────────────────────────────────────────────
# Sanitizers (Debug builds only)
# ──────────────────────────────────────────────────────────────────────────

if(AEROFORGE_ENABLE_SANITIZERS AND CMAKE_BUILD_TYPE STREQUAL "Debug")
  if(NOT MSVC)
    add_compile_options(-fsanitize=address -fsanitize=undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address -fsanitize=undefined)
    message(STATUS "Sanitizers enabled: AddressSanitizer and UBSan")
  else()
    message(WARNING "Sanitizers not supported on MSVC")
  endif()
endif()

# ──────────────────────────────────────────────────────────────────────────
# Dependencies
# ──────────────────────────────────────────────────────────────────────────

find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio highgui)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)

if(AEROFORGE_WITH_IMGUI)
  find_package(imgui REQUIRED)
  find_package(glfw3 REQUIRED)
  find_package(OpenGL REQUIRED)
endif()

if(AEROFORGE_BUILD_TESTS)
  find_package(Catch2 3 REQUIRED)
  include(CTest)
  enable_testing()
endif()

# ──────────────────────────────────────────────────────────────────────────
# Core library
# ──────────────────────────────────────────────────────────────────────────

add_library(af_core STATIC
  # Core
  src/core/frame_pool.cpp
  src/core/timing.cpp

  # Math
  src/math/pid.cpp
  src/math/kalman.cpp

  # Config
  src/config/loader.cpp

  # IO
  src/io/webcam_capture.cpp

  # Mock drone
  src/dji/mock_drone.cpp
)

target_include_directories(af_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(af_core
  PUBLIC
    opencv_core
    opencv_imgproc
    opencv_videoio
    # opencv_aruco  # Temporarily disabled
    Eigen3::Eigen
    fmt::fmt
    spdlog::spdlog
    yaml-cpp
)

target_compile_definitions(af_core
  PUBLIC
    $<$<BOOL:${AEROFORGE_WITH_DJI}>:AEROFORGE_WITH_DJI>
    $<$<BOOL:${AEROFORGE_WITH_IMGUI}>:AEROFORGE_WITH_IMGUI>
)

# ──────────────────────────────────────────────────────────────────────────
# UI library (optional, if ImGui enabled)
# ──────────────────────────────────────────────────────────────────────────

if(AEROFORGE_WITH_IMGUI)
  add_library(af_ui STATIC
    src/ui/hud.cpp
  )

  target_include_directories(af_ui
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(af_ui
    PUBLIC
      af_core
      imgui::imgui
      glfw
      OpenGL::GL
  )
endif()

# ──────────────────────────────────────────────────────────────────────────
# Vision library (detectors)
# ──────────────────────────────────────────────────────────────────────────

add_library(af_vision STATIC
  src/vision/detector_colorball.cpp
  # src/vision/detector_aruco.cpp  # Temporarily disabled - requires opencv_contrib
)

target_include_directories(af_vision
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(af_vision
  PUBLIC
    af_core
    opencv_imgproc
    # opencv_aruco  # Temporarily disabled
)

# ──────────────────────────────────────────────────────────────────────────
# Tracking library
# ──────────────────────────────────────────────────────────────────────────

add_library(af_tracking STATIC
  src/tracking/kalman_tracker.cpp
)

target_include_directories(af_tracking
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(af_tracking
  PUBLIC
    af_core
)

# ──────────────────────────────────────────────────────────────────────────
# Estimation library
# ──────────────────────────────────────────────────────────────────────────

add_library(af_estimate STATIC
  src/estimate/estimator_size_based.cpp
  src/estimate/estimator_ground_plane.cpp
)

target_include_directories(af_estimate
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(af_estimate
  PUBLIC
    af_core
)

# ──────────────────────────────────────────────────────────────────────────
# Control library
# ──────────────────────────────────────────────────────────────────────────

add_library(af_control STATIC
  src/control/controller_pid_velocity.cpp
)

target_include_directories(af_control
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(af_control
  PUBLIC
    af_core
)

# ──────────────────────────────────────────────────────────────────────────
# DJI library (optional, gated by build flag)
# ──────────────────────────────────────────────────────────────────────────

if(AEROFORGE_WITH_DJI)
  add_library(af_dji STATIC
    src/dji/dji_sdk_wrapper.cpp
  )

  target_include_directories(af_dji
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(af_dji
    PUBLIC
      af_core
      # TODO: Add DJI SDK link when available
  )
endif()

# ──────────────────────────────────────────────────────────────────────────
# Applications
# ──────────────────────────────────────────────────────────────────────────

if(AEROFORGE_BUILD_APPS)
  add_subdirectory(apps)
endif()

# ──────────────────────────────────────────────────────────────────────────
# Tests
# ──────────────────────────────────────────────────────────────────────────

if(AEROFORGE_BUILD_TESTS)
  add_subdirectory(tests)
endif()

# ──────────────────────────────────────────────────────────────────────────
# Installation
# ──────────────────────────────────────────────────────────────────────────

include(GNUInstallDirs)

install(TARGETS af_core af_vision af_tracking af_estimate af_control
  EXPORT AeroForgeTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(AEROFORGE_WITH_IMGUI)
  install(TARGETS af_ui
    EXPORT AeroForgeTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  )
endif()

install(DIRECTORY include/aeroforge
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# ──────────────────────────────────────────────────────────────────────────
# Status summary
# ──────────────────────────────────────────────────────────────────────────

message(STATUS "")
message(STATUS "╔════════════════════════════════════════════════════════╗")
message(STATUS "║           AeroForge ${PROJECT_VERSION} Configuration           ║")
message(STATUS "╚════════════════════════════════════════════════════════╝")
message(STATUS "Build type:        ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard:      C++${CMAKE_CXX_STANDARD}")
message(STATUS "Install prefix:    ${CMAKE_INSTALL_PREFIX}")
message(STATUS "")
message(STATUS "Options:")
message(STATUS "  DJI SDK:         ${AEROFORGE_WITH_DJI}")
message(STATUS "  ImGui HUD:       ${AEROFORGE_WITH_IMGUI}")
message(STATUS "  Sanitizers:      ${AEROFORGE_ENABLE_SANITIZERS}")
message(STATUS "  Build tests:     ${AEROFORGE_BUILD_TESTS}")
message(STATUS "  Build apps:      ${AEROFORGE_BUILD_APPS}")
message(STATUS "")
message(STATUS "Dependencies:")
message(STATUS "  OpenCV:          ${OpenCV_VERSION}")
message(STATUS "  Eigen3:          ${Eigen3_VERSION}")
message(STATUS "  fmt:             ${fmt_VERSION}")
message(STATUS "  spdlog:          ${spdlog_VERSION}")
message(STATUS "  yaml-cpp:        ${yaml-cpp_VERSION}")
if(AEROFORGE_WITH_IMGUI)
  message(STATUS "  imgui:           found")
  message(STATUS "  glfw3:           found")
endif()
message(STATUS "")
