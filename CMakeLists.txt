# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, Sadie Forbes

cmake_minimum_required(VERSION 3.20)
project(Nlaser VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

option(NLASER_WITH_NET     "Build the UDP transport (ncad_laser_net)" ON)
option(NLASER_BUILD_TESTS  "Build the test suite"                    ON)

# NotoCAD's restrained dialect, enforced the same way: no RTTI, loud warnings.
add_library(nlaser_flags INTERFACE)
target_compile_options(nlaser_flags INTERFACE
  $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall;-Wextra;-Wpedantic;-Wshadow;-Wconversion;-fno-rtti>
  $<$<CXX_COMPILER_ID:MSVC>:/W4;/GR->
)

# ncad_laser_core: headless. Job, planner, encoder, decoder. Never includes a
# socket header -- that boundary is the reason it is its own target.
add_subdirectory(src/laser)
add_subdirectory(src/tools)

if(NLASER_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

message(STATUS "Nlaser ${PROJECT_VERSION}  build=${CMAKE_BUILD_TYPE}  net=${NLASER_WITH_NET}")
