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
option(NLASER_BUILD_TOOLS  "Build the ncad-laser command-line tool"  ON)
option(NLASER_WITH_NOTOCAD "Build dxf2ljob, the bridge from a NotoCAD drawing (ncad_laser_ncad)" OFF)
set(NLASER_NOTOCAD_DIR "${PROJECT_SOURCE_DIR}/../NotoCAD" CACHE PATH "NotoCAD's source tree")

# NotoCAD's restrained dialect, enforced the same way: no RTTI, loud warnings.
add_library(nlaser_flags INTERFACE)
target_compile_options(nlaser_flags INTERFACE
  $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall;-Wextra;-Wpedantic;-Wshadow;-Wconversion;-fno-rtti>
  $<$<CXX_COMPILER_ID:MSVC>:/W4;/GR->
)

# ncad_laser_core: headless. Job, planner, encoder, decoder. Never includes a
# socket header -- that boundary is the reason it is its own target.
add_subdirectory(src/laser)
if(NLASER_WITH_NET)
  add_subdirectory(src/net)
endif()
if(NLASER_WITH_NOTOCAD AND TARGET ncad_core)
  # Consumed BY NotoCAD (its NCAD_WITH_LASER): the enclosing project already
  # defines ncad_core, and adding NotoCAD again from here would be a cycle --
  # NotoCAD adds us, we add NotoCAD, which adds us. Use what is there.
  message(STATUS "Nlaser: building inside NotoCAD, using its ncad_core")
  add_subdirectory(src/ncad)
elseif(NLASER_WITH_NOTOCAD)
  if(NOT EXISTS "${NLASER_NOTOCAD_DIR}/src/core/CMakeLists.txt")
    message(FATAL_ERROR "NLASER_WITH_NOTOCAD: no NotoCAD source tree at '${NLASER_NOTOCAD_DIR}'; "
                        "set NLASER_NOTOCAD_DIR")
  endif()
  # NotoCAD's own root, as a subproject, rather than an ExternalProject or an
  # imported prebuilt libncad_core.a. ncad_core is static, depends on a version
  # header NotoCAD generates at build time and on NotoCAD's flags (-fno-rtti
  # among them); its own CMakeLists already knows all of that, so building it
  # here means one compiler, one configure, and an edit to NotoCAD rebuilding
  # what depends on it. A prebuilt library can be stale or built with other
  # flags without anyone noticing, and an ExternalProject would have to restate
  # the include and generated-header paths by hand. EXCLUDE_FROM_ALL builds only
  # what ncad_laser_ncad pulls in (ncad_core and its version header); NotoCAD's
  # tests, GUI and DWG are switched off through its own options, which honour
  # these variables (CMP0077).
  #
  # SYSTEM, so NotoCAD's headers are judged by NotoCAD's flags and not ours:
  # Nlaser's own files stay at zero warnings without that depending on every
  # header NotoCAD ships (tables.hpp's visible_color() trips clang's
  # -Wimplicit-int-conversion-on-negation today). That keyword needs 3.25.
  if(CMAKE_VERSION VERSION_LESS 3.25)
    message(FATAL_ERROR "NLASER_WITH_NOTOCAD needs CMake 3.25 or later")
  endif()
  set(NCAD_BUILD_TESTS OFF)
  set(NCAD_BUILD_GUI OFF)
  set(NCAD_WITH_DWG OFF)
  # NotoCAD would otherwise add Nlaser back (its NCAD_WITH_LASER defaults ON
  # beside us) and the two would recurse. The other direction is the
  # `TARGET ncad_core` branch above.
  set(NCAD_WITH_LASER OFF)
  add_subdirectory("${NLASER_NOTOCAD_DIR}" notocad EXCLUDE_FROM_ALL SYSTEM)
  add_subdirectory(src/ncad)
endif()
if(NLASER_BUILD_TOOLS)
  add_subdirectory(src/tools)
endif()

if(NLASER_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

message(STATUS "Nlaser ${PROJECT_VERSION}  build=${CMAKE_BUILD_TYPE}  net=${NLASER_WITH_NET}  notocad=${NLASER_WITH_NOTOCAD}")
