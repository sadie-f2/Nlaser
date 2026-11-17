# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, Sadie Forbes

add_library(ncad_laser_core STATIC
  ruida_codec.cpp
  ruida_decode.cpp
  ruida_encode.cpp
  ruida_job.cpp
  move.cpp
  ruida_memory.cpp
  job.cpp
  xml.cpp
  lbrn2.cpp
  plan.cpp
  raster.cpp
  raster_image.cpp
  device_config.cpp
  ruida_link.cpp
  ruida_transport.cpp
  svg_import.cpp
  svg_nanosvg.cpp
)
# nanosvg is vendored and not ours to make warning-clean (it is C-style and
# float-heavy). Only the one file that includes it loses its warnings; it
# holds nothing but the copy out of nanosvg's structures.
set_source_files_properties(svg_nanosvg.cpp PROPERTIES COMPILE_OPTIONS
  "$<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-w>;$<$<CXX_COMPILER_ID:MSVC>:/W0>")
target_include_directories(ncad_laser_core PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(ncad_laser_core PRIVATE nlaser_flags)
add_library(ncad::laser_core ALIAS ncad_laser_core)

# Vendored stb (third_party/stb): its warnings are not ours to fix, so they are
# silenced for the one file that includes it, and the headers are SYSTEM.
target_include_directories(ncad_laser_core SYSTEM PRIVATE ${PROJECT_SOURCE_DIR}/third_party)
set_source_files_properties(raster_image.cpp PROPERTIES COMPILE_OPTIONS
  "$<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-w>;$<$<CXX_COMPILER_ID:MSVC>:/w>")
