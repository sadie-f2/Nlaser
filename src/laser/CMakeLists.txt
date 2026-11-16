# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, Sadie Forbes

add_library(ncad_laser_core STATIC
  ruida_codec.cpp
  ruida_decode.cpp
  ruida_encode.cpp
  ruida_job.cpp
  job.cpp
  plan.cpp
)
target_include_directories(ncad_laser_core PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(ncad_laser_core PRIVATE nlaser_flags)
add_library(ncad::laser_core ALIAS ncad_laser_core)
