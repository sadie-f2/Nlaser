# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, Sadie Forbes

# ncad_laser_ncad: the bridge from a NotoCAD drawing to a LaserJob. The only
# target that links NotoCAD, so ncad_laser_core stays buildable without it and
# the dependency is visible here rather than inferred.
add_library(ncad_laser_ncad STATIC
  laser_renderer.cpp
  laser_command.cpp
)
target_link_libraries(ncad_laser_ncad PUBLIC ncad_laser_core ncad_core PRIVATE nlaser_flags)
add_library(ncad::laser_ncad ALIAS ncad_laser_ncad)

# The LASER command opens devices through a LaserConnector. Wired to the real
# ports when the net target exists; otherwise the command still builds and its
# device operations say so.
if(TARGET ncad_laser_net)
  target_link_libraries(ncad_laser_ncad PUBLIC ncad_laser_net)
  target_compile_definitions(ncad_laser_ncad PUBLIC NLASER_HAVE_NET)
endif()
