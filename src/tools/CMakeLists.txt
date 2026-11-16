# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, Sadie Forbes

add_executable(ncad-laser ncad_laser.cpp)
target_link_libraries(ncad-laser PRIVATE nlaser_flags)
if(NLASER_WITH_NET)
  # ncad_laser_net brings ncad_laser_core with it. Linked here only when the
  # bridge, which links it itself, is absent: naming it twice makes the linker
  # warn about the duplicate.
  if(NOT NLASER_WITH_NOTOCAD)
    target_link_libraries(ncad-laser PRIVATE ncad_laser_net)
  endif()
  target_compile_definitions(ncad-laser PRIVATE NLASER_WITH_NET)
else()
  target_link_libraries(ncad-laser PRIVATE ncad_laser_core)
endif()
if(NLASER_WITH_NOTOCAD)
  target_link_libraries(ncad-laser PRIVATE ncad_laser_ncad)
  target_compile_definitions(ncad-laser PRIVATE NLASER_WITH_NOTOCAD)
endif()
