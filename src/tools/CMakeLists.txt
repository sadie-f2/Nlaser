# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, Sadie Forbes

add_executable(ncad-laser ncad_laser.cpp)
target_link_libraries(ncad-laser PRIVATE nlaser_flags)
if(NLASER_WITH_NET)
  # ncad_laser_net brings ncad_laser_core with it.
  target_link_libraries(ncad-laser PRIVATE ncad_laser_net)
  target_compile_definitions(ncad-laser PRIVATE NLASER_WITH_NET)
else()
  target_link_libraries(ncad-laser PRIVATE ncad_laser_core)
endif()
