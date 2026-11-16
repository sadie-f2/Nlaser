# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, Sadie Forbes

add_executable(ncad-laser ncad_laser.cpp)
target_link_libraries(ncad-laser PRIVATE nlaser_flags)
# Each target is named once: the bridge links the core (and net, when built),
# and the linker warns about a library given twice.
if(NLASER_WITH_NET)
  target_compile_definitions(ncad-laser PRIVATE NLASER_WITH_NET)
endif()
if(NLASER_WITH_NOTOCAD)
  target_link_libraries(ncad-laser PRIVATE ncad_laser_ncad)
  target_compile_definitions(ncad-laser PRIVATE NLASER_WITH_NOTOCAD)
elseif(NLASER_WITH_NET)
  # ncad_laser_net brings ncad_laser_core with it.
  target_link_libraries(ncad-laser PRIVATE ncad_laser_net)
else()
  target_link_libraries(ncad-laser PRIVATE ncad_laser_core)
endif()
