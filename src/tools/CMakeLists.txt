# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, Sadie Forbes

add_executable(ncad-laser ncad_laser.cpp)
target_link_libraries(ncad-laser PRIVATE ncad_laser_core nlaser_flags)
