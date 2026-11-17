# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, Sadie Forbes

# The only target that includes a socket or termios header.
add_library(ncad_laser_net STATIC
  udp_port.cpp
  serial_port.cpp
  connect.cpp
  keypad.cpp
)
target_link_libraries(ncad_laser_net PUBLIC ncad_laser_core PRIVATE nlaser_flags)
add_library(ncad::laser_net ALIAS ncad_laser_net)
