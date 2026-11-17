// SPDX-License-Identifier: BSD-3-Clause AND MIT
// Copyright (c) 2026, Sadie Forbes
// Copyright (c) 2021 meerk40t
//
// The key codes and the keypad channel come from MeerK40t; see keypad.cpp and
// third_party/meerk40t/NOTICE.

// The controller's keypad channel (UDP only): the panel's Pulse key pressed
// and released over the network. Lives in the net target because it needs its
// own UDP socket, separate from the data link on 50200.
//
// THIS FIRES THE LASER at the power set on the controller's panel; nothing
// here chooses a power.
#pragma once

#include "ncad/laser/transport.hpp"

#include <string>

namespace ncad::laser {

// Sends key-down, waits `ms` (10..2000), sends key-up. UDP devices only. If
// the key-up cannot be sent the error says the laser may still be firing and
// to press Stop. `cfg.keypad_port` is the controller's, `keypad_local_port` ours.
// The key method fires at the panel's own power, which nothing here can read
// or cap, so it is allowed only when neither max_power nor power_limit is
// below 100. False with the reason in `error` otherwise.
bool key_pulse_allowed(const DeviceConfig& cfg, std::string& error);

bool pulse_key(const DeviceConfig& cfg, int ms, std::string& error);

}  // namespace ncad::laser
