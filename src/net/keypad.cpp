// SPDX-License-Identifier: BSD-3-Clause AND MIT
// Copyright (c) 2026, Sadie Forbes
// Copyright (c) 2021 meerk40t

#include "ncad/laser/keypad.hpp"

#include "ncad/laser/ruida_transport.hpp"
#include "ncad/laser/udp_port.hpp"

#include <chrono>
#include <cstdio>
#include <thread>

namespace ncad::laser {

namespace {

// meerk40t/ruida/rdjob.py:30-31: INTERFACE_PULSE_DOWN = A5 50 05, _UP = A5 51 05
// (the block at rdjob.py:12 is headed "Ports 50207 and 40207").
const std::string_view kPulseDown("\xA5\x50\x05", 3);
const std::string_view kPulseUp("\xA5\x51\x05", 3);

}  // namespace

bool key_pulse_allowed(const DeviceConfig& cfg, std::string& error) {
    if (cfg.max_power < 100.0 || cfg.power_limit < 100.0) {
        char buf[300];
        std::snprintf(buf, sizeof buf,
                      "the key method fires at the power set on the controller's panel, which cannot be held below "
                      "this device's limits (operator max_power %g%%, hard limit power_limit %g%%); it is refused "
                      "unless both are 100. Use the job method", cfg.max_power, cfg.power_limit);
        error = buf;
        return false;
    }
    return true;
}

bool pulse_key(const DeviceConfig& cfg, int ms, std::string& error) {
    if (!key_pulse_allowed(cfg, error)) return false;
    if (!cfg.serial.empty() || cfg.host.empty()) {
        error = "the key method needs a UDP device (--host): the keypad channel does not exist over USB serial";
        return false;
    }
    if (ms < ruida::kPulseMinMs || ms > ruida::kPulseMaxMs) {
        error = "pulse duration must be " + std::to_string(ruida::kPulseMinMs) + ".." +
                std::to_string(ruida::kPulseMaxMs) + " ms";
        return false;
    }
    // Ports: controller 50207, answered to 40207 (udp_connection.py:25-26,
    // udp_transport.py:22-23; control.py:42,66 bind the same pair).
    // Bytes: sent plain, neither swizzled nor checksummed. MeerK40t's emulator
    // takes this channel through realtime_write, which sets swizzle_mode False
    // and writes with unswizzle=False (emulator.py:132-141), while the 50200
    // channel goes through checksum_write (emulator.py:102-113). MeerK40t's own
    // sender never uses the keypad port (udp_transport.py:20-21 "not used"), so
    // that this matches a real controller is unverified.
    UdpPort port;
    if (!port.open(cfg.host, cfg.keypad_port, cfg.keypad_local_port, error)) return false;
    if (!port.send(kPulseDown, error)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    std::string up_error;
    if (!port.send(kPulseUp, up_error)) {
        error = "key-up could not be sent (" + up_error + "); the laser may still be firing: press Stop on the panel";
        return false;
    }
    return true;
}

}  // namespace ncad::laser
