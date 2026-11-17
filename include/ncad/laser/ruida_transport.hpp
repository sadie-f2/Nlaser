// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// LaserTransport for a Ruida controller, over a DatagramPort (UDP) or a
// StreamPort (USB serial; ruida_link.hpp says how the framing differs).
//
// Send uploads the encoded job; the controller runs it once its end-of-file
// arrives. Frame uploads a travel-only job -- the job's box as laser-off
// moves, with every power at 0 and no cut command in the file -- so the frame
// is placed by exactly the rules that will place the job. Dryrun is the same
// idea for the whole path. Pulse is a one-cut job (the only other call that
// fires the laser). Stop is D8 01.
#pragma once

#include "ncad/laser/ruida_link.hpp"
#include "ncad/laser/ruida_machine.hpp"
#include "ncad/laser/transport.hpp"

namespace ncad::laser::ruida {

class RuidaTransport final : public LaserTransport {
public:
    RuidaTransport(DatagramPort& port, const DeviceConfig& cfg);  // UDP framing
    RuidaTransport(StreamPort& port, const DeviceConfig& cfg);    // serial framing

    bool open(std::string& error) override;
    bool send(const LaserJob& planned, const Progress& progress, std::string& error) override;
    bool frame(const LaserJob& planned, std::string& error) override;
    bool dryrun(const LaserJob& planned, std::string& error) override;
    bool pulse(double power_pct, int ms, std::string& error) override;
    bool stop(std::string& error) override;
    bool status(MachineStatus& out, std::string& error) override;
    bool poll(MachineStatus& out, std::string& error) override;
    bool counters(Counters& out, std::string& error) override;

    const LinkStats& stats() const { return link_.stats(); }

private:
    DeviceConfig cfg_;
    RuidaLink link_;
};

// The bytes Send would upload, and the checks it makes first. Public so
// `ncad-laser send` can show what it is about to do, and so tests can see
// exactly what went on the wire.
bool prepare_send(const LaserJob& planned, const DeviceConfig& cfg, std::string& plain, std::string& error);
bool prepare_frame(const LaserJob& planned, const DeviceConfig& cfg, std::string& plain, std::string& error);

// The whole planned path as a laser-off job: what prepare_send encodes, but
// with travel_only set, so every cut is a move and no power is above 0. The
// device's max_power is not checked, because no power is asked for.
bool prepare_dryrun(const LaserJob& planned, const DeviceConfig& cfg, std::string& plain, std::string& error);

// A test fire as a job: Move to the head, then one Cut of `length_mm` at
// `speed_mm_s` so that length/speed is `ms`. The speed is fixed at 1 mm/s
// (slow enough that the head barely moves), so the length in mm is ms/1000.
// Refuses power <= 0 or above cfg.max_power, and ms outside 10..2000.
// The cut runs along design +X from the head; origin_corner/flip_x may mirror
// that on the machine.
inline constexpr int kPulseMinMs = 10;
inline constexpr int kPulseMaxMs = 2000;
inline constexpr double kPulseSpeedMmS = 1.0;
bool prepare_pulse(double power_pct, int ms, const DeviceConfig& cfg, std::string& plain, double& length_mm,
                   std::string& error);

// The machine mapping a device's configuration asks for; an unset
// origin_corner means bottom-left (the identity).
RdMachine machine_of(const DeviceConfig& cfg);

}  // namespace ncad::laser::ruida
