// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// The driver interface: what a front end asks of a laser controller.
//
// Nothing here includes a socket header. A transport moves bytes through a
// `DatagramPort`, which the network target implements over UDP and the tests
// implement in memory, so every protocol decision is tested with no network.
// A second controller (GRBL, a USB-serial Ruida) implements LaserTransport
// without touching the front ends.
#pragma once

#include "ncad/laser/job.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <string_view>

namespace ncad::laser {

// One datagram out, one datagram in. Implementations deliver only datagrams
// from the device they were opened for.
class DatagramPort {
public:
    virtual ~DatagramPort() = default;

    virtual bool send(std::string_view bytes, std::string& error) = 0;

    // 1: a datagram arrived in `out`. 0: nothing within `timeout_ms`.
    // -1: the port failed (`error` says how).
    virtual int recv(std::string& out, int timeout_ms, std::string& error) = 0;
};

// Called from whatever thread runs the transfer: the terminal prints a
// percentage, the GUI posts to its event loop.
using Progress = std::function<void(std::size_t done, std::size_t total)>;

struct DeviceConfig {
    std::string name;
    std::string host;
    std::uint8_t magic{0x88};
    int port{50200};        // controller
    int local_port{40200};  // ours; 0 = any (tests)
    std::size_t max_packet{1000};
    double max_power{100.0};  // Send refuses a layer above this
    int ack_timeout_ms{2000};

    // From lasers.conf, kept for later. The machine transform (origin corner,
    // flips) is not applied anywhere yet: today a job is placed at the head's
    // current position and the bed is read back from the controller.
    double bed_w_mm{0.0}, bed_h_mm{0.0};  // 0 = not configured
    std::string origin_corner;            // "bl" "br" "tl" "tr"; empty = not configured
    bool flip_x{false}, flip_y{false};
};

// Every field is optional: a controller that does not answer a query leaves
// its field unset rather than failing the whole read.
struct MachineStatus {
    bool has_state{false};
    std::uint64_t state{0};  // raw; meaning not yet confirmed on hardware
    bool has_position{false};
    double x_mm{0.0}, y_mm{0.0};
    bool has_bed{false};
    double bed_w_mm{0.0}, bed_h_mm{0.0};
    bool has_progress{false};
    std::uint64_t file_total{0}, file_progress{0};
};

struct Counters {
    bool has_power_on{false};   std::uint64_t power_on{0};   // "Total Open Time (s)"
    bool has_work_time{false};  std::uint64_t work_time{0};  // "Total Work Time (s)"
    bool has_laser_on{false};   std::uint64_t laser_on{0};   // "Total Laser Work Time"
    bool has_jobs{false};       std::uint64_t jobs{0};       // "Total Work Number"
    bool has_travel_x{false};   std::uint64_t travel_x{0};   // "X Total Travel (m)"
    bool has_travel_y{false};   std::uint64_t travel_y{0};   // "Y Total Travel (m)"
    bool has_card_id{false};    std::uint64_t card_id{0};
    bool has_firmware{false};   std::string firmware;
};

class LaserTransport {
public:
    virtual ~LaserTransport() = default;

    virtual bool open(std::string& error) = 0;

    // Uploads a planned job, which the controller runs on arrival. The only
    // call that moves the head under power. Refuses an unplanned job and any
    // layer above the device's max power.
    virtual bool send(const LaserJob& planned, const Progress& progress, std::string& error) = 0;

    // Traces the job's box with the laser off.
    virtual bool frame(const LaserJob& planned, std::string& error) = 0;

    virtual bool stop(std::string& error) = 0;
    virtual bool status(MachineStatus& out, std::string& error) = 0;
    virtual bool counters(Counters& out, std::string& error) = 0;
};

}  // namespace ncad::laser
