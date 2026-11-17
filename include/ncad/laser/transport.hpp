// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// The driver interface: what a front end asks of a laser controller.
//
// Nothing here includes a socket header. A transport moves bytes through a
// `DatagramPort`, which the network target implements over UDP and the tests
// implement in memory, so every protocol decision is tested with no network.
// A second controller (GRBL, say) implements LaserTransport
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

// A byte stream, for a link with no message boundaries (a serial port). It is
// a sibling of DatagramPort, not a disguise for it: a datagram port promises
// that one recv is one whole packet, which a tty cannot, and faking that in
// the OS layer would bury the reply framing where the tests cannot see it. So
// the port only moves bytes and the core (RuidaLink) decides where a reply
// ends.
class StreamPort {
public:
    virtual ~StreamPort() = default;

    // All of `bytes`, or false with `error`.
    virtual bool write(std::string_view bytes, std::string& error) = 0;

    // Appends whatever is available to `out` (at most `max` bytes). 1: at
    // least one byte was appended. 0: nothing within `timeout_ms` (0 polls).
    // -1: the port failed (`error` says how).
    virtual int read(std::string& out, std::size_t max, int timeout_ms, std::string& error) = 0;
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
    // The controller's keypad channel (UDP only), used by the panel-key pulse.
    // Ruida's own: 50207, answered to 40207 (udp_connection.py:25-26).
    int keypad_port{50207};
    int keypad_local_port{40207};  // 0 = any (tests)
    std::string serial;     // device path; when set, the link is USB serial, not UDP
    int baud{115200};       // serial only; see serial_port.hpp on why it hardly matters
    bool hw_flow{true};     // serial only: RTS/CTS, as MeerK40t opens it
    std::size_t max_packet{1000};
    double max_power{100.0};  // Send refuses a layer above this
    int ack_timeout_ms{2000};

    // From lasers.conf. The origin corner and flips map design to machine
    // coordinates; the bed size lets an absolute job (bed coordinates) be
    // mapped on a machine that counts from its right or top edge. When it is
    // not configured the transport asks the controller (memory 00 26 / 00 36).
    double bed_w_mm{0.0}, bed_h_mm{0.0};  // 0 = not configured
    std::string origin_corner;            // "bl" "br" "tl" "tr"; empty = not configured
    bool flip_x{false}, flip_y{false};
    // The red-dot pointer offset K in mm, in the machine/file frame as
    // LightBurn writes it (lasers.conf `pointer_offset = X Y`): the beam is K
    // from the dot. Added to the laser path of send and dryrun, never to the
    // pointer moves (frame, goto, jog); see ruida_encode.hpp.
    double pointer_offset_x{0.0}, pointer_offset_y{0.0};
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

// What a move's coordinates mean.
enum class MoveFrame : std::uint8_t {
    Bed,      // bed coordinates: (0,0) bottom-left, X right, Y up, mm
    Machine,  // raw machine coordinates, as the controller counts them
    Offset,   // a displacement from the head, in bed directions (X right, Y up)
};

struct MoveReport {
    double machine_x{0.0}, machine_y{0.0};  // where the head was sent
    bool has_bed_pos{false};
    double bed_x{0.0}, bed_y{0.0};          // the same point in bed coordinates
    std::string note;                       // a caveat for the user, or empty
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

    // Traces the whole planned path with the laser off: the job Send would
    // upload, every cut turned into a move and every power 0.
    virtual bool dryrun(const LaserJob& planned, std::string& error) = 0;

    // FIRES THE LASER: a one-shot micro-job, one cut at the head's current
    // position, at `power_pct` for about `ms` milliseconds. Refuses power
    // above the device's max_power and a duration outside 10..2000 ms.
    virtual bool pulse(double power_pct, int ms, std::string& error) = 0;

    // Moves the head (laser off) to a point, rapid. `frame` says what x and y
    // mean; `speed_mm_s` <= 0 leaves the controller's own rapid speed. Refuses
    // a target outside the bed. Default: not supported.
    virtual bool move_head(MoveFrame frame, double x, double y, double speed_mm_s, MoveReport& report,
                           std::string& error) {
        (void)frame; (void)x; (void)y; (void)speed_mm_s; (void)report;
        error = "this controller cannot move the head";
        return false;
    }

    virtual bool stop(std::string& error) = 0;
    virtual bool status(MachineStatus& out, std::string& error) = 0;

    // The cheap read for repeated polling: state, head position and file
    // progress only (no bed size). Fields the controller does not answer stay
    // unset; false only when even the first read gets no reply.
    virtual bool poll(MachineStatus& out, std::string& error) = 0;
    virtual bool counters(Counters& out, std::string& error) = 0;

    // A non-fatal remark about the last send/frame/dryrun (for instance that
    // the head or the bed could not be read, so the job was not checked
    // against the bed). Empty when there is nothing to say.
    virtual std::string note() const { return {}; }
};

}  // namespace ncad::laser
