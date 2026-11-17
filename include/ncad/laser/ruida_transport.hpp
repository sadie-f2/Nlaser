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
//
// The red-dot pointer offset (DeviceConfig::pointer_offset_*) is applied by
// send and dryrun (the beam's path, as LightBurn) and not by frame, goto or
// jog (pointer moves); the pulse is a head-travel diagnostic and is not offset
// either. ruida_encode.hpp has the table and the evidence.
#pragma once

#include "ncad/laser/ruida_link.hpp"
#include "ncad/laser/ruida_machine.hpp"
#include "ncad/laser/transport.hpp"

namespace ncad::laser::ruida {

// A rapid move of the head to absolute MACHINE coordinates, laser off, as the
// bytes the controller is sent. `speed_mm_s` > 0 prefixes C9 02 (see the .cpp:
// UNVERIFIED on our controller).
bool prepare_goto(double machine_x_mm, double machine_y_mm, double speed_mm_s, std::string& plain,
                  std::string& error);

// What placement_in_bed concluded.
struct PlacementReport {
    bool verified{true};   // false: the check could not be made (the note says why)
    bool has_box{false};
    double x_min{0.0}, x_max{0.0}, y_min{0.0}, y_max{0.0};  // the job's box in machine coordinates
    std::string note;      // for the user, on success: "placement ok ..." or "WARNING: ..."
};

class RuidaTransport final : public LaserTransport {
public:
    RuidaTransport(DatagramPort& port, const DeviceConfig& cfg);  // UDP framing
    RuidaTransport(StreamPort& port, const DeviceConfig& cfg);    // serial framing

    bool open(std::string& error) override;
    bool send(const LaserJob& planned, const Progress& progress, std::string& error) override;
    bool frame(const LaserJob& planned, std::string& error) override;
    bool dryrun(const LaserJob& planned, std::string& error) override;
    // TEST ONLY, for hardware probing: a dry run whose Set File Sum is wrong,
    // to learn whether the controller rejects it. Safe because a dry run holds
    // no cut command and no power; there is deliberately no such call for send.
    bool dryrun_bad_sum(const LaserJob& planned, std::string& error);
    bool pulse(double power_pct, int ms, std::string& error) override;
    bool stop(std::string& error) override;
    bool move_head(MoveFrame frame, double x, double y, double speed_mm_s, MoveReport& report,
                   std::string& error) override;
    bool status(MachineStatus& out, std::string& error) override;
    bool poll(MachineStatus& out, std::string& error) override;
    bool counters(Counters& out, std::string& error) override;

    // Reads the head and the bed and judges where `job` would land; what
    // send/frame/dryrun do first, public so a front end can show the result
    // before it asks for confirmation. `effective` is the configuration with
    // the controller's bed filled in when the file had none.
    bool check_placement(const LaserJob& job, DeviceConfig& effective, PlacementReport& report, std::string& error,
                         bool beam = true);

    // Listening: send nothing and see what the controller says by itself.
    // listen_begin makes the one initial contact the protocol needs -- over UDP
    // the ENQ handshake (the controller will not talk to a port it has not
    // heard from), over serial nothing at all. listen_next is RuidaLink::hear.
    bool listen_begin(std::string& error);
    int listen_next(std::string& raw, std::string& plain, int timeout_ms, std::string& error);

    // One memory read (DA 00), for the settings dump. Read-only.
    bool read_memory(std::uint16_t mem, MemReply& out, std::string& error) { return link_.query(mem, out, error); }

    std::string note() const override { return last_note_; }

    const LinkStats& stats() const { return link_.stats(); }

private:
    enum class Kind : std::uint8_t { Send, Frame, Dryrun };
    bool prepare_checked(Kind kind, const LaserJob& job, std::string& plain, std::string& error);

    DeviceConfig cfg_;
    RuidaLink link_;
    std::string last_note_;
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

// Flips one bit in the Set File Sum (E5 05) of an encoded job so the sum no
// longer matches. False when there is no such command. Used only by
// RuidaTransport::dryrun_bad_sum.
bool corrupt_file_sum(std::string& plain, std::string& error);

// `ncad-laser raw`: hex text ("DA 00 04 21", "da000421", "CE") to the one
// kind of command it may send. Only a memory read (DA 00 + two address bytes
// below 0x80) and the ENQ (CE) are accepted; everything else is refused, so
// the tool cannot be used to move, cut or change a setting.
bool parse_readonly_raw(std::string_view hex, std::string& plain, std::string& error);

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

// The check that Send, Frame and Dryrun make before uploading a job that
// starts from the head's current position: where would its box land, with the
// head where `head` says it is, and does that leave the bed? Absolute jobs
// pass untouched (the encoder checks those). When the head position or the bed
// size is missing the job is ALLOWED, with a WARNING in `report.note`: over
// USB serial a controller need not answer reads, and that must not make the
// link unusable. When both are known and the box leaves the bed it is refused,
// with the numbers in `error`.
// `beam` true (send, dryrun): the path lands cfg.pointer_offset from the head,
// which is the red dot. False (frame): it lands on the head.
bool placement_in_bed(const LaserJob& job, const DeviceConfig& cfg, const MachineStatus& head, PlacementReport& report,
                      std::string& error, bool beam = true);

// The machine mapping a device's configuration asks for; an unset
// origin_corner means bottom-left (the identity).
RdMachine machine_of(const DeviceConfig& cfg);

}  // namespace ncad::laser::ruida
