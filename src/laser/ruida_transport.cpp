// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

#include "ncad/laser/ruida_transport.hpp"

#include "ncad/laser/ruida_encode.hpp"
#include "ncad/laser/move.hpp"
#include "ncad/laser/ruida_codec.hpp"
#include "ncad/laser/ruida_job.hpp"

#include <algorithm>
#include <chrono>
#include <thread>
#include <cmath>
#include <cstdio>

namespace ncad::laser::ruida {

namespace {

LinkOptions link_options(const DeviceConfig& cfg) {
    LinkOptions o;
    o.ack_timeout_ms = cfg.ack_timeout_ms;
    o.max_packet = cfg.max_packet;
    return o;
}

}  // namespace

RdMachine machine_of(const DeviceConfig& cfg) {
    RdMachine m;
    if (!cfg.origin_corner.empty()) parse_machine_corner(cfg.origin_corner, m.origin);  // validated at parse
    m.flip_x = cfg.flip_x;
    m.flip_y = cfg.flip_y;
    m.bed_w_mm = cfg.bed_w_mm;
    m.bed_h_mm = cfg.bed_h_mm;
    return m;
}

// Which limit stopped the job, in words: the operator's or the hard one.
static std::string limit_words(const DeviceConfig& cfg) {
    char buf[200];
    if (cfg.max_power <= cfg.power_limit) {
        std::snprintf(buf, sizeof buf, "operator max_power %g%% allows no more", cfg.max_power);
    } else {
        std::snprintf(buf, sizeof buf, "hard limit %g%% (maintenance setting in lasers.conf) allows no more",
                      cfg.power_limit);
    }
    return buf;
}

bool placement_in_bed(const LaserJob& job, const DeviceConfig& cfg, const MachineStatus& head, PlacementReport& rep,
                      std::string& error, bool beam) {
    rep = PlacementReport{};
    // Absolute jobs carry their own place; the encoder checks them against
    // the bed. Only a job started from the head can leave the bed because of
    // where the head happens to be.
    if (job.start == StartMode::Absolute) return true;
    Pt lo, hi;
    if (!job_bounds(job, lo, hi)) return true;

    // POLICY: a check that cannot be made is a warning, not a refusal. Over
    // USB serial the controller need not answer reads at all, and a link that
    // cannot report its head position must still be able to run a job -- the
    // alternative makes serial unusable. When the position and the bed ARE
    // known, leaving the bed is refused outright.
    double bw = cfg.bed_w_mm, bh = cfg.bed_h_mm;
    if (!(bw > 0.0 && bh > 0.0) && head.has_bed) {
        bw = head.bed_w_mm;
        bh = head.bed_h_mm;
    }
    const bool have_bed = bw > 0.0 && bh > 0.0;
    if (!head.has_position || !have_bed) {
        rep.verified = false;
        rep.note = std::string("WARNING: cannot verify that the job stays on the bed (") +
                   (!head.has_position ? "the head position could not be read" : "the bed size is not known") +
                   "); sending anyway";
        return true;
    }

    // The head is at machine (hx,hy); the origin point lands there and every
    // other point is displaced from it, with the sign of each axis.
    const RdMachine m = machine_of(cfg);
    const double sx = m.sign_x(), sy = m.sign_y();
    // The head reported is the red dot. A laser path (`beam`) lands K from it
    // (send, dryrun); a pointer path (frame) lands on it.
    const double kx = beam ? cfg.pointer_offset_x : 0.0, ky = beam ? cfg.pointer_offset_y : 0.0;
    double x0 = head.x_mm + kx + sx * (lo.x - job.origin.x), x1 = head.x_mm + kx + sx * (hi.x - job.origin.x);
    double y0 = head.y_mm + ky + sy * (lo.y - job.origin.y), y1 = head.y_mm + ky + sy * (hi.y - job.origin.y);
    if (x0 > x1) std::swap(x0, x1);
    if (y0 > y1) std::swap(y0, y1);
    rep.x_min = x0;
    rep.x_max = x1;
    rep.y_min = y0;
    rep.y_max = y1;
    rep.has_box = true;
    constexpr double kEps = 0.0005;  // half a micrometre: the file's resolution
    char buf[400];
    if (x0 < -kEps || y0 < -kEps || x1 > bw + kEps || y1 > bh + kEps) {
        std::snprintf(buf, sizeof buf,
                      "the job would leave the bed: with the head at machine %.3f, %.3f mm it covers x %.3f..%.3f, "
                      "y %.3f..%.3f, and the bed is 0..%.3f by 0..%.3f. Move the head (ncad-laser goto/jog) or use "
                      "start=absolute",
                      head.x_mm, head.y_mm, x0, x1, y0, y1, bw, bh);
        error = buf;
        return false;
    }
    std::snprintf(buf, sizeof buf, "placement ok: head at machine %.3f, %.3f mm, job covers x %.3f..%.3f, y %.3f..%.3f "
                                   "of the %.3f x %.3f bed",
                  head.x_mm, head.y_mm, x0, x1, y0, y1, bw, bh);
    rep.note = buf;
    return true;
}

bool prepare_send(const LaserJob& planned, const DeviceConfig& cfg, std::string& plain, std::string& error) {
    const double cap = std::min(cfg.max_power, cfg.power_limit);
    for (const LaserPath& p : planned.paths) {
        const LaserLayer* l = planned.layer(p.pen);
        if (l && l->mode != LayerMode::Skip && (l->max_power > cap || l->min_power > cap ||
                                                (l->has_power2 && l->max_power2 > cap))) {
            char buf[320];
            std::snprintf(buf, sizeof buf, "pen %d asks for %.1f%% power; %s", l->pen, l->max_power,
                          limit_words(cfg).c_str());
            error = buf;
            return false;
        }
    }
    RdJob rd;
    if (!job_to_rd(planned, rd, error)) return false;
    rd.power_limit = cfg.power_limit;
    rd.machine = machine_of(cfg);
    rd.pointer_offset = {cfg.pointer_offset_x, cfg.pointer_offset_y};  // the beam, as LightBurn
    return encode_rd(rd, plain, error);
}

bool prepare_frame(const LaserJob& planned, const DeviceConfig& cfg, std::string& plain, std::string& error) {
    if (!planned.planned) {
        error = "the job is not planned; run 'ncad-laser plan' first";
        return false;
    }
    Pt lo, hi;
    if (!job_bounds(planned, lo, hi)) {
        error = "the job is empty";
        return false;
    }
    RdJob rd;
    rd.travel_only = true;
    rd.machine = machine_of(cfg);
    // No pointer offset: Frame traces the box with the red dot, so the dot
    // (not the beam) must land on it. Send and dryrun add K; see ruida_encode.hpp.
    // Framed by the rules that will place the job, including where: an
    // absolute job is traced at its absolute place, not at the head.
    rd.start = planned.start == StartMode::Absolute ? RdStart::Absolute
               : planned.start == StartMode::User   ? RdStart::User
                                                    : RdStart::Current;
    rd.origin = {planned.origin.x, planned.origin.y};
    RdLayer l;
    l.speed_mm_s = 100.0;
    l.min_power = l.max_power = 0.0;
    l.air = false;
    rd.layers.push_back(l);
    RdPath box;
    // Round the box, then back to the origin: where the head was put is where
    // it should be left, ready for Send. An absolute job has no origin to
    // return to (it is placed by bed coordinates), so it ends at the box's
    // starting corner.
    box.pts = {{lo.x, lo.y}, {hi.x, lo.y}, {hi.x, hi.y}, {lo.x, hi.y}, {lo.x, lo.y}};
    if (rd.start != RdStart::Absolute) box.pts.push_back({planned.origin.x, planned.origin.y});
    rd.paths.push_back(box);
    return encode_rd(rd, plain, error);
}

bool prepare_dryrun(const LaserJob& planned, const DeviceConfig& cfg, std::string& plain, std::string& error) {
    RdJob rd;
    if (!job_to_rd(planned, rd, error)) return false;
    rd.machine = machine_of(cfg);
    rd.power_limit = cfg.power_limit;
    rd.travel_only = true;  // every cut a move, every power 0
    rd.pointer_offset = {cfg.pointer_offset_x, cfg.pointer_offset_y};  // the beam's path, as send
    return encode_rd(rd, plain, error);
}

bool prepare_pulse(double power_pct, int ms, const DeviceConfig& cfg, std::string& plain, double& length_mm,
                   std::string& error) {
    if (!(power_pct > 0.0) || power_pct > 100.0) {
        error = "pulse power must be above 0 and at most 100%";
        return false;
    }
    if (power_pct > std::min(cfg.max_power, cfg.power_limit)) {
        char buf[320];
        std::snprintf(buf, sizeof buf, "a pulse at %.1f%% power is refused; %s", power_pct, limit_words(cfg).c_str());
        error = buf;
        return false;
    }
    if (ms < kPulseMinMs || ms > kPulseMaxMs) {
        error = "pulse duration must be " + std::to_string(kPulseMinMs) + ".." + std::to_string(kPulseMaxMs) + " ms";
        return false;
    }
    // length / speed = ms; at 1 mm/s that is ms micrometres, exactly the
    // encoder's resolution, so the time is not distorted by rounding.
    length_mm = kPulseSpeedMmS * ms / 1000.0;
    RdJob rd;
    rd.machine = machine_of(cfg);
    rd.power_limit = cfg.power_limit;
    rd.origin = {0.0, 0.0};  // the job starts at the head
    RdLayer l;
    l.speed_mm_s = kPulseSpeedMmS;
    l.min_power = l.max_power = power_pct;
    l.air = false;
    rd.layers.push_back(l);
    RdPath cut;
    cut.pts = {{0.0, 0.0}, {length_mm, 0.0}};
    rd.paths.push_back(cut);
    return encode_rd(rd, plain, error);
}

RuidaTransport::RuidaTransport(DatagramPort& port, const DeviceConfig& cfg)
    : cfg_(cfg), link_(port, cfg.magic, link_options(cfg)) {}

RuidaTransport::RuidaTransport(StreamPort& port, const DeviceConfig& cfg)
    : cfg_(cfg), link_(port, cfg.magic, link_options(cfg)) {}

bool RuidaTransport::open(std::string& error) { return link_.connect(error); }

bool RuidaTransport::check_placement(const LaserJob& job, DeviceConfig& effective, PlacementReport& rep,
                                     std::string& error, bool beam) {
    effective = cfg_;
    rep = PlacementReport{};
    // The head and the bed are read from the controller; either may be missing
    // (policy in placement_in_bed). A configured bed wins over a reported one.
    MachineStatus st;
    std::string ignore;
    if (!status(st, ignore)) st = MachineStatus{};
    if (!(effective.bed_w_mm > 0.0 && effective.bed_h_mm > 0.0) && st.has_bed) {
        effective.bed_w_mm = st.bed_w_mm;
        effective.bed_h_mm = st.bed_h_mm;
    }
    if (job.start == StartMode::Absolute) {
        if (!(effective.bed_w_mm > 0.0)) {
            rep.verified = false;
            rep.note = "WARNING: the bed size is not known, so the job's far edge was not checked against it";
        }
        return true;  // the encoder refuses what it can; see prepare_send
    }
    return placement_in_bed(job, effective, st, rep, error, beam);
}

bool RuidaTransport::prepare_checked(Kind kind, const LaserJob& job, std::string& plain, std::string& error) {
    auto prep = [&](const DeviceConfig& c) {
        return kind == Kind::Send    ? prepare_send(job, c, plain, error)
               : kind == Kind::Frame ? prepare_frame(job, c, plain, error)
                                     : prepare_dryrun(job, c, plain, error);
    };
    last_note_.clear();
    // What can be refused without asking the controller anything is refused
    // first, so a bad job never costs a byte on the wire. Only an absolute job
    // on a machine that counts from its far edge needs the bed size first.
    const bool needs_bed = job.start == StartMode::Absolute && machine_of(cfg_).mirrored() &&
                           !(cfg_.bed_w_mm > 0.0 && cfg_.bed_h_mm > 0.0);
    if (!needs_bed && !prep(cfg_)) return false;
    DeviceConfig eff;
    PlacementReport rep;
    if (!check_placement(job, eff, rep, error, kind != Kind::Frame)) return false;
    last_note_ = rep.note;
    return prep(eff);
}

bool RuidaTransport::send(const LaserJob& planned, const Progress& progress, std::string& error) {
    std::string plain;
    if (!prepare_checked(Kind::Send, planned, plain, error)) return false;
    return link_.upload(plain, progress, error);
}

bool RuidaTransport::frame(const LaserJob& planned, std::string& error) {
    std::string plain;
    if (!prepare_checked(Kind::Frame, planned, plain, error)) return false;
    return link_.upload(plain, Progress{}, error);
}

bool RuidaTransport::dryrun(const LaserJob& planned, std::string& error) {
    std::string plain;
    if (!prepare_checked(Kind::Dryrun, planned, plain, error)) return false;
    return link_.upload(plain, Progress{}, error);
}

bool corrupt_file_sum(std::string& plain, std::string& error) {
    // Split into commands rather than search for the bytes, so only a real E5 05 is touched.
    for (const std::string_view c : split_commands(plain)) {
        if (c.size() == 7 && static_cast<std::uint8_t>(c[0]) == 0xE5 && c[1] == 0x05) {
            // The last argument byte holds the sum's low 7 bits; flip the lowest.
            plain[static_cast<std::size_t>(c.data() - plain.data()) + 6] ^= 0x01;
            return true;
        }
    }
    error = "no Set File Sum (E5 05) in the job";
    return false;
}

bool parse_readonly_raw(std::string_view hex, std::string& plain, std::string& error) {
    plain.clear();
    int hi = -1;
    for (const char ch : hex) {
        if (ch == ' ' || ch == ':' || ch == ',' || ch == '\t') continue;
        int v = -1;
        if (ch >= '0' && ch <= '9') v = ch - '0';
        else if (ch >= 'a' && ch <= 'f') v = ch - 'a' + 10;
        else if (ch >= 'A' && ch <= 'F') v = ch - 'A' + 10;
        if (v < 0) {
            error = "raw: not hexadecimal";
            return false;
        }
        if (hi < 0) {
            hi = v;
        } else {
            plain.push_back(static_cast<char>((hi << 4) | v));
            hi = -1;
        }
    }
    if (hi >= 0) {
        error = "raw: odd number of hex digits";
        return false;
    }
    const auto b = [&](std::size_t i) { return static_cast<std::uint8_t>(plain[i]); };
    const bool enq = plain.size() == 1 && b(0) == 0xCE;
    const bool read = plain.size() == 4 && b(0) == 0xDA && b(1) == 0x00 && b(2) < 0x80 && b(3) < 0x80;
    if (!enq && !read) {
        plain.clear();
        error = "raw: only a memory read (DA 00 hh ll, address bytes below 80) or the ENQ (CE) may be sent";
        return false;
    }
    return true;
}

bool RuidaTransport::dryrun_bad_sum(const LaserJob& planned, std::string& error) {
    std::string plain;
    if (!prepare_checked(Kind::Dryrun, planned, plain, error)) return false;
    if (!corrupt_file_sum(plain, error)) return false;
    return link_.upload(plain, Progress{}, error);
}

bool RuidaTransport::pulse(double power_pct, int ms, std::string& error) {
    std::string plain;
    double length = 0.0;
    if (!prepare_pulse(power_pct, ms, cfg_, plain, length, error)) return false;
    return link_.upload(plain, Progress{}, error);
}

bool prepare_goto(double machine_x_mm, double machine_y_mm, double speed_mm_s, std::string& plain,
                  std::string& error) {
    // Rapid Move XY, D9 10: an options byte and two absolute coordinates
    // (rdjob.py RAPID_MOVE_XY; MeerK40t's driver.move_abs sends it with
    // origin=True, light=False for a move to an absolute position).
    //
    // The options byte (rdjob.py _rapid_options): bit 0 is "light" (the laser
    // is switched on lightly during the move -- the red-dot/low-power guide),
    // and bit 1 clear means "origin": the coordinates are measured from the
    // origin, not from the head (0x02, option NONE, is what MeerK40t uses for
    // a RELATIVE one-axis move). So 0x00 = origin, no light: nothing fires and
    // nothing lights, and the move is absolute. That is MeerK40t's choice for
    // an absolute move on an RDC6442S; whether the controller measures "origin"
    // from machine zero or from a user origin set on the panel is UNVERIFIED
    // here, as is the whole reading of the bits (MeerK40t's emulator only
    // labels them).
    //
    // Speed: MeerK40t precedes each such move with C9 02 (Speed Laser 1) at
    // its jog speed, 600 mm/s by default (driver.py move_abs). C9 03 (Speed
    // Axis; LightBurn writes 15 mm/s in every job header) and C9 06 (Axis Move
    // Speed) exist but MeerK40t never sends them for a jog. C9 02 is the
    // only command with a track record for rapid speed, and is what --speed
    // uses; it is UNVERIFIED on this controller whether it changes the rapid
    // speed, or the speed of the next job (LightBurn sets that itself).
    std::string out;
    std::int64_t x = 0, y = 0;
    const double fx = std::round(machine_x_mm * 1000.0), fy = std::round(machine_y_mm * 1000.0);
    constexpr double kLim = 2147483647.0;
    if (!std::isfinite(fx) || !std::isfinite(fy) || std::fabs(fx) > kLim || std::fabs(fy) > kLim) {
        error = "a coordinate is out of range";
        return false;
    }
    x = static_cast<std::int64_t>(fx);
    y = static_cast<std::int64_t>(fy);
    if (speed_mm_s > 0.0) {
        out += std::string("\xC9\x02", 2);
        if (!put_speed(out, speed_mm_s)) {
            error = "the speed is out of range";
            return false;
        }
    }
    out += std::string("\xD9\x10\x00", 3);
    if (!put_abs_coord(out, x) || !put_abs_coord(out, y)) {
        error = "a coordinate could not be encoded";
        return false;
    }
    plain = std::move(out);
    return true;
}

namespace {
constexpr int kSettleMs = 150;    // jog: gap between the two position reads
constexpr int kSettleMaxS = 30;   // jog: give up waiting for the head after this
}  // namespace

bool RuidaTransport::move_head(MoveFrame frame, double x, double y, double speed_mm_s, MoveReport& report,
                               std::string& error) {
    // Head and bed, as far as the controller tells them; plan_move says what
    // is missing and whether that matters for this move.
    MachineStatus st;
    std::string ignore;
    if (!status(st, ignore)) st = MachineStatus{};
    // A jog is a step from where the head IS. If an earlier move is still
    // running, the position read is a point on the way, and the step would be
    // taken from there (seen on the Black Cat: two 100 mm jogs 3 s apart).
    // So wait until two reads agree before taking the step.
    if (frame == MoveFrame::Offset && st.has_position) {
        const auto start = std::chrono::steady_clock::now();
        for (;;) {
            std::this_thread::sleep_for(std::chrono::milliseconds(kSettleMs));
            MachineStatus again;
            if (!status(again, ignore) || !again.has_position) break;  // keep the read we have
            const bool still = std::fabs(again.x_mm - st.x_mm) < 0.005 && std::fabs(again.y_mm - st.y_mm) < 0.005;
            st = again;
            if (still) break;
            if (std::chrono::steady_clock::now() - start > std::chrono::seconds(kSettleMaxS)) {
                error = "the head is still moving after " + std::to_string(kSettleMaxS) +
                        " s; jog again when it has stopped";
                return false;
            }
        }
    }
    if (!plan_move(cfg_, st, frame, x, y, report, error)) return false;
    std::string plain;
    if (!prepare_goto(report.machine_x, report.machine_y, speed_mm_s, plain, error)) return false;
    return link_.command(plain, error);
}

bool RuidaTransport::listen_begin(std::string& error) {
    if (link_.serial_framing()) return true;
    return link_.connect(error);
}

int RuidaTransport::listen_next(std::string& raw, std::string& plain, int timeout_ms, std::string& error) {
    return link_.hear(raw, plain, timeout_ms, error);
}

bool RuidaTransport::stop(std::string& error) {
    const char cmd[2] = {static_cast<char>(0xD8), 0x01};
    return link_.command(std::string_view(cmd, 2), error);
}

namespace {

// A coordinate reply: signed 32-bit um in the 35-bit field.
double reply_mm(const MemReply& r) {
    return static_cast<double>(static_cast<std::int32_t>(static_cast<std::uint32_t>(r.number & 0xFFFFFFFFu))) /
           1000.0;
}

}  // namespace

bool RuidaTransport::poll(MachineStatus& out, std::string& error) {
    MachineStatus s;
    MemReply r, r2;
    // The first read must succeed: it is the test that the link is alive.
    if (!link_.query(kMemMachineStatus, r, error)) return false;
    if (r.has_number) {
        s.has_state = true;
        s.state = r.number;
    }
    std::string ignore;
    if (link_.query(kMemPosX, r, ignore) && link_.query(kMemPosY, r2, ignore) && r.has_number && r2.has_number) {
        s.has_position = true;
        s.x_mm = reply_mm(r);
        s.y_mm = reply_mm(r2);
    }
    if (link_.query(kMemFileTotal, r, ignore) && link_.query(kMemFileProgress, r2, ignore) && r.has_number &&
        r2.has_number) {
        s.has_progress = true;
        s.file_total = r.number;
        s.file_progress = r2.number;
    }
    out = s;
    return true;
}

bool RuidaTransport::status(MachineStatus& out, std::string& error) {
    MachineStatus s;
    if (!poll(s, error)) return false;
    MemReply r, r2;
    std::string ignore;
    if (link_.query(kMemBedX, r, ignore) && link_.query(kMemBedY, r2, ignore) && r.has_number && r2.has_number) {
        s.has_bed = true;
        s.bed_w_mm = reply_mm(r);
        s.bed_h_mm = reply_mm(r2);
    }
    out = s;
    return true;
}

bool RuidaTransport::counters(Counters& out, std::string& error) {
    Counters c;
    MemReply r;
    if (!link_.query(kMemCardId, r, error)) return false;
    if (r.has_number) {
        c.has_card_id = true;
        c.card_id = r.number;
    }
    std::string ignore;
    auto read = [&](std::uint16_t mem, bool& has, std::uint64_t& v) {
        if (link_.query(mem, r, ignore) && r.has_number) {
            has = true;
            v = r.number;
        }
    };
    read(kMemPowerOnTime, c.has_power_on, c.power_on);
    read(kMemWorkTime, c.has_work_time, c.work_time);
    read(kMemLaserOnTime, c.has_laser_on, c.laser_on);
    read(kMemJobCount, c.has_jobs, c.jobs);
    read(kMemTravelX, c.has_travel_x, c.travel_x);
    read(kMemTravelY, c.has_travel_y, c.travel_y);
    if (link_.query(kMemFirmware, r, ignore)) {
        // A string, NUL-terminated, in MeerK40t's emulator; keep the printable part.
        c.has_firmware = true;
        for (char ch : r.raw) {
            if (ch == '\0') break;
            if (ch >= 0x20 && ch < 0x7F) c.firmware.push_back(ch);
        }
    }
    out = c;
    return true;
}

}  // namespace ncad::laser::ruida
