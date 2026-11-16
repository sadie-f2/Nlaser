// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

#include "ncad/laser/ruida_transport.hpp"

#include "ncad/laser/ruida_encode.hpp"
#include "ncad/laser/ruida_job.hpp"

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

bool prepare_send(const LaserJob& planned, const DeviceConfig& cfg, std::string& plain, std::string& error) {
    for (const LaserPath& p : planned.paths) {
        const LaserLayer* l = planned.layer(p.pen);
        if (l && l->mode != LayerMode::Skip && (l->max_power > cfg.max_power || l->min_power > cfg.max_power ||
                                                (l->has_power2 && l->max_power2 > cfg.max_power))) {
            char buf[160];
            std::snprintf(buf, sizeof buf, "layer %d asks for %.1f%% power; this device allows at most %.1f%%",
                          l->pen, l->max_power, cfg.max_power);
            error = buf;
            return false;
        }
    }
    RdJob rd;
    return job_to_rd(planned, rd, error) && encode_rd(rd, plain, error);
}

bool prepare_frame(const LaserJob& planned, std::string& plain, std::string& error) {
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
    rd.origin = {planned.origin.x, planned.origin.y};
    RdLayer l;
    l.speed_mm_s = 100.0;
    l.min_power = l.max_power = 0.0;
    l.air = false;
    rd.layers.push_back(l);
    RdPath box;
    // Round the box, then back to the origin: where the head was put is where
    // it should be left, ready for Send.
    box.pts = {{lo.x, lo.y}, {hi.x, lo.y}, {hi.x, hi.y}, {lo.x, hi.y}, {lo.x, lo.y},
               {planned.origin.x, planned.origin.y}};
    rd.paths.push_back(box);
    return encode_rd(rd, plain, error);
}

RuidaTransport::RuidaTransport(DatagramPort& port, const DeviceConfig& cfg)
    : cfg_(cfg), link_(port, cfg.magic, link_options(cfg)) {}

RuidaTransport::RuidaTransport(StreamPort& port, const DeviceConfig& cfg)
    : cfg_(cfg), link_(port, cfg.magic, link_options(cfg)) {}

bool RuidaTransport::open(std::string& error) { return link_.connect(error); }

bool RuidaTransport::send(const LaserJob& planned, const Progress& progress, std::string& error) {
    std::string plain;
    if (!prepare_send(planned, cfg_, plain, error)) return false;
    return link_.upload(plain, progress, error);
}

bool RuidaTransport::frame(const LaserJob& planned, std::string& error) {
    std::string plain;
    if (!prepare_frame(planned, plain, error)) return false;
    return link_.upload(plain, Progress{}, error);
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

bool RuidaTransport::status(MachineStatus& out, std::string& error) {
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
    if (link_.query(kMemBedX, r, ignore) && link_.query(kMemBedY, r2, ignore) && r.has_number && r2.has_number) {
        s.has_bed = true;
        s.bed_w_mm = reply_mm(r);
        s.bed_h_mm = reply_mm(r2);
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
