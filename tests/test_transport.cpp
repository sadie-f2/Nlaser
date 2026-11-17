// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// The Ruida link and transport against an in-memory controller.

#include "test.hpp"

#include "fake_controller.hpp"

#include "ncad/laser/job.hpp"
#include "ncad/laser/move.hpp"
#include "ncad/laser/plan.hpp"
#include "ncad/laser/ruida_decode.hpp"
#include "ncad/laser/ruida_link.hpp"
#include "ncad/laser/ruida_memory.hpp"
#include "ncad/laser/ruida_transport.hpp"

#include <cmath>
#include <cstring>
#include <utility>
#include <vector>
#include <string>

using namespace ncad::laser;
using namespace ncad::laser::ruida;
using nlaser_test::FakeController;

namespace {

LaserJob planned_square(double power = 10.0) {
    LaserJob raw;
    LaserLayer l;
    l.pen = 5;
    l.color = 5;
    l.speed = 100;
    l.min_power = l.max_power = power;
    raw.layers.push_back(l);
    LaserPath p;
    p.pen = 5;
    p.closed = true;
    p.pts = {{0, 0}, {30, 0}, {30, 30}, {0, 30}};
    raw.paths.push_back(p);
    LaserJob planned;
    PlanReport r;
    std::string error;
    plan_job(raw, planned, PlanOptions{}, r, error);
    return planned;
}

LinkOptions small_packets(std::size_t n) {
    LinkOptions o;
    o.max_packet = n;
    return o;
}

}  // namespace

// --- link --------------------------------------------------------------------

TEST_CASE("link: connect sends ENQ and accepts the ACK") {
    FakeController c;
    RuidaLink link(c, 0x88, LinkOptions{});
    std::string error;
    CHECK(link.connect(error));
    CHECK(c.enqs == 1);
}

TEST_CASE("link: connect gives up on a silent controller, with words") {
    FakeController c;
    c.silent = true;
    RuidaLink link(c, 0x88, LinkOptions{});
    std::string error;
    CHECK(!link.connect(error));
    CHECK(error.find("no answer") != std::string::npos);
}

TEST_CASE("link: a NAK resends the same packet") {
    FakeController c;
    c.nak_next = 2;
    RuidaLink link(c, 0x88, LinkOptions{});
    std::string error;
    CHECK(link.command(std::string("\xD8\x01", 2), error));
    CHECK(c.naks_sent == 2);
    CHECK(c.stops == 1);  // accepted once, not three times
    CHECK(link.stats().packets == 3);
}

TEST_CASE("link: a packet damaged in flight is NAKed and resent intact") {
    FakeController c;
    c.corrupt_next = true;
    RuidaLink link(c, 0x88, LinkOptions{});
    std::string error;
    CHECK(link.command(std::string("\xD8\x01", 2), error));
    CHECK(c.bad_checksums == 1);
    CHECK(c.stops == 1);
}

TEST_CASE("link: too many NAKs fail the command") {
    FakeController c;
    c.nak_next = 100;
    LinkOptions o;
    o.max_naks = 3;
    RuidaLink link(c, 0x88, o);
    std::string error;
    CHECK(!link.command(std::string("\xD8\x01", 2), error));
    CHECK(error.find("refused") != std::string::npos);
}

TEST_CASE("link: a missing ACK is a failure, never a resend") {
    FakeController c;
    c.drop_acks_after = 0;
    RuidaLink link(c, 0x88, LinkOptions{});
    std::string error;
    CHECK(!link.command(std::string("\xD8\x01", 2), error));
    CHECK(error.find("not resent") != std::string::npos);
    CHECK(link.stats().packets == 1);
}

TEST_CASE("link: ERR fails the command") {
    FakeController c;
    c.err_next = true;
    RuidaLink link(c, 0x88, LinkOptions{});
    std::string error;
    CHECK(!link.command(std::string("\xD8\x01", 2), error));
    CHECK(error.find("ERR") != std::string::npos);
}

TEST_CASE("link: keep-alives are ignored while waiting") {
    FakeController c;
    c.say(0xCE);
    RuidaLink link(c, 0x88, LinkOptions{});
    std::string error;
    // The queued keep-alive arrives first; the ACK after it completes.
    CHECK(link.command(std::string("\xD8\x01", 2), error));
    CHECK(link.stats().keepalives == 1);
}

TEST_CASE("link: memory read, with the reply after or before its ACK") {
    for (bool early : {false, true}) {
        FakeController c;
        c.reply_before_ack = early;
        c.set_number(kMemCardId, 0x65106510);
        RuidaLink link(c, 0x88, LinkOptions{});
        MemReply r;
        std::string error;
        REQUIRE(link.query(kMemCardId, r, error));
        CHECK(r.mem == kMemCardId);
        CHECK(r.has_number);
        CHECK(r.number == 0x65106510u);
        // The query went out as DA 00 05 7E.
        CHECK(c.packets.back() == std::string("\xDA\x00\x05\x7E", 4));
    }
}

TEST_CASE("link: upload splits only at command boundaries, under max_packet") {
    const LaserJob job = planned_square();
    std::string plain, error;
    REQUIRE(prepare_send(job, DeviceConfig{}, plain, error));

    for (std::size_t limit : {std::size_t{16}, std::size_t{100}, std::size_t{1000}}) {
        FakeController c;
        RuidaLink link(c, 0x88, small_packets(limit));
        std::size_t last = 0, calls = 0;
        REQUIRE(link.upload(plain, [&](std::size_t done, std::size_t total) {
            CHECK(done > last);
            CHECK(total == plain.size());
            last = done;
            ++calls;
        }, error));
        CHECK(c.job == plain);  // reassembled byte for byte
        CHECK(last == plain.size());
        CHECK(calls == c.packets.size());
        for (const std::string& p : c.packets) {
            CHECK(static_cast<std::uint8_t>(p[0]) >= 0x80);  // every packet starts a command
            // Longer only if it is a single command that cannot be split.
            if (p.size() > limit) CHECK(split_commands(p).size() == 1);
        }
    }
}

TEST_CASE("link: an upload that loses its ACKs stops and says where") {
    const LaserJob job = planned_square();
    std::string plain, error;
    REQUIRE(prepare_send(job, DeviceConfig{}, plain, error));
    FakeController c;
    c.drop_acks_after = 3;
    RuidaLink link(c, 0x88, small_packets(64));
    CHECK(!link.upload(plain, Progress{}, error));
    CHECK(error.find("upload failed at byte") != std::string::npos);
    CHECK(c.packets.size() == 4);  // nothing sent after the lost ACK
    const RdSummary s = summarize_rd(decode_rd(c.job));
    CHECK(!s.has_eof);  // so the controller never has a job to start
}

// --- transport ---------------------------------------------------------------

TEST_CASE("transport: send uploads exactly the encoded job") {
    FakeController c;
    DeviceConfig cfg;
    RuidaTransport t(c, cfg);
    std::string error;
    REQUIRE(t.open(error));
    const LaserJob job = planned_square();
    REQUIRE(t.send(job, Progress{}, error));
    std::string expected;
    REQUIRE(prepare_send(job, cfg, expected, error));
    CHECK(c.job == expected);
    const RdSummary s = summarize_rd(decode_rd(c.job));
    CHECK(s.has_eof);
    CHECK(s.file_sum_stated == s.file_sum_computed);
    CHECK(s.cuts == 4);
}

TEST_CASE("transport: send refuses an unplanned job and too much power") {
    FakeController c;
    DeviceConfig cfg;
    cfg.max_power = 50.0;
    RuidaTransport t(c, cfg);
    std::string error;

    LaserJob unplanned = planned_square();
    unplanned.planned = false;
    CHECK(!t.send(unplanned, Progress{}, error));
    CHECK(error.find("plan") != std::string::npos);

    CHECK(!t.send(planned_square(65.0), Progress{}, error));
    CHECK(error.find("at most 50.0%") != std::string::npos);
    CHECK(c.packets.empty());  // nothing went on the wire
}

TEST_CASE("transport: frame uploads a job with no cut command and no power") {
    FakeController c;
    RuidaTransport t(c, DeviceConfig{});
    std::string error;
    LaserJob job = planned_square(80.0);
    job.origin = {-5, -5};  // off the box's corners, as the fixed-origin design allows
    REQUIRE(t.frame(job, error));

    const auto cmds = decode_rd(c.job);
    const RdSummary s = summarize_rd(cmds);
    CHECK(s.has_eof);
    CHECK(s.file_sum_stated == s.file_sum_computed);
    CHECK(s.cuts == 0);
    CHECK(s.moves == 6);
    for (const RdCommand& cmd : cmds) {
        REQUIRE(cmd.spec != nullptr);
        for (const RdField& f : cmd.fields) {
            if (f.kind == 'p') CHECK(f.i == 0);  // every power field, both tubes, all kinds
        }
        CHECK(motion_of(cmd, nullptr) != RdMotion::CutAbs);
        CHECK(motion_of(cmd, nullptr) != RdMotion::CutRel);
    }
    CHECK(s.origin_x == -5000 && s.origin_y == -5000);
}

// Positions (um) after every motion command, in order, for the moves only.
static std::vector<std::pair<std::int64_t, std::int64_t>> move_targets(const std::vector<RdCommand>& cmds) {
    std::vector<std::pair<std::int64_t, std::int64_t>> out;
    std::int64_t x = 0, y = 0;
    for (const RdCommand& c : cmds) {
        char axis = 'b';
        const RdMotion m = motion_of(c, &axis);
        if (m == RdMotion::None) continue;
        if (m == RdMotion::MoveAbs || m == RdMotion::CutAbs) {
            x = c.fields[0].i;
            y = c.fields[1].i;
        } else if (axis == 'x') {
            x += c.fields[0].i;
        } else if (axis == 'y') {
            y += c.fields[0].i;
        } else {
            x += c.fields[0].i;
            y += c.fields[1].i;
        }
        out.emplace_back(x, y);
    }
    return out;
}

TEST_CASE("transport: dryrun traces every vertex in planned order with no cut and no power") {
    LaserJob raw;
    LaserLayer l;
    l.pen = 5;
    l.color = 5;
    l.speed = 100;
    l.min_power = l.max_power = 80.0;
    raw.layers.push_back(l);
    LaserPath a;
    a.pen = 5;
    a.closed = true;
    a.pts = {{0, 0}, {30, 0}, {30, 30}, {0, 30}};
    raw.paths.push_back(a);
    LaserPath b;
    b.pen = 5;
    b.closed = false;
    b.pts = {{50, 5}, {60, 7.5}, {70, 5}};
    raw.paths.push_back(b);
    LaserJob job;
    PlanReport r;
    std::string error;
    REQUIRE(plan_job(raw, job, PlanOptions{}, r, error));

    FakeController c;
    DeviceConfig cfg;
    cfg.max_power = 10.0;  // below the layer's power: a dry run asks for none, so it is not refused
    RuidaTransport t(c, cfg);
    REQUIRE(t.open(error));
    REQUIRE(t.dryrun(job, error));
    std::string expected;
    REQUIRE(prepare_dryrun(job, cfg, expected, error));
    CHECK(c.job == expected);

    const auto cmds = decode_rd(c.job);
    const RdSummary s = summarize_rd(cmds);
    CHECK(s.has_eof);
    CHECK(s.file_sum_stated == s.file_sum_computed);
    CHECK(s.cuts == 0);
    for (const RdCommand& cmd : cmds) {
        REQUIRE(cmd.spec != nullptr);
        for (const RdField& f : cmd.fields) {
            if (f.kind == 'p') CHECK(f.i == 0);
        }
        CHECK(motion_of(cmd, nullptr) != RdMotion::CutAbs);
        CHECK(motion_of(cmd, nullptr) != RdMotion::CutRel);
    }

    // Every vertex of every path, in the planned order, appears among the
    // moves (a closed path's return to its start is extra, so match in order).
    const auto targets = move_targets(cmds);
    std::size_t at = 0;
    for (const LaserPath& p : job.paths) {
        for (const Pt& q : p.pts) {
            const std::int64_t ux = std::llround(q.x * 1000.0), uy = std::llround(q.y * 1000.0);
            while (at < targets.size() && targets[at] != std::make_pair(ux, uy)) ++at;
            REQUIRE(at < targets.size());
            ++at;
        }
    }

    LaserJob unplanned = job;
    unplanned.planned = false;
    CHECK(!t.dryrun(unplanned, error));
}

TEST_CASE("transport: pulse is one cut of the computed length and speed at the given power") {
    FakeController c;
    DeviceConfig cfg;
    cfg.max_power = 40.0;
    RuidaTransport t(c, cfg);
    std::string error;
    REQUIRE(t.open(error));
    REQUIRE(t.pulse(25.0, 500, error));

    const auto cmds = decode_rd(c.job);
    const RdSummary s = summarize_rd(cmds);
    CHECK(s.has_eof);
    CHECK(s.file_sum_stated == s.file_sum_computed);
    CHECK(s.cuts == 1);
    CHECK_NEAR(s.cut_mm, 0.5, 1e-9);  // 1 mm/s for 500 ms
    int speeds = 0, powers = 0;
    for (const RdCommand& cmd : cmds) {
        for (const RdField& f : cmd.fields) {
            if (f.kind == 's' && cmd.spec->name == std::string_view("Speed Laser 1")) {
                CHECK(f.i == 1000);  // um/s
                ++speeds;
            }
            if (f.kind == 'p' && cmd.spec->key[1] == 0x01 && cmd.spec->key[0] == 0xC6) {
                CHECK_NEAR(f.d, 25.0, 0.01);
                ++powers;
            }
        }
    }
    CHECK(speeds >= 1);
    CHECK(powers >= 1);
    // The first move is to the origin: the job starts where the head is.
    const auto targets = move_targets(cmds);
    REQUIRE(targets.size() == 2);
    CHECK(targets[0].first == 0 && targets[0].second == 0);
    CHECK(targets[1].first == 500 && targets[1].second == 0);
}

TEST_CASE("transport: pulse refuses too much power and a duration out of range") {
    FakeController c;
    DeviceConfig cfg;
    cfg.max_power = 30.0;
    RuidaTransport t(c, cfg);
    std::string error;
    CHECK(!t.pulse(30.5, 500, error));
    CHECK(error.find("at most 30.0%") != std::string::npos);
    CHECK(!t.pulse(0.0, 500, error));
    CHECK(!t.pulse(10.0, 2001, error));
    CHECK(error.find("10..2000") != std::string::npos);
    CHECK(!t.pulse(10.0, 9, error));
    CHECK(!t.pulse(10.0, -5, error));
    CHECK(c.packets.empty());  // nothing went on the wire
    CHECK(t.pulse(30.0, 2000, error));  // the limits themselves are allowed
}

TEST_CASE("transport: poll reads state, position and progress but not the bed") {
    FakeController c;
    c.set_number(kMemMachineStatus, 3);
    c.set_number(kMemPosX, 1000);
    c.set_number(kMemPosY, 2000);
    c.set_number(kMemFileTotal, 9);
    c.set_number(kMemFileProgress, 4);
    RuidaTransport t(c, DeviceConfig{});
    MachineStatus s;
    std::string error;
    REQUIRE(t.poll(s, error));
    CHECK(s.has_state && s.state == 3);
    CHECK(s.has_position && s.has_progress && s.file_progress == 4);
    CHECK(!s.has_bed);
    CHECK(c.packets.size() == 5);  // five reads, no ENQ, no bed
    c.silent = true;
    CHECK(!t.poll(s, error));
}

TEST_CASE("transport: stop is D8 01") {
    FakeController c;
    RuidaTransport t(c, DeviceConfig{});
    std::string error;
    REQUIRE(t.stop(error));
    CHECK(c.stops == 1);
}

TEST_CASE("transport: status reads state, position, bed and progress") {
    FakeController c;
    c.set_number(kMemMachineStatus, 22);
    c.set_number(kMemPosX, 123456);
    c.set_number(kMemPosY, static_cast<std::uint32_t>(-2500));  // 32-bit two's complement
    c.set_number(kMemBedX, 900000);
    c.set_number(kMemBedY, 600000);
    c.set_number(kMemFileTotal, 151);
    c.set_number(kMemFileProgress, 40);
    RuidaTransport t(c, DeviceConfig{});
    MachineStatus s;
    std::string error;
    REQUIRE(t.status(s, error));
    CHECK(s.has_state && s.state == 22);
    REQUIRE(s.has_position);
    CHECK_NEAR(s.x_mm, 123.456, 1e-9);
    CHECK_NEAR(s.y_mm, -2.5, 1e-9);
    CHECK(s.has_bed && s.bed_w_mm == 900.0 && s.bed_h_mm == 600.0);
    CHECK(s.has_progress && s.file_total == 151 && s.file_progress == 40);
}

TEST_CASE("transport: counters, including a firmware string") {
    FakeController c;
    c.set_number(kMemCardId, 0x65106510);
    c.set_number(kMemPowerOnTime, 3600);
    c.set_number(kMemJobCount, 42);
    c.memory[kMemFirmware] = std::string("RDC8.01.50\0", 11);
    RuidaTransport t(c, DeviceConfig{});
    Counters k;
    std::string error;
    REQUIRE(t.counters(k, error));
    CHECK(k.has_card_id && k.card_id == 0x65106510u);
    CHECK(k.has_power_on && k.power_on == 3600);
    CHECK(k.has_jobs && k.jobs == 42);
    CHECK(k.has_firmware && k.firmware == "RDC8.01.50");
}

TEST_CASE("transport: a dead controller fails the first read, not with zeros") {
    FakeController c;
    c.silent = true;
    RuidaTransport t(c, DeviceConfig{});
    Counters k;
    MachineStatus s;
    std::string error;
    CHECK(!t.counters(k, error));
    CHECK(!t.status(s, error));
    CHECK(!error.empty());
}

// --- serial framing ----------------------------------------------------------
//
// The same link over a byte stream: bare swizzled bytes, no checksum, no ACK.

namespace {

LinkOptions quick_serial() {
    LinkOptions o;
    o.ack_timeout_ms = 50;
    o.stream_quiet_ms = 1;
    return o;
}

}  // namespace

using nlaser_test::FakeSerialController;

TEST_CASE("serial: connect asks for machine status, since ENQ gets no answer") {
    FakeSerialController c;
    RuidaLink link(c, 0x88, quick_serial());
    CHECK(link.serial_framing());
    std::string error;
    REQUIRE(link.connect(error));
    CHECK(c.enqs == 0);
    REQUIRE(!c.packets.empty());
    CHECK(c.packets[0] == std::string("\xDA\x00\x04\x00", 4));
}

TEST_CASE("serial: a silent controller fails connect, with words") {
    FakeSerialController c;
    c.silent = true;
    RuidaLink link(c, 0x88, quick_serial());
    std::string error;
    CHECK(!link.connect(error));
    CHECK(error.find("no answer") != std::string::npos);
}

TEST_CASE("serial: a command is swizzled bytes with no checksum, and no ACK is awaited") {
    FakeSerialController c;
    c.silent = true;  // nobody will ever answer, and the command must still succeed
    RuidaLink link(c, 0x88, quick_serial());
    std::string error;
    const std::string stop("\xD8\x01", 2);
    CHECK(link.command(stop, error));
    CHECK(c.wire == Swizzle(0x88).encode(stop));
    CHECK(c.wire.size() == 2);  // a UDP packet would carry two more bytes
    CHECK(link.stats().acks == 0);
    CHECK(link.stats().packets == 1);
}

TEST_CASE("serial: memory reads survive one-byte reads, stale bytes and a long tail") {
    FakeSerialController c;
    c.set_number(kMemCardId, 0x65106510);
    c.read_chunk = 1;  // the reply trickles in
    c.stale = Swizzle(0x88).encode(std::string("\xCC\xCC\x12", 3));  // leftovers from before
    RuidaLink link(c, 0x88, quick_serial());
    MemReply r;
    std::string error;
    REQUIRE(link.query(kMemCardId, r, error));
    CHECK(r.mem == kMemCardId);
    CHECK(r.has_number);
    CHECK(r.number == 0x65106510u);

    // A reply longer than 5 value bytes (a firmware string) is kept whole.
    c.memory[kMemFirmware] = std::string("RDC8.01.50\0", 11);
    REQUIRE(link.query(kMemFirmware, r, error));
    CHECK(r.raw == std::string("RDC8.01.50\0", 11));
}

TEST_CASE("serial: a stray reply left in the stream is not taken for the next answer") {
    FakeSerialController c;
    c.set_number(kMemJobCount, 1);
    c.set_number(kMemPowerOnTime, 2);
    RuidaLink link(c, 0x88, quick_serial());
    MemReply r;
    std::string error;
    // Ask for the job count behind the link's back: its reply sits unread.
    std::string ignore;
    c.write(Swizzle(0x88).encode(std::string("\xDA\x00\x04\x03", 4)), ignore);
    REQUIRE(link.query(kMemPowerOnTime, r, error));
    CHECK(r.number == 2);
}

TEST_CASE("serial: no reply to a read is an error, not zeros") {
    FakeSerialController c;
    c.silent = true;
    RuidaLink link(c, 0x88, quick_serial());
    MemReply r;
    std::string error;
    CHECK(!link.query(kMemCardId, r, error));
    CHECK(error.find("no reply") != std::string::npos);
}

TEST_CASE("serial: a whole job uploads byte for byte, in the swizzled stream") {
    const LaserJob job = planned_square();
    std::string plain, error;
    REQUIRE(prepare_send(job, DeviceConfig{}, plain, error));
    FakeSerialController c;
    LinkOptions o = quick_serial();
    o.max_packet = 64;
    RuidaLink link(c, 0x88, o);
    std::size_t last = 0;
    REQUIRE(link.upload(plain, [&](std::size_t done, std::size_t) { last = done; }, error));
    CHECK(last == plain.size());
    CHECK(c.job == plain);
    CHECK(c.wire == Swizzle(0x88).encode(plain));  // no per-packet overhead on the wire
    CHECK(summarize_rd(decode_rd(c.job)).has_eof);
}

TEST_CASE("serial: transport status, counters, send, frame and stop") {
    FakeSerialController c;
    c.set_number(kMemMachineStatus, 22);
    c.set_number(kMemPosX, 123456);
    c.set_number(kMemPosY, static_cast<std::uint32_t>(-2500));
    c.set_number(kMemBedX, 900000);
    c.set_number(kMemBedY, 600000);
    c.set_number(kMemCardId, 0x65106510);
    c.set_number(kMemJobCount, 42);
    c.memory[kMemFirmware] = std::string("RDC8.01.50\0", 11);
    DeviceConfig cfg;
    cfg.ack_timeout_ms = 50;
    RuidaTransport t(c, cfg);
    std::string error;
    REQUIRE(t.open(error));

    MachineStatus s;
    REQUIRE(t.status(s, error));
    CHECK(s.has_state && s.state == 22);
    CHECK(s.has_position);
    CHECK_NEAR(s.y_mm, -2.5, 1e-9);
    CHECK(s.has_bed && s.bed_w_mm == 900.0);

    Counters k;
    REQUIRE(t.counters(k, error));
    CHECK(k.card_id == 0x65106510u);
    CHECK(k.jobs == 42);
    CHECK(k.has_firmware && k.firmware == "RDC8.01.50");

    c.set_number(kMemPosY, 2500);  // on the bed again: the square must not leave it
    c.job.clear();
    const LaserJob job = planned_square();
    REQUIRE(t.send(job, Progress{}, error));
    std::string expected;
    REQUIRE(prepare_send(job, cfg, expected, error));
    CHECK(c.job == expected);

    c.job.clear();
    REQUIRE(t.frame(job, error));
    CHECK(summarize_rd(decode_rd(c.job)).cuts == 0);

    REQUIRE(t.stop(error));
    CHECK(c.stops == 1);
}

TEST_CASE("serial: transport refuses too much power before writing anything") {
    FakeSerialController c;
    DeviceConfig cfg;
    cfg.max_power = 50.0;
    RuidaTransport t(c, cfg);
    std::string error;
    CHECK(!t.send(planned_square(65.0), Progress{}, error));
    CHECK(c.wire.empty());
}

// --- the bed: where a job lands -----------------------------------------------

namespace {

// A top-right 900 x 600 machine whose bed the controller reports and whose
// head is at (hx, hy) mm; the config names no bed.
void top_right_controller(nlaser_test::ControllerState& c, double hx, double hy) {
    c.set_number(kMemMachineStatus, 22);
    c.set_number(kMemPosX, static_cast<std::uint64_t>(hx * 1000.0));
    c.set_number(kMemPosY, static_cast<std::uint64_t>(hy * 1000.0));
    c.set_number(kMemBedX, 900000);
    c.set_number(kMemBedY, 600000);
}

DeviceConfig top_right_cfg() {
    DeviceConfig cfg;
    cfg.origin_corner = "tr";
    cfg.ack_timeout_ms = 50;
    return cfg;
}

}  // namespace

TEST_CASE("bed: placement_in_bed accepts a job inside, refuses one that leaves, with the numbers") {
    const LaserJob job = planned_square();  // 30 x 30, origin at its corner (0,0)
    DeviceConfig cfg = top_right_cfg();
    cfg.bed_w_mm = 900.0;
    cfg.bed_h_mm = 600.0;
    MachineStatus head;
    head.has_position = true;
    head.x_mm = 100.0;  // top-right: the job extends towards +machine-x as design x DEcreases
    head.y_mm = 100.0;
    PlacementReport rep;
    std::string error;
    CHECK(placement_in_bed(job, cfg, head, rep, error));
    CHECK(rep.verified && rep.has_box);
    CHECK_NEAR(rep.x_min, 70.0, 1e-9);  // machine x = 100 - design x
    CHECK_NEAR(rep.x_max, 100.0, 1e-9);
    CHECK(rep.note.find("placement ok") == 0);
    head.x_mm = 10.0;  // 10 - 30 = -20: past the home edge
    CHECK(!placement_in_bed(job, cfg, head, rep, error));
    CHECK(error.find("leave the bed") != std::string::npos);
    CHECK(error.find("-20.000") != std::string::npos);
    head.x_mm = 899.0;  // the other side: 869..899 fits ...
    head.y_mm = 20.0;   // ... but y 20 - 30 = -10 does not
    CHECK(!placement_in_bed(job, cfg, head, rep, error));
    // A bottom-left machine grows the other way: head 880 + 30 > 900.
    cfg.origin_corner = "bl";
    head.x_mm = 880.0;
    head.y_mm = 10.0;
    CHECK(!placement_in_bed(job, cfg, head, rep, error));
    head.x_mm = 870.0;  // exactly to the edge is inside
    CHECK(placement_in_bed(job, cfg, head, rep, error));
}

TEST_CASE("bed: placement_in_bed warns, and allows, when the head or the bed cannot be read") {
    const LaserJob job = planned_square();
    DeviceConfig cfg;
    MachineStatus head;
    PlacementReport rep;
    std::string error;
    CHECK(placement_in_bed(job, cfg, head, rep, error));
    CHECK(!rep.verified);
    CHECK(rep.note.find("WARNING") == 0);
    CHECK(rep.note.find("head position") != std::string::npos);
    head.has_position = true;
    CHECK(placement_in_bed(job, cfg, head, rep, error));
    CHECK(!rep.verified);
    CHECK(rep.note.find("bed size") != std::string::npos);
}

TEST_CASE("bed: send refuses a job that leaves the bed (head and bed read from the controller), before uploading") {
    FakeSerialController c;
    top_right_controller(c, 10.0, 100.0);
    RuidaTransport t(c, top_right_cfg());
    std::string error;
    REQUIRE(t.open(error));
    c.job.clear();
    const LaserJob job = planned_square();
    CHECK(!t.send(job, Progress{}, error));
    CHECK(error.find("leave the bed") != std::string::npos);
    CHECK(c.job.empty());  // nothing of the job went out
    CHECK(!t.frame(job, error));
    CHECK(!t.dryrun(job, error));
    CHECK(c.job.empty());

    top_right_controller(c, 100.0, 100.0);
    REQUIRE(t.send(job, Progress{}, error));
    CHECK(!c.job.empty());
    CHECK(t.note().find("placement ok") == 0);
    c.job.clear();
    REQUIRE(t.frame(job, error));
    REQUIRE(t.dryrun(job, error));
}

TEST_CASE("bed: a controller that does not answer reads still gets the job, with a warning") {
    FakeSerialController c;
    top_right_controller(c, 10.0, 10.0);  // would leave the bed -- if anyone could tell
    RuidaTransport t(c, top_right_cfg());
    std::string error;
    REQUIRE(t.open(error));
    c.silent = true;
    c.job.clear();
    REQUIRE(t.send(planned_square(), Progress{}, error));
    CHECK(!c.wire.empty());
    CHECK(t.note().find("WARNING") == 0);
    CHECK(t.note().find("cannot verify") != std::string::npos);
}

TEST_CASE("bed: absolute send on a top-right machine takes the bed from the controller") {
    FakeSerialController c;
    top_right_controller(c, 5.2, 31.2);
    RuidaTransport t(c, top_right_cfg());  // no bed in the config
    std::string error;
    REQUIRE(t.open(error));
    LaserJob job = planned_square();  // design 0..30 = bed 0..30
    job.start = StartMode::Absolute;
    c.job.clear();
    REQUIRE(t.send(job, Progress{}, error));
    const RdSummary s = summarize_rd(decode_rd(c.job));
    CHECK(s.start == RdStart::Absolute);
    CHECK_NEAR(s.cut_min_x, 870.0, 1e-9);  // 900 - 30
    CHECK_NEAR(s.cut_max_x, 900.0, 1e-9);
    CHECK_NEAR(s.cut_min_y, 570.0, 1e-9);
    CHECK_NEAR(s.cut_max_y, 600.0, 1e-9);

    // The same machine with no bed anywhere: refused, in words, nothing sent.
    FakeSerialController blind;
    blind.set_number(kMemMachineStatus, 22);
    RuidaTransport t2(blind, top_right_cfg());
    REQUIRE(t2.open(error));
    blind.job.clear();
    CHECK(!t2.send(job, Progress{}, error));
    CHECK(error.find("needs the bed size") != std::string::npos);
    CHECK(blind.job.empty());
}

TEST_CASE("bed: frame of an absolute job is traced where the job will be, and ends at the box corner") {
    DeviceConfig cfg = top_right_cfg();
    cfg.bed_w_mm = 900.0;
    cfg.bed_h_mm = 600.0;
    LaserJob job = planned_square();
    job.start = StartMode::Absolute;
    std::string plain, error;
    REQUIRE(prepare_frame(job, cfg, plain, error));
    const RdSummary s = summarize_rd(decode_rd(plain));
    CHECK(s.start == RdStart::Absolute);
    CHECK(s.cuts == 0);
    CHECK(s.moves == 5);  // the box, closed: no trip to an origin
}

// --- goto and jog -------------------------------------------------------------

TEST_CASE("goto: the rapid move is D9 10, options 00 (origin, no light), two absolute coordinates") {
    std::string plain, error;
    REQUIRE(prepare_goto(100.0, 50.0, 0.0, plain, error));
    CHECK(plain == std::string("\xD9\x10\x00" "\x00\x00\x06\x0D\x20" "\x00\x00\x03\x06\x50", 13));
    // A speed puts C9 02 in front, encoded as the job's speeds are.
    std::string speed, expect_speed;
    REQUIRE(put_speed(expect_speed, 600.0));
    REQUIRE(prepare_goto(100.0, 50.0, 600.0, speed, error));
    CHECK(speed == std::string("\xC9\x02", 2) + expect_speed + plain);
    CHECK(!prepare_goto(5e6, 0.0, 0.0, plain, error));  // beyond 32 bits of micrometres
}

TEST_CASE("goto: bed coordinates map through the machine; bounds are checked") {
    DeviceConfig cfg = top_right_cfg();
    cfg.bed_w_mm = 900.0;
    cfg.bed_h_mm = 600.0;
    MachineStatus st;
    MoveReport r;
    std::string error;
    REQUIRE(plan_move(cfg, st, MoveFrame::Bed, 100.0, 50.0, r, error));
    CHECK_NEAR(r.machine_x, 800.0, 1e-9);
    CHECK_NEAR(r.machine_y, 550.0, 1e-9);
    CHECK(r.has_bed_pos);
    CHECK_NEAR(r.bed_x, 100.0, 1e-9);
    CHECK(r.note.empty());
    REQUIRE(plan_move(cfg, st, MoveFrame::Machine, 12.5, 7.0, r, error));
    CHECK_NEAR(r.machine_x, 12.5, 1e-9);
    CHECK_NEAR(r.bed_x, 887.5, 1e-9);
    CHECK(!plan_move(cfg, st, MoveFrame::Bed, 900.5, 10.0, r, error));
    CHECK(error.find("leaves the bed") != std::string::npos);
    CHECK(!plan_move(cfg, st, MoveFrame::Bed, 10.0, -0.01, r, error));
    CHECK(!plan_move(cfg, st, MoveFrame::Machine, -1.0, 10.0, r, error));
    CHECK(!plan_move(cfg, st, MoveFrame::Machine, 10.0, 600.1, r, error));
    CHECK(plan_move(cfg, st, MoveFrame::Bed, 900.0, 600.0, r, error));  // the corner itself
}

TEST_CASE("goto: no bed size known -- refuse a bed goto on a top-right machine, check one side otherwise") {
    DeviceConfig cfg = top_right_cfg();
    MachineStatus st;
    MoveReport r;
    std::string error;
    CHECK(!plan_move(cfg, st, MoveFrame::Bed, 10.0, 10.0, r, error));
    CHECK(error.find("bed size") != std::string::npos);
    REQUIRE(plan_move(cfg, st, MoveFrame::Machine, 10.0, 10.0, r, error));  // raw machine coordinates need no bed
    CHECK(!r.note.empty());
    CHECK(!plan_move(cfg, st, MoveFrame::Machine, -3.0, 10.0, r, error));
    st.has_bed = true;  // the controller's report fills the gap
    st.bed_w_mm = 900.0;
    st.bed_h_mm = 600.0;
    REQUIRE(plan_move(cfg, st, MoveFrame::Bed, 10.0, 10.0, r, error));
    CHECK_NEAR(r.machine_x, 890.0, 1e-9);
    cfg.origin_corner = "bl";  // bottom-left needs no bed to map
    st = MachineStatus{};
    REQUIRE(plan_move(cfg, st, MoveFrame::Bed, 10.0, 10.0, r, error));
    CHECK_NEAR(r.machine_x, 10.0, 1e-9);
}

TEST_CASE("jog: arithmetic in bed directions, whatever the machine's origin") {
    // Head at machine (100,100) on a top-right machine: bed +x is machine -x.
    DeviceConfig cfg = top_right_cfg();
    cfg.bed_w_mm = 900.0;
    cfg.bed_h_mm = 600.0;
    MachineStatus st;
    st.has_position = true;
    st.x_mm = 100.0;
    st.y_mm = 100.0;
    MoveReport r;
    std::string error;
    REQUIRE(plan_move(cfg, st, MoveFrame::Offset, 0.1, 0.0, r, error));
    CHECK_NEAR(r.machine_x, 99.9, 1e-9);
    CHECK_NEAR(r.machine_y, 100.0, 1e-9);
    REQUIRE(plan_move(cfg, st, MoveFrame::Offset, 0.0, 10.0, r, error));  // bed up = machine y down... i.e. -y
    CHECK_NEAR(r.machine_y, 90.0, 1e-9);
    CHECK_NEAR(r.bed_y, 510.0, 1e-9);  // 600 - 90
    CHECK(!plan_move(cfg, st, MoveFrame::Offset, -1000.0, 0.0, r, error));  // past the far edge
    st.x_mm = 0.0;
    CHECK(!plan_move(cfg, st, MoveFrame::Offset, 0.1, 0.0, r, error));  // 0 - 0.1: past machine zero
    st.has_position = false;
    CHECK(!plan_move(cfg, st, MoveFrame::Offset, 1.0, 0.0, r, error));
    CHECK(error.find("head position") != std::string::npos);
}

TEST_CASE("jog: arguments to a displacement") {
    double dx = 0, dy = 0;
    std::string error;
    REQUIRE(parse_jog({"+x"}, 0.0, dx, dy, error));
    CHECK_NEAR(dx, 1.0, 1e-12);
    CHECK_NEAR(dy, 0.0, 1e-12);
    REQUIRE(parse_jog({"-y"}, kJogFineMm, dx, dy, error));
    CHECK_NEAR(dx, 0.0, 1e-12);
    CHECK_NEAR(dy, -0.1, 1e-12);
    REQUIRE(parse_jog({"+x", "-y"}, kJogCoarseMm, dx, dy, error));
    CHECK_NEAR(dx, 10.0, 1e-12);
    CHECK_NEAR(dy, -10.0, 1e-12);
    REQUIRE(parse_jog({"x"}, 2.5, dx, dy, error));
    CHECK_NEAR(dx, 2.5, 1e-12);
    REQUIRE(parse_jog({"-3.5", "0.25"}, 0.0, dx, dy, error));  // explicit mm
    CHECK_NEAR(dx, -3.5, 1e-12);
    CHECK_NEAR(dy, 0.25, 1e-12);
    CHECK(!parse_jog({"1", "2"}, kJogFineMm, dx, dy, error));  // a step flag with distances
    CHECK(!parse_jog({"+x", "-x"}, 0.0, dx, dy, error));
    CHECK(!parse_jog({"up"}, 0.0, dx, dy, error));
    CHECK(!parse_jog({}, 0.0, dx, dy, error));
    CHECK(!parse_jog({"+x"}, -1.0, dx, dy, error));
}

TEST_CASE("goto: the transport sends the move, from the head and bed it reads") {
    FakeSerialController c;
    top_right_controller(c, 100.0, 100.0);
    RuidaTransport t(c, top_right_cfg());  // bed comes from the controller
    std::string error;
    REQUIRE(t.open(error));
    c.job.clear();
    MoveReport r;
    MachineStatus flush;  // the fake parses a command once the next one begins: any read does
    REQUIRE(t.move_head(MoveFrame::Bed, 100.0, 50.0, 0.0, r, error));
    REQUIRE(t.poll(flush, error));
    // bed (100,50) on a 900 x 600 top-right machine is machine (800, 550)
    std::string expect;
    REQUIRE(prepare_goto(800.0, 550.0, 0.0, expect, error));
    CHECK(c.job == expect);
    c.job.clear();
    REQUIRE(t.move_head(MoveFrame::Offset, 5.0, 0.0, 0.0, r, error));  // bed +x from head 100 = machine 95
    REQUIRE(t.poll(flush, error));
    REQUIRE(prepare_goto(95.0, 100.0, 0.0, expect, error));
    CHECK(c.job == expect);
    c.job.clear();
    CHECK(!t.move_head(MoveFrame::Bed, 901.0, 0.0, 0.0, r, error));
    REQUIRE(t.poll(flush, error));
    CHECK(c.job.empty());  // a refused move sends nothing
}

TEST_CASE("goto: over UDP the move is one acknowledged packet") {
    FakeController c;
    top_right_controller(c, 100.0, 100.0);
    RuidaTransport t(c, top_right_cfg());
    std::string error;
    REQUIRE(t.open(error));
    MoveReport r;
    REQUIRE(t.move_head(MoveFrame::Machine, 20.0, 30.0, 0.0, r, error));
    std::string expect;
    REQUIRE(prepare_goto(20.0, 30.0, 0.0, expect, error));
    CHECK(c.job == expect);
}

// --- listening ------------------------------------------------------------------

namespace {

std::string status_push(std::uint16_t mem, std::uint64_t v) {
    std::string r("\xDA\x01", 2);
    r.push_back(static_cast<char>((mem >> 7) & 0x7F));
    r.push_back(static_cast<char>(mem & 0x7F));
    ncad::laser::ruida::put_u35(r, v);
    return r;
}

}  // namespace

TEST_CASE("listen: over UDP one ENQ, then only what the controller pushes, decoded") {
    FakeController c;
    RuidaTransport t(c, DeviceConfig{});
    std::string error;
    REQUIRE(t.listen_begin(error));
    CHECK(c.enqs == 1);
    const std::size_t sent = c.packets.size();
    std::string raw, plain;
    CHECK(t.listen_next(raw, plain, 10, error) == 0);  // silence is silence
    c.push(status_push(kMemPosX, 123456));
    c.push(std::string("\xFF\x01\x02", 3));         // not a reply: raw
    REQUIRE(t.listen_next(raw, plain, 10, error) == 1);
    CHECK(plain == status_push(kMemPosX, 123456));
    CHECK(describe_wire(plain) == "DA 01 mem 0x0221 = 123456 (0x1E240)");
    REQUIRE(t.listen_next(raw, plain, 10, error) == 1);
    CHECK(describe_wire(plain) == "raw FF 01 02");
    CHECK(t.listen_next(raw, plain, 10, error) == 0);
    CHECK(c.packets.size() == sent);  // listening wrote nothing
}

TEST_CASE("listen: over serial nothing is sent at all, and a burst arrives whole") {
    FakeSerialController c;
    RuidaTransport t(c, DeviceConfig{});
    std::string error;
    REQUIRE(t.listen_begin(error));
    CHECK(c.wire.empty());
    c.push(status_push(kMemPosX, 5) + status_push(kMemPosY, 6));
    c.read_chunk = 4;  // arrives in pieces; still one burst
    std::string raw, plain;
    REQUIRE(t.listen_next(raw, plain, 10, error) == 1);
    CHECK(describe_wire(plain) == "DA 01 mem 0x0221 = 5 (0x5); DA 01 mem 0x0231 = 6 (0x6)");
    CHECK(c.wire.empty());
    c.push(std::string("\xCC\xCE", 2));
    REQUIRE(t.listen_next(raw, plain, 10, error) == 1);
    CHECK(describe_wire(plain) == "ACK; ENQ");
}

// --- settings dump ----------------------------------------------------------------

TEST_CASE("settings: MeerK40t's name table, ported as data") {
    CHECK(memory_name(0x0026) == "Axis Range 1, Get Frame X");
    CHECK(memory_name(0x0036) == "Axis Range 2, Get Frame Y");
    CHECK(memory_name(0x0200) == "Machine Status");
    CHECK(memory_name(0x02FE) == "Card ID");
    CHECK(memory_name(0x0181) == "PC Lock 0");
    CHECK(memory_name(0x0187) == "PC Lock 6");
    CHECK(memory_name(0x0391) == "Time for File 1 to Run");
    CHECK(memory_name(0x0263) == "Document Number +2");
    CHECK(memory_name(0x0077) == "Auto Type Space 2");  // the first of MeerK40t's two (the second is unreachable there)
    CHECK(memory_name(0x3FFF).empty());
    const std::vector<std::uint16_t> all = named_addresses();
    CHECK(all.size() > 400);
    for (std::size_t i = 1; i < all.size(); ++i) CHECK(all[i - 1] < all[i]);
}

TEST_CASE("settings: addresses are 14-bit, and the two wire bytes") {
    std::uint16_t a = 0;
    REQUIRE(parse_memory_address("0x0221", a));
    CHECK(a == kMemPosX);
    REQUIRE(parse_memory_address("04:21", a));
    CHECK(a == 0x221);
    REQUIRE(parse_memory_address("26", a));
    CHECK(a == 0x26);
    CHECK(!parse_memory_address("0x4000", a));
    CHECK(!parse_memory_address("80:00", a));  // a wire byte is 7 bits
    CHECK(!parse_memory_address("zz", a));
    CHECK(!parse_memory_address("", a));
    CHECK(format_memory_address(0x0221) == "0x0221 [04 21]");
}

TEST_CASE("settings: red, light, pointer and offset are flagged; Preferred is not red") {
    CHECK(memory_name_interesting("Axis Home Offset 1"));
    CHECK(memory_name_interesting("Offset 3 Start"));
    CHECK(memory_name_interesting("Red Dot Pointer"));
    CHECK(memory_name_interesting("Light Source"));
    CHECK(memory_name_interesting("Laser red"));
    CHECK(!memory_name_interesting("Axis Preferred Position 1, Pos X"));
    CHECK(!memory_name_interesting("Card ID"));
}

TEST_CASE("settings: the dump reads, records a missing reply and keeps going") {
    FakeSerialController c;
    c.set_number(kMemMachineStatus, 22);  // for open()
    c.set_number(0x0026, 900000);
    c.memory[0x02FF] = std::string("MEERK40T\0", 9);
    c.mute.insert(0x0036);
    DeviceConfig cfg;
    cfg.ack_timeout_ms = 20;
    RuidaTransport t(c, cfg);
    std::string error;
    REQUIRE(t.open(error));
    c.set_number(kMemMachineStatus, 22);
    std::vector<SettingRow> rows;
    std::size_t last = 0;
    REQUIRE(read_settings(t, {0x0026, 0x0036, 0x002A, 0x02FF, 0x1234}, rows, {},
                          [&](std::size_t done, std::size_t) { last = done; }, error));
    REQUIRE(rows.size() == 5);
    CHECK(last == 5);
    CHECK(rows[0].replied && rows[0].has_number && rows[0].number == 900000);
    CHECK(rows[0].name == "Axis Range 1, Get Frame X");
    CHECK(!rows[1].replied);  // muted: marked, and the dump went on
    CHECK(!rows[1].error.empty());
    CHECK(rows[2].replied && rows[2].interesting && rows[2].number == 0);  // Axis Home Offset 1
    CHECK(rows[3].replied && rows[3].text == "MEERK40T");
    CHECK(rows[4].replied && rows[4].name.empty());  // not in MeerK40t's table, still readable
    // Read-only: every write was a DA 00 memory read.
    for (const std::string& p : c.packets) {
        const bool read = p.size() == 4 && static_cast<unsigned char>(p[0]) == 0xDA && p[1] == 0x00;
        CHECK(read);
    }
}

TEST_CASE("settings: Ctrl-C (stop) ends the dump with what was read") {
    FakeController c;
    RuidaTransport t(c, DeviceConfig{});
    std::string error;
    REQUIRE(t.open(error));
    int calls = 0;
    std::vector<SettingRow> rows;
    REQUIRE(read_settings(t, {1, 2, 3, 4}, rows, [&] { return ++calls > 2; }, {}, error));
    CHECK(rows.size() == 2);
}
