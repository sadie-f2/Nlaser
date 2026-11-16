// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// The Ruida link and transport against an in-memory controller.

#include "test.hpp"

#include "fake_controller.hpp"

#include "ncad/laser/job.hpp"
#include "ncad/laser/plan.hpp"
#include "ncad/laser/ruida_decode.hpp"
#include "ncad/laser/ruida_link.hpp"
#include "ncad/laser/ruida_transport.hpp"

#include <cstring>
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
