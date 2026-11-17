// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// The .rd encoder.
//
// The central test rebuilds each LightBurn fixture as an RdJob -- its layers'
// settings, its paths in order, its origin -- and requires our encoder to
// produce LightBurn's exact bytes. Everything the encoder DERIVES (header
// boxes, part bounds, the choice of absolute or relative motion, the cut
// length, the file sum) is then checked against LightBurn by construction.

#include "test.hpp"

#include "ncad/laser/ruida_codec.hpp"
#include "ncad/laser/ruida_decode.hpp"
#include "ncad/laser/ruida_encode.hpp"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

using namespace ncad::laser::ruida;

namespace {

bool read_fixture(const char* name, std::string& out) {
    const std::string path = std::string(NLASER_FIXTURES) + "/" + name;
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    out.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    return true;
}

bool is(const RdCommand& c, const char* name) { return c.spec && std::strcmp(c.spec->name, name) == 0; }

// Reads an RdJob back out of a decoded file: settings from the header's part
// table and each layer block, geometry from the motions.
RdJob job_from_rd(const std::vector<RdCommand>& cmds, const RdSummary& s) {
    RdJob job;
    job.layout = RdLayout::Legacy;
    std::size_t pass = 0;  // passes seen on the current path (current layout)
    // A pass starts with the short block, and a later path's Move comes after
    // it, so a pass is counted at its first cut, not at the block.
    bool sf_seen = false;
    // K, read from the header in either mode. The points below are the file's
    // own coordinates minus K with a zero origin and an identity machine, so
    // the encoder's "machine(design) - machine(origin) + K" gives the file's
    // moves back. (This is the shortcut; the from-design tests do not use it.)
    const std::int64_t kx = s.origin_x, ky = s.origin_y;
    job.pointer_offset = {static_cast<double>(kx) / 1000.0, static_cast<double>(ky) / 1000.0};
    job.start = s.start;
    std::int64_t x = 0, y = 0;
    std::size_t layer = 0;
    bool in_layers = false;
    auto part_layer = [&](std::int64_t part) -> RdLayer& {
        const auto p = static_cast<std::size_t>(part);
        if (job.layers.size() <= p) job.layers.resize(p + 1);
        return job.layers[p];
    };
    for (const RdCommand& c : cmds) {
        if (!c.spec) continue;
        if (is(c, "Speed Laser 1 Part")) part_layer(c.fields[0].i).speed_mm_s = c.fields[1].d;
        if (is(c, "Min Power 1 Part")) part_layer(c.fields[0].i).min_power = c.fields[1].d;
        if (is(c, "Max Power 1 Part")) part_layer(c.fields[0].i).max_power = c.fields[1].d;
        if (is(c, "Min Power 2 Part")) part_layer(c.fields[0].i).min_power2 = c.fields[1].d;
        if (is(c, "Max Power 2 Part")) part_layer(c.fields[0].i).max_power2 = c.fields[1].d;
        // The current layout is the one with a Work Mode 6 block per pass and
        // a Speed Axis in front of every Layer End.
        if (is(c, "Speed Axis")) job.axis_speed_mm_s = c.fields[0].d;
        if (is(c, "Work Mode 6")) {
            job.layout = RdLayout::Current;
            sf_seen = true;
        }
        if (is(c, "Layer Color Part")) {
            const auto v = static_cast<std::uint32_t>(c.fields[1].i);
            part_layer(c.fields[0].i).rgb = ((v & 0xFFu) << 16) | (v & 0xFF00u) | ((v >> 16) & 0xFFu);
        }
        if (is(c, "Layer Number Part")) {
            layer = static_cast<std::size_t>(c.fields[0].i);
            in_layers = true;
        }
        if (in_layers && is(c, "Air Assist On")) job.layers[layer].air = true;
        if (in_layers && is(c, "Air Assist Off")) job.layers[layer].air = false;

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
        const RdPoint p{static_cast<double>(x - kx) / 1000.0, static_cast<double>(y - ky) / 1000.0};
        if (m == RdMotion::MoveAbs || m == RdMotion::MoveRel) {
            RdPath path;
            path.layer = layer;
            path.pts.push_back(p);
            job.paths.push_back(path);
            pass = 0;
        } else {
            if (sf_seen) {
                sf_seen = false;
                ++pass;
                if (in_layers) {
                    int& n = job.layers[layer].passes;
                    n = std::max(n, static_cast<int>(pass));
                }
            }
            // Later passes repeat the first; the pass count carries them.
            if (pass <= 1) job.paths.back().pts.push_back(p);
        }
    }
    return job;
}

std::string first_difference(const std::string& a, const std::string& b) {
    RdTextOptions o;
    o.summary = false;
    const std::string ta = rd_to_text(a, o), tb = rd_to_text(b, o);
    std::size_t la = 0, lb = 0;
    while (la < ta.size() && lb < tb.size()) {
        const std::size_t ea = ta.find('\n', la), eb = tb.find('\n', lb);
        const std::string sa = ta.substr(la, ea - la), sb = tb.substr(lb, eb - lb);
        if (sa != sb) return "ours:      " + sa + "\n       lightburn: " + sb;
        la = ea + 1;
        lb = eb + 1;
    }
    return "length differs";
}

void check_reproduces(const char* name) {
    std::string file;
    if (!read_fixture(name, file)) {
        ::nototest::fail(__FILE__, __LINE__, std::string("fixture missing: ") + name);
        return;
    }
    const std::string lightburn = Swizzle(0x88).decode(file);
    const auto cmds = decode_rd(lightburn);
    const RdSummary s = summarize_rd(cmds);
    const RdJob job = job_from_rd(cmds, s);

    std::string ours, error;
    const bool ok = encode_rd(job, ours, error);
    if (!ok) ::nototest::fail(__FILE__, __LINE__, std::string(name) + ": " + error);
    if (ok && ours != lightburn) ::nototest::fail(__FILE__, __LINE__, std::string(name) + "\n       " + first_difference(ours, lightburn));
    // And the swizzled file is LightBurn's file, byte for byte.
    if (ok) CHECK(Swizzle(0x88).encode(ours) == file);
}

RdJob square_job() {
    RdJob job;
    RdLayer L;
    L.speed_mm_s = 20.0;
    L.min_power = 60.0;
    L.max_power = 65.0;
    L.rgb = 0xFF0000;
    job.layers.push_back(L);
    RdPath p;
    p.closed = true;
    p.pts = {{10, 10}, {60, 10}, {60, 60}, {10, 60}};
    job.paths.push_back(p);
    job.origin = {10, 10};
    return job;
}

}  // namespace

TEST_CASE("rd encode: reproduces LightBurn's power test byte for byte (origin bottom-left)") {
    check_reproduces("lightburn_power_test.rd");
}

TEST_CASE("rd encode: reproduces LightBurn's power test byte for byte (origin bottom-right)") {
    check_reproduces("lightburn_power_test_bengal.rd");
}

TEST_CASE("rd encode: reproduces LightBurn's box joint byte for byte (origin top-left, relative moves)") {
    check_reproduces("lightburn_box_joint.rd");
}

TEST_CASE("rd encode: reproduces LightBurn's f01 byte for byte (current layout, top-right machine)") {
    check_reproduces("lightburn/f01.rd");
}

TEST_CASE("rd encode: reproduces f02-p2 and f02-p3 byte for byte (passes)") {
    check_reproduces("lightburn/f02-p2.rd");
    check_reproduces("lightburn/f02-p3.rd");
}

TEST_CASE("rd encode: reproduces f05, f06-BR and f07 byte for byte (centre and corner origins, circles, passes)") {
    check_reproduces("lightburn/f05.rd");
    check_reproduces("lightburn/f06-BR.rd");
    check_reproduces("lightburn/f07.rd");
}

TEST_CASE("rd encode: a closed square decodes back to its geometry and settings") {
    std::string plain, error;
    REQUIRE(encode_rd(square_job(), plain, error));
    const auto cmds = decode_rd(plain);
    const RdSummary s = summarize_rd(cmds);
    CHECK(s.unknown == 0);
    CHECK(s.bad_length == 0);
    CHECK(s.file_sum_stated == s.file_sum_computed);
    CHECK(s.has_eof);
    CHECK(s.cuts == 4);
    CHECK_NEAR(s.cut_mm, 200.0, 1e-9);
    CHECK(s.has_origin);
    // The summary's origin is Document Min Point - Process Top Left = K, which
    // is 0 with no pointer offset; the job's origin is where the box sits
    // relative to it (the corner name), not a coordinate in the file.
    CHECK(s.origin_x == 0);
    CHECK(s.origin_y == 0);
    CHECK(std::strcmp(s.origin_corner, "bottom-left") == 0);
    bool red = false;
    for (const RdCommand& c : cmds) {
        if (is(c, "Layer Color Part")) red = (c.fields[1].i == 0x0000FF);  // stored 0xBBGGRR
    }
    CHECK(red);
}

TEST_CASE("rd encode: the origin may sit off the job's box") {
    // The fixed-origin design: a job whose origin is NOT a corner. The header
    // then describes a box that does not touch zero; whether the controller
    // places it correctly is the frame test on hardware.
    RdJob job = square_job();
    job.origin = {0, 0};
    std::string plain, error;
    REQUIRE(encode_rd(job, plain, error));
    const RdSummary s = summarize_rd(decode_rd(plain));
    REQUIRE(s.has_origin);
    CHECK(s.origin_x == 0);
    CHECK(s.origin_y == 0);
    CHECK(s.box_min_x == 10000);
    CHECK(s.box_max_y == 60000);
    CHECK(std::strcmp(s.origin_corner, "other") == 0);
}

TEST_CASE("rd encode: a step of exactly 8.192 mm is absolute, never a wrapped relative") {
    RdJob job = square_job();
    job.paths[0].closed = false;
    job.paths[0].pts = {{20, 20}, {28.191, 20}, {36.383, 20}};  // +8.191 then +8.192
    job.origin = {0, 0};  // written coordinates are design minus origin
    std::string plain, error;
    REQUIRE(encode_rd(job, plain, error));
    std::vector<std::string> names;
    for (const RdCommand& c : decode_rd(plain)) {
        if (motion_of(c, nullptr) != RdMotion::None) names.push_back(c.spec->name);
    }
    REQUIRE(names.size() == 3);
    CHECK(names[0] == "Move Abs");
    CHECK(names[1] == "Cut Rel X");
    CHECK(names[2] == "Cut Abs");
    const RdSummary s = summarize_rd(decode_rd(plain));
    CHECK_NEAR(s.cut_max_x, 36.383, 1e-9);
}

TEST_CASE("rd encode: rounding never accumulates over many small relative steps") {
    RdJob job = square_job();
    job.paths[0].closed = false;
    job.paths[0].pts.clear();
    for (int k = 0; k <= 1000; ++k) job.paths[0].pts.push_back({10.0 + k * 0.0333, 10.0});
    job.origin = {0, 0};  // written coordinates are design minus origin
    std::string plain, error;
    REQUIRE(encode_rd(job, plain, error));
    const RdSummary s = summarize_rd(decode_rd(plain));
    CHECK_NEAR(s.cut_max_x, 10.0 + 1000 * 0.0333, 0.0005);  // within half a um of the true end
}

TEST_CASE("rd encode: refuses what it cannot encode, leaving the output untouched") {
    std::string plain = "untouched", error;
    RdJob job = square_job();

    job.layers[0].passes = 0;
    CHECK(!encode_rd(job, plain, error));
    CHECK(error.find("passes") != std::string::npos);

    // The legacy layout has no sample of passes.
    job.layers[0].passes = 2;
    job.layout = RdLayout::Legacy;
    CHECK(!encode_rd(job, plain, error));
    CHECK(error.find("passes") != std::string::npos);
    job.layout = RdLayout::Current;
    CHECK(encode_rd(job, plain, error));
    plain = "untouched";

    job = square_job();
    job.layers[0].max_power = 101.0;
    CHECK(!encode_rd(job, plain, error));

    job = square_job();
    job.layers[0].min_power = 70.0;
    CHECK(!encode_rd(job, plain, error));

    job = square_job();
    job.layers[0].speed_mm_s = 0.0;
    CHECK(!encode_rd(job, plain, error));

    job = square_job();
    job.layers.push_back(job.layers[0]);
    RdPath a = job.paths[0];
    a.layer = 1;
    job.paths.push_back(a);
    job.paths.push_back(job.paths[0]);  // layer 0 again, after layer 1
    CHECK(!encode_rd(job, plain, error));
    CHECK(error.find("contiguous") != std::string::npos);

    job = square_job();
    job.paths[0].pts = {{0, 0}};
    CHECK(!encode_rd(job, plain, error));

    job = square_job();
    job.paths[0].pts[1].x = 1e9;
    CHECK(!encode_rd(job, plain, error));

    CHECK(!encode_rd(RdJob{}, plain, error));
    CHECK(plain == "untouched");
}

namespace {

// Every move and cut position, in um, in file order.
std::vector<std::pair<std::int64_t, std::int64_t>> motion_points(const std::vector<RdCommand>& cmds) {
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
        out.push_back({x, y});
    }
    return out;
}

// The 40 mm DXF square of the LightBurn fixtures, as the layer LightBurn used.
RdJob dxf_square_job() {
    RdJob job;
    RdLayer L;
    L.speed_mm_s = 100.0;
    L.min_power = L.max_power = 15.0;
    L.min_power2 = 10.0;
    L.max_power2 = 20.0;
    L.rgb = 0xFF0000;
    job.layers.push_back(L);
    RdPath p;
    p.closed = true;
    // LightBurn starts the square at its job origin (the top-left, where the
    // head is) and cuts the loop from there.
    p.pts = {{0, 40}, {0, 0}, {40, 0}, {40, 40}};
    job.paths.push_back(p);
    job.origin = {0, 40};
    job.machine.origin = MachineCorner::TopRight;
    job.pointer_offset = {5.2, 31.2};  // the red-dot offset in LightBurn's device settings
    return job;
}

}  // namespace

TEST_CASE("rd encode: the DXF square on a top-right machine with pointer offset (5.2, 31.2) IS f01") {
    // Built from DESIGN coordinates, no job_from_rd: file = K + (M(design) -
    // M(origin)), M negating both axes, which is how LightBurn writes f01.
    std::string file;
    REQUIRE(read_fixture("lightburn/f01.rd", file));
    std::string plain, error;
    REQUIRE(encode_rd(dxf_square_job(), plain, error));
    CHECK(plain == Swizzle(0x88).decode(file));

    // Without the offset the same job is the same shape written K lower-left.
    RdJob bare = dxf_square_job();
    bare.pointer_offset = {};
    std::string plain0;
    REQUIRE(encode_rd(bare, plain0, error));
    const auto with = motion_points(decode_rd(plain));
    const auto without = motion_points(decode_rd(plain0));
    REQUIRE(with.size() == without.size());
    for (std::size_t i = 0; i < with.size(); ++i) {
        CHECK(with[i].first == without[i].first + 5200);
        CHECK(with[i].second == without[i].second + 31200);
    }
    // The header boxes are relative to the origin, so K does not enter them.
    const RdSummary a = summarize_rd(decode_rd(plain));
    const RdSummary b = summarize_rd(decode_rd(plain0));
    CHECK(a.box_min_x == b.box_min_x);
    CHECK(a.box_max_y == b.box_max_y);
    CHECK(a.origin_x == 5200);
    CHECK(a.origin_y == 31200);
    CHECK(b.origin_x == 0);
}

TEST_CASE("rd encode: absolute with the pointer offset reproduces f03 from the workspace position") {
    // f03: the square at workspace (221.666, 211.024) on a top-right machine.
    // Absolute mode takes the machine-frame position and adds K.
    std::string file;
    REQUIRE(read_fixture("lightburn/f03.rd", file));
    RdJob job = dxf_square_job();
    job.start = RdStart::Absolute;
    job.machine = RdMachine{};  // the workspace position is already machine coordinates
    job.layers[0].passes = 2;  // f03 was saved with Passes still at 2
    job.paths[0].pts = {{221.666, 211.024}, {261.666, 211.024}, {261.666, 251.024}, {221.666, 251.024}};
    std::string plain, error;
    REQUIRE(encode_rd(job, plain, error));
    CHECK(plain == Swizzle(0x88).decode(file));
}

TEST_CASE("rd encode: each machine origin corner and flip negates the right axes") {
    struct Want { MachineCorner c; bool fx, fy; int sx, sy; };
    const Want wants[] = {
        {MachineCorner::BottomLeft, false, false, 1, 1},   {MachineCorner::BottomRight, false, false, -1, 1},
        {MachineCorner::TopLeft, false, false, 1, -1},     {MachineCorner::TopRight, false, false, -1, -1},
        {MachineCorner::BottomLeft, true, false, -1, 1},   {MachineCorner::TopRight, true, true, 1, 1},
    };
    for (const Want& w : wants) {
        RdJob job = square_job();  // (10,10)-(60,60), origin (10,10)
        job.machine = {w.c, w.fx, w.fy};
        CHECK(job.machine.sign_x() == w.sx);
        CHECK(job.machine.sign_y() == w.sy);
        std::string plain, error;
        REQUIRE(encode_rd(job, plain, error));
        const auto cmds = decode_rd(plain);
        const RdSummary s = summarize_rd(cmds, job.machine);
        // Displacements from the origin (10,10), with the machine's signs.
        CHECK(s.cut_min_x == (w.sx > 0 ? 0.0 : -50.0));
        CHECK(s.cut_max_x == (w.sx > 0 ? 50.0 : 0.0));
        CHECK(s.cut_min_y == (w.sy > 0 ? 0.0 : -50.0));
        CHECK(s.cut_max_y == (w.sy > 0 ? 50.0 : 0.0));
        // The design origin is the box's bottom-left whatever the machine.
        CHECK(std::strcmp(s.origin_corner, "bottom-left") == 0);
        CHECK(s.origin_x == 0);  // K
        CHECK(s.origin_y == 0);
    }
}

TEST_CASE("rd encode: job_from_rd recovers layout, passes and the axis speed") {
    for (const auto& w : {std::pair<const char*, int>{"lightburn/f01.rd", 1}, {"lightburn/f02-p2.rd", 2},
                          {"lightburn/f02-p3.rd", 3}, {"lightburn/f06-BR.rd", 2}}) {
        std::string file;
        REQUIRE(read_fixture(w.first, file));
        const std::string plain = Swizzle(0x88).decode(file);
        const auto cmds = decode_rd(plain);
        const RdJob job = job_from_rd(cmds, summarize_rd(cmds));
        CHECK(job.layout == RdLayout::Current);
        REQUIRE(job.layers.size() == 1);
        CHECK(job.layers[0].passes == w.second);
        CHECK(job.paths.size() == 1);
        CHECK(job.paths[0].pts.size() == 5);  // the Move and one pass of four cuts
        CHECK_NEAR(job.axis_speed_mm_s, 15.0, 1e-9);
    }
    std::string file;
    REQUIRE(read_fixture("lightburn_box_joint.rd", file));
    const auto cmds = decode_rd(Swizzle(0x88).decode(file));
    CHECK(job_from_rd(cmds, summarize_rd(cmds)).layout == RdLayout::Legacy);
}

TEST_CASE("rd encode: reproduces f03 byte for byte (Start From = Absolute Coords)") {
    check_reproduces("lightburn/f03.rd");
    std::string file;
    REQUIRE(read_fixture("lightburn/f03.rd", file));
    const auto cmds = decode_rd(Swizzle(0x88).decode(file));
    const RdSummary s = summarize_rd(cmds);
    CHECK(s.start == RdStart::Absolute);
    const RdJob job = job_from_rd(cmds, s);
    CHECK(job.start == RdStart::Absolute);
    CHECK_NEAR(job.pointer_offset.x, 5.2, 1e-9);
    CHECK_NEAR(job.pointer_offset.y, 31.2, 1e-9);
}

TEST_CASE("rd encode: absolute mode changes the header opening only") {
    // On the bed (positive machine coordinates), a job's absolute form differs
    // from its current-position form only in the opening: Ref Point 2 + Set
    // Absolute for Ref Point 0. f01 stays a current-mode file.
    std::string file;
    REQUIRE(read_fixture("lightburn/f01.rd", file));
    CHECK(summarize_rd(decode_rd(Swizzle(0x88).decode(file))).start == RdStart::Current);

    RdJob job = square_job();
    std::string cur, abs, error;
    REQUIRE(encode_rd(job, cur, error));
    job.start = RdStart::Absolute;
    REQUIRE(encode_rd(job, abs, error));
    const RdSummary s = summarize_rd(decode_rd(abs));
    CHECK(s.start == RdStart::Absolute);
    CHECK(s.file_sum_stated == s.file_sum_computed);
    CHECK(abs.size() == cur.size() + 2);
}

TEST_CASE("rd encode: passes multiply the cuts and the length counter; an open path returns to its start") {
    RdJob job = square_job();
    job.paths[0].closed = false;
    job.paths[0].pts = {{10, 10}, {30, 10}, {30, 30}};
    job.layers[0].passes = 3;
    std::string plain, error;
    REQUIRE(encode_rd(job, plain, error));
    const auto cmds = decode_rd(plain);
    const RdSummary s = summarize_rd(cmds);
    CHECK(s.unknown == 0);
    CHECK(s.file_sum_stated == s.file_sum_computed);
    CHECK(s.cuts == 6);                // two cuts per pass
    CHECK(s.moves == 3);               // the first, and a return before passes 2 and 3
    CHECK_NEAR(s.cut_mm, 120.0, 1e-9);
    std::size_t work_modes = 0;
    for (const RdCommand& c : cmds) work_modes += is(c, "Work Mode 6") ? 1 : 0;
    CHECK(work_modes == 3);
}

TEST_CASE("rd encode: the axis speed is a field, not a constant") {
    RdJob job = square_job();
    job.axis_speed_mm_s = 25.0;
    std::string plain, error;
    REQUIRE(encode_rd(job, plain, error));
    std::size_t n = 0;
    for (const RdCommand& c : decode_rd(plain)) {
        if (is(c, "Speed Axis")) {
            CHECK_NEAR(c.fields[0].d, 25.0, 1e-9);
            ++n;
        }
    }
    CHECK(n == 3);  // before each of the three Layer Ends: layer, pass short block, pass settings
}

TEST_CASE("rd encode: absolute start refuses a job past the home corner") {
    // A top-right machine negates design coordinates; in absolute mode that
    // puts the job at negative machine coordinates, off the bed.
    RdJob job = square_job();
    job.start = RdStart::Absolute;
    job.machine.origin = MachineCorner::TopRight;
    std::string plain = "untouched", error;
    CHECK(!encode_rd(job, plain, error));
    CHECK(error.find("bed size") != std::string::npos);
    CHECK(plain == "untouched");
}

// --- absolute mode: design coordinates are bed coordinates -----------------

TEST_CASE("rd encode: absolute on a top-right machine maps bed coordinates with the bed size") {
    // Bed 900 x 600, origin top-right: bed (x,y) is machine (900 - x, 600 - y).
    // The square is 10..60 on the bed, so the machine sees 840..890 / 540..590.
    RdJob job = square_job();
    job.start = RdStart::Absolute;
    job.machine.origin = MachineCorner::TopRight;
    job.machine.bed_w_mm = 900.0;
    job.machine.bed_h_mm = 600.0;
    std::string plain, error;
    REQUIRE(encode_rd(job, plain, error));
    const RdSummary s = summarize_rd(decode_rd(plain));
    CHECK(s.start == RdStart::Absolute);
    CHECK(s.have_cut_bounds);
    CHECK_NEAR(s.cut_min_x, 840.0, 1e-9);
    CHECK_NEAR(s.cut_max_x, 890.0, 1e-9);
    CHECK_NEAR(s.cut_min_y, 540.0, 1e-9);
    CHECK_NEAR(s.cut_max_y, 590.0, 1e-9);
    CHECK(s.file_sum_stated == s.file_sum_computed);
}

TEST_CASE("rd encode: absolute on a bottom-left machine is the identity, with or without a bed") {
    RdJob job = square_job();
    job.start = RdStart::Absolute;
    std::string without, with, error;
    REQUIRE(encode_rd(job, without, error));
    job.machine.bed_w_mm = 900.0;
    job.machine.bed_h_mm = 600.0;
    REQUIRE(encode_rd(job, with, error));
    CHECK(without == with);
    const RdSummary s = summarize_rd(decode_rd(with));
    CHECK_NEAR(s.cut_min_x, 10.0, 1e-9);
    CHECK_NEAR(s.cut_max_y, 60.0, 1e-9);
}

TEST_CASE("rd encode: absolute honours flips like the origin corner (a flipped axis counts from the far side)") {
    RdJob job = square_job();
    job.start = RdStart::Absolute;
    job.machine.flip_x = true;  // bottom-left origin, X wired the other way: x_m = W - x
    job.machine.bed_w_mm = 900.0;
    job.machine.bed_h_mm = 600.0;
    std::string plain, error;
    REQUIRE(encode_rd(job, plain, error));
    const RdSummary s = summarize_rd(decode_rd(plain));
    CHECK_NEAR(s.cut_min_x, 840.0, 1e-9);
    CHECK_NEAR(s.cut_min_y, 10.0, 1e-9);
}

TEST_CASE("rd encode: absolute refuses a job that leaves the bed, on either side, with the numbers") {
    RdJob job = square_job();
    job.start = RdStart::Absolute;
    job.machine.bed_w_mm = 100.0;
    job.machine.bed_h_mm = 100.0;
    job.paths[0].pts = {{10, 10}, {120, 10}, {120, 60}, {10, 60}};  // past W
    std::string plain = "untouched", error;
    CHECK(!encode_rd(job, plain, error));
    CHECK(error.find("leaves the bed") != std::string::npos);
    CHECK(error.find("120.000") != std::string::npos);
    CHECK(plain == "untouched");
    job.paths[0].pts = {{-5, 10}, {20, 10}, {20, 60}};  // before 0
    CHECK(!encode_rd(job, plain, error));
    CHECK(error.find("leaves the bed") != std::string::npos);
    job.paths[0].pts = {{0, 0}, {100, 0}, {100, 100}};  // exactly the bed: fine
    CHECK(encode_rd(job, plain, error));
}

TEST_CASE("rd encode: absolute on a non-bottom-left machine without a bed size refuses, in words") {
    for (MachineCorner c : {MachineCorner::TopRight, MachineCorner::BottomRight, MachineCorner::TopLeft}) {
        RdJob job = square_job();
        job.start = RdStart::Absolute;
        job.machine.origin = c;
        std::string plain, error;
        CHECK(!encode_rd(job, plain, error));
        CHECK(error.find("needs the bed size") != std::string::npos);
        CHECK(error.find("lasers.conf") != std::string::npos);
    }
}
