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

#include <cstring>
#include <fstream>
#include <iterator>
#include <string>

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
    job.origin = {static_cast<double>(s.origin_x) / 1000.0, static_cast<double>(s.origin_y) / 1000.0};
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
        const RdPoint p{static_cast<double>(x) / 1000.0, static_cast<double>(y) / 1000.0};
        if (m == RdMotion::MoveAbs || m == RdMotion::MoveRel) {
            RdPath path;
            path.layer = layer;
            path.pts.push_back(p);
            job.paths.push_back(path);
        } else {
            job.paths.back().pts.push_back(p);
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
    CHECK(s.origin_x == 10000);
    CHECK(s.origin_y == 10000);
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
    std::string plain, error;
    REQUIRE(encode_rd(job, plain, error));
    const RdSummary s = summarize_rd(decode_rd(plain));
    CHECK_NEAR(s.cut_max_x, 10.0 + 1000 * 0.0333, 0.0005);  // within half a um of the true end
}

TEST_CASE("rd encode: refuses what it cannot encode, leaving the output untouched") {
    std::string plain = "untouched", error;
    RdJob job = square_job();

    job.layers[0].passes = 2;
    CHECK(!encode_rd(job, plain, error));
    CHECK(error.find("passes") != std::string::npos);

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
