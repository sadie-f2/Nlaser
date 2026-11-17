// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// The raster encoder against LightBurn's own raster files (r01 greyscale, r02
// Jarvis dither, r03 threshold, r03-pt threshold with Pass-Through), from
// tests/fixtures/lightburn. See docs/lightburn-fixtures.md, "Raster".

#include "test.hpp"

#include "ncad/laser/raster.hpp"
#include "ncad/laser/ruida_codec.hpp"
#include "ncad/laser/ruida_decode.hpp"
#include "ncad/laser/ruida_raster.hpp"

#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

using namespace ncad::laser;
using namespace ncad::laser::ruida;

namespace {

bool read_file(const std::string& rel, std::string& out) {
    std::ifstream in(std::string(NLASER_FIXTURES) + "/lightburn/" + rel, std::ios::binary);
    if (!in) return false;
    out.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    return true;
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

// Loads a fixture, decodes it (the magic is guessed: that is part of the test)
// and recovers its RdRasterJob.
struct Loaded {
    std::string file, plain;
    RdRasterJob job;
    bool ok{false};
};

void load(const char* name, Loaded& l) {
    if (!read_file(name, l.file)) {
        ::nototest::fail(__FILE__, __LINE__, std::string("fixture missing: ") + name);
        return;
    }
    std::uint8_t magic = 0;
    if (!guess_magic(l.file, magic) || magic != 0x88) {
        ::nototest::fail(__FILE__, __LINE__, std::string(name) + ": the magic guess is wrong");
        return;
    }
    l.plain = Swizzle(magic).decode(l.file);
    const auto cmds = decode_rd(l.plain);
    const RdSummary s = summarize_rd(cmds);
    if (s.unknown != 0 || s.bad_length != 0) {
        ::nototest::fail(__FILE__, __LINE__, std::string(name) + ": the decoder does not read the file cleanly");
        return;
    }
    std::string error;
    if (!raster_job_from_rd(cmds, s, l.job, error)) {
        ::nototest::fail(__FILE__, __LINE__, std::string(name) + ": " + error);
        return;
    }
    l.job.power_limit = 100.0;  // r01's per-pixel powers reach 99%
    l.ok = true;
}

bool reproduces(const char* name, Loaded* keep = nullptr) {
    Loaded local;
    Loaded& l = keep ? *keep : local;
    load(name, l);
    if (!l.ok) return false;
    std::string ours, error;
    if (!encode_rd_raster(l.job, ours, error)) {
        ::nototest::fail(__FILE__, __LINE__, std::string(name) + ": " + error);
        return false;
    }
    if (ours != l.plain) {
        ::nototest::fail(__FILE__, __LINE__, std::string(name) + "\n       " + first_difference(ours, l.plain));
        return false;
    }
    return Swizzle(0x88).encode(ours) == l.file;
}

bool load_png(const char* name, GreyImage& img) {
    std::string error;
    return load_grey_image(std::string(NLASER_FIXTURES) + "/lightburn/" + name, img, error);
}

}  // namespace

TEST_CASE("raster rd: the magic guess and the decoder read all four raster files cleanly") {
    for (const char* n : {"r01.rd", "r02.rd", "r03.rd", "r03-pt.rd"}) {
        Loaded l;
        load(n, l);
        CHECK(l.ok);
    }
}

TEST_CASE("raster rd: greyscale power is float32 g/254 of 16383") {
    CHECK(raster_level_to_raw(0) == 0);
    CHECK(raster_level_to_raw(1) == 65);
    CHECK(raster_level_to_raw(9) == 580);   // 580.5 mathematically; LightBurn's float rounds down
    CHECK(raster_level_to_raw(3) == 194);
    CHECK(raster_level_to_raw(252) == 16254);
    CHECK(raster_level_to_raw(255) == 16383);  // clamped (guess; the wedge never reaches it)
}

TEST_CASE("raster rd: r03-pt (threshold, pass-through) re-encodes byte for byte") { CHECK(reproduces("r03-pt.rd")); }
TEST_CASE("raster rd: r03 (threshold) re-encodes byte for byte") { CHECK(reproduces("r03.rd")); }
TEST_CASE("raster rd: r02 (Jarvis dither, trimmed lines, diagonal moves) re-encodes byte for byte") {
    CHECK(reproduces("r02.rd"));
}
TEST_CASE("raster rd: r01 (greyscale, per-pixel power) re-encodes byte for byte") { CHECK(reproduces("r01.rd")); }

TEST_CASE("raster rd: the recovered rasters are what the lbrn2 says") {
    Loaded l;
    load("r03.rd", l);
    REQUIRE(l.ok);
    CHECK(l.job.ref_point == 1);
    CHECK(l.job.raster.cols == 303);
    CHECK(l.job.raster.rows == 60);
    CHECK(l.job.raster.first_forward);
    CHECK(!l.job.raster.grayscale);
    CHECK(!l.job.lead_wm6);
    CHECK_NEAR(l.job.layer.speed_mm_s, 450.012, 1e-6);
    CHECK_NEAR(l.job.layer.min_power, 40.0, 0.01);
    CHECK_NEAR(l.job.pointer_offset.x, 3.0, 1e-9);
    CHECK_NEAR(l.job.pointer_offset.y, 34.0, 1e-9);
    Loaded g;
    load("r01.rd", g);
    REQUIRE(g.ok);
    CHECK(g.job.raster.grayscale);
    CHECK(!g.job.raster.first_forward);
    CHECK(g.job.lead_wm6);
    CHECK(g.job.raster.cols == 302);
}

namespace {

RdRasterJob lightburn_layer() {
    RdRasterJob j;
    j.layer.speed_mm_s = 450.012;
    j.layer.min_power = 40.0;
    j.layer.max_power = 50.0;
    j.layer.min_power2 = 10.0;
    j.layer.max_power2 = 20.0;
    j.layer.rgb = 0x000000;
    j.layer.air = true;
    j.ref_point = 1;
    j.pointer_offset = {3.0, 34.0};
    return j;
}

}  // namespace

TEST_CASE("raster rd: r03-pt is produced from the PNG alone") {
    GreyImage img;
    REQUIRE(load_png("R03-lines.png", img));
    // Pass-through: one pixel per image pixel, 100 um. The grid sits half a
    // pixel in from the image's file-frame edges; that is LightBurn's, and the
    // only thing taken from the file.
    RdRasterGeometry geo;
    geo.width_mm = 50.0;
    geo.height_mm = 10.0;
    geo.pitch_um = 100.0;
    geo.mirror_x = geo.mirror_y = true;  // top-right machine
    geo.file_x_lo_um = -47000.0;         // K.x 3 - 50
    geo.file_y_hi_um = 44000.0;          // K.y 34 + 10
    geo.x_phase_um = 50.0;
    geo.y_phase_um = 50.0;
    geo.first_forward = false;
    RdRasterSettings set;
    set.mode = RdRasterMode::Threshold;
    set.negative = true;  // LightBurn's "negative": light pixels burn
    RdRasterJob j = lightburn_layer();
    std::string error;
    REQUIRE(make_rd_raster(img, geo, set, j.raster, error));
    j.lead_wm6 = true;
    std::string ours, file;
    REQUIRE(encode_rd_raster(j, ours, error));
    REQUIRE(read_file("r03-pt.rd", file));
    const std::string lb = Swizzle(0x88).decode(file);
    if (ours != lb) ::nototest::fail(__FILE__, __LINE__, "r03-pt from the PNG\n       " + first_difference(ours, lb));
    CHECK(ours == lb);
}

TEST_CASE("raster rd: r03 is produced from the PNG with LightBurn's measured grid") {
    GreyImage img;
    REQUIRE(load_png("R03-lines.png", img));
    // 154 dpi. The grid phase (0.25 um in x, -220.14 um in y from the image's
    // edges) and the pitch (a hair above 25400/154: the tie-breaking noise of
    // LightBurn's float arithmetic) are fitted to the file; see the docs.
    RdRasterGeometry geo;
    geo.width_mm = 50.0;
    geo.height_mm = 10.0;
    geo.pitch_um = 164.93512;
    geo.mirror_x = geo.mirror_y = true;
    geo.file_x_lo_um = -47000.0;
    geo.file_y_hi_um = 44000.0;
    geo.x_phase_um = 0.25456;
    geo.y_phase_um = -220.1354;
    geo.first_forward = true;
    RdRasterSettings set;
    set.mode = RdRasterMode::Threshold;
    set.negative = true;
    RdRasterJob j = lightburn_layer();
    std::string error;
    REQUIRE(make_rd_raster(img, geo, set, j.raster, error));
    std::string ours, file;
    REQUIRE(encode_rd_raster(j, ours, error));
    REQUIRE(read_file("r03.rd", file));
    const std::string lb = Swizzle(0x88).decode(file);
    if (ours != lb) ::nototest::fail(__FILE__, __LINE__, "r03 from the PNG\n       " + first_difference(ours, lb));
    CHECK(ours == lb);
}

TEST_CASE("raster rd: refusals") {
    Loaded l;
    load("r01.rd", l);
    REQUIRE(l.ok);
    std::string out, error;
    // Greyscale powers reach 99%: the default hard limit of 80% refuses.
    RdRasterJob j = l.job;
    j.power_limit = kDefaultPowerLimit;
    CHECK(!encode_rd_raster(j, out, error));
    CHECK(error.find("hard limit") != std::string::npos);
    j = l.job;
    j.layer.max_power = 90.0;
    j.power_limit = 80.0;
    CHECK(!encode_rd_raster(j, out, error));
    j = l.job;
    j.layer.passes = 2;
    CHECK(!encode_rd_raster(j, out, error));
    j = l.job;
    j.raster.level.clear();
    CHECK(!encode_rd_raster(j, out, error));
    // A blank raster fires nothing.
    Loaded t;
    load("r03.rd", t);
    REQUIRE(t.ok);
    j = t.job;
    for (auto& v : j.raster.level) v = 0;
    CHECK(!encode_rd_raster(j, out, error));
}
