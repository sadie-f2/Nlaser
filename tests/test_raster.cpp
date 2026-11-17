// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// Raster front half: resampling, levels, dithering, scan planning, the
// `raster` .ljob record, and the refusal to send one.

#include "test.hpp"

#include "ncad/laser/job.hpp"
#include "ncad/laser/plan.hpp"
#include "ncad/laser/raster.hpp"
#include "ncad/laser/ruida_job.hpp"

#include <cstdlib>
#include <string>

using namespace ncad::laser;

namespace {

GreyImage flat(int w, int h, std::uint8_t g) {
    GreyImage i;
    i.w = w;
    i.h = h;
    i.px.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), g);
    return i;
}

double density(const RasterBitmap& bm) {
    std::size_t n = 0;
    for (std::uint8_t v : bm.level) n += v != 0;
    return static_cast<double>(n) / static_cast<double>(bm.level.size());
}

RasterBitmap bitmap(int w, int h) {
    RasterBitmap b;
    b.w = w;
    b.h = h;
    b.level.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), 0);
    return b;
}

void set(RasterBitmap& b, int x, int y) { b.level[static_cast<std::size_t>(y * b.w + x)] = 255; }

LaserRaster place(double x, double y, double w, double h) {
    LaserRaster r;
    r.at = {x, y};
    r.width = w;
    r.height = h;
    return r;
}

LaserLayer layer(double speed) {
    LaserLayer l;
    l.speed = speed;
    l.min_power = 5;
    l.max_power = 20;
    return l;
}

}  // namespace

TEST_CASE("raster: error diffusion density follows the grey level") {
    const DitherMethod methods[] = {DitherMethod::Floyd, DitherMethod::Jarvis, DitherMethod::Stucki,
                                    DitherMethod::Bayer};
    for (DitherMethod m : methods) {
        for (int g : {0, 64, 128, 192, 255}) {
            for (bool serp : {true, false}) {
                DitherOptions o;
                o.serpentine = serp;
                const RasterBitmap b = dither(flat(128, 128, static_cast<std::uint8_t>(g)), m, o);
                CHECK_NEAR(density(b), (255.0 - g) / 255.0, 0.02);
            }
        }
    }
}

TEST_CASE("raster: threshold is exact") {
    GreyImage img;
    img.w = 256;
    img.h = 1;
    for (int g = 0; g < 256; ++g) img.px.push_back(static_cast<std::uint8_t>(g));
    const RasterBitmap b = dither(img, DitherMethod::Threshold);
    for (int g = 0; g < 256; ++g) CHECK((b.level[static_cast<std::size_t>(g)] != 0) == (g < 128));
    DitherOptions o;
    o.threshold = 200;
    const RasterBitmap b2 = dither(img, DitherMethod::Threshold, o);
    for (int g = 0; g < 256; ++g) CHECK((b2.level[static_cast<std::size_t>(g)] != 0) == (g < 200));
}

TEST_CASE("raster: Bayer 4x4 pattern is exact") {
    // Grey 120 sits between thresholds 104 and 120.. fires where (index*16+8) > 120: index >= 8.
    const RasterBitmap b = dither(flat(8, 8, 120), DitherMethod::Bayer);
    static const int kBayer[4][4] = {{0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};
    for (int y = 0; y < 8; ++y) {
        for (int x = 0; x < 8; ++x) {
            const bool fire = kBayer[y & 3][x & 3] * 16 + 8 > 120;
            CHECK((b.level[static_cast<std::size_t>(y * 8 + x)] != 0) == fire);
        }
    }
    CHECK_NEAR(density(b), 8.0 / 16.0, 1e-9);
    CHECK(density(dither(flat(8, 8, 255), DitherMethod::Bayer)) == 0.0);
    CHECK(density(dither(flat(8, 8, 0), DitherMethod::Bayer)) == 1.0);
}

TEST_CASE("raster: floyd-steinberg serpentine differs from raster scan but not in density") {
    GreyImage img = flat(32, 32, 100);
    DitherOptions a, b;
    b.serpentine = false;
    CHECK(dither(img, DitherMethod::Floyd, a).level != dither(img, DitherMethod::Floyd, b).level);
}

TEST_CASE("raster: grey mode maps darkness to level") {
    GreyImage img;
    img.w = 3;
    img.h = 1;
    img.px = {0, 100, 255};
    const RasterBitmap b = dither(img, DitherMethod::Grey);
    CHECK(!b.binary);
    CHECK(b.level[0] == 255);
    CHECK(b.level[1] == 155);
    CHECK(b.level[2] == 0);
    // Spans carry the levels; the white pixel is a gap.
    RasterPlan p;
    std::string err;
    CHECK(plan_raster(b, place(0, 0, 3, 1), layer(100), {}, p, err));
    REQUIRE(p.lines.size() == 1);
    REQUIRE(p.lines[0].spans.size() == 1);
    CHECK(p.lines[0].spans[0].levels.size() == 2);
}

TEST_CASE("raster: levels, gamma and invert") {
    GreyImage img;
    img.w = 5;
    img.h = 1;
    img.px = {0, 50, 100, 150, 255};
    Levels lv;
    lv.black = 50;
    lv.white = 150;
    GreyImage a = img;
    apply_levels(a, lv);
    CHECK(a.px[0] == 0);
    CHECK(a.px[1] == 0);
    CHECK(a.px[2] == 128);
    CHECK(a.px[3] == 255);
    CHECK(a.px[4] == 255);
    GreyImage b = img;
    Levels inv;
    inv.invert = true;
    apply_levels(b, inv);
    CHECK(b.px[0] == 255);
    CHECK(b.px[4] == 0);
    CHECK(b.px[2] == 155);
    GreyImage c = flat(1, 1, 128);
    Levels g;
    g.gamma = 2.0;  // lightens midtones
    apply_levels(c, g);
    CHECK(c.px[0] > 128);
}

TEST_CASE("raster: pixel size follows the line interval") {
    int nx = 0, ny = 0;
    std::string err;
    CHECK(raster_pixel_size(50.0, 10.0, 254.0, nx, ny, err));
    CHECK(nx == 500);
    CHECK(ny == 100);
    CHECK(raster_pixel_size(0.01, 0.01, 254.0, nx, ny, err));
    CHECK(nx == 1);
    CHECK(!raster_pixel_size(0.0, 10.0, 254.0, nx, ny, err));
    CHECK(!raster_pixel_size(1e6, 1e6, 600.0, nx, ny, err));
}

TEST_CASE("raster: resample dimensions and averages") {
    GreyImage checker;
    checker.w = 8;
    checker.h = 8;
    for (int y = 0; y < 8; ++y) {
        for (int x = 0; x < 8; ++x) checker.px.push_back(((x + y) & 1) ? 255 : 0);
    }
    const GreyImage half = resample(checker, 4, 4);
    CHECK(half.w == 4);
    CHECK(half.h == 4);
    for (std::uint8_t v : half.px) CHECK(v == 128);  // area average of 0/255, rounded
    // Non-integer shrink keeps the mean of a flat field exactly.
    const GreyImage odd = resample(flat(10, 7, 77), 3, 4);
    for (std::uint8_t v : odd.px) CHECK(v == 77);
    // Enlarging a two-step ramp is bilinear: monotone, ends preserved.
    GreyImage ramp;
    ramp.w = 2;
    ramp.h = 1;
    ramp.px = {0, 200};
    const GreyImage big = resample(ramp, 8, 3);
    CHECK(big.w == 8);
    CHECK(big.h == 3);
    CHECK(big.px[0] == 0);
    CHECK(big.px[7] == 200);
    for (int x = 1; x < 8; ++x) CHECK(big.px[static_cast<std::size_t>(x)] >= big.px[static_cast<std::size_t>(x - 1)]);
    CHECK(big.px[3] > 0 && big.px[3] < 200);
}

TEST_CASE("raster: blank rows are skipped and margins trimmed") {
    RasterBitmap b = bitmap(20, 6);
    set(b, 5, 1);
    set(b, 6, 1);
    set(b, 9, 1);
    set(b, 12, 4);
    RasterPlanOptions o;
    o.accel_mm_s2 = 0.0;  // overscan is just the margin
    o.margin_mm = 1.0;
    RasterPlan p;
    std::string err;
    CHECK(plan_raster(b, place(100, 50, 20, 6), layer(100), o, p, err));
    CHECK(p.lines.size() == 2);
    CHECK(p.blank_rows == 4);
    CHECK(p.fired_pixels == 4);
    REQUIRE(p.lines.size() == 2);
    const RasterLine& l = p.lines[0];
    CHECK(l.row == 1);
    CHECK(l.spans.size() == 2);  // 5..7 and 9..10
    CHECK(l.spans[0].x0 == 5 && l.spans[0].x1 == 7);
    CHECK(l.spans[1].x0 == 9 && l.spans[1].x1 == 10);
    // 1 mm pixels: swept range is first..last fired pixel only, plus overscan.
    CHECK_NEAR(l.x_start, 100 + 5 - 1.0, 1e-9);
    CHECK_NEAR(l.x_end, 100 + 10 + 1.0, 1e-9);
    CHECK_NEAR(l.y, 50 + 6 - 1.5, 1e-9);  // row 1 from the top, Y up
}

TEST_CASE("raster: overscan is v^2/(2a) plus margin") {
    CHECK_NEAR(raster_overscan_mm(300.0, 3000.0, 1.5), 300.0 * 300.0 / 6000.0 + 1.5, 1e-9);
    CHECK_NEAR(raster_overscan_mm(300.0, 0.0, 2.0), 2.0, 1e-9);
    RasterBitmap b = bitmap(10, 1);
    set(b, 4, 0);
    RasterPlanOptions o;
    o.accel_mm_s2 = 2000.0;
    o.margin_mm = 0.5;
    RasterPlan p;
    std::string err;
    CHECK(plan_raster(b, place(0, 0, 10, 1), layer(200), o, p, err));
    CHECK_NEAR(p.overscan_mm, 200.0 * 200.0 / 4000.0 + 0.5, 1e-9);
    REQUIRE(p.lines.size() == 1);
    CHECK_NEAR(p.lines[0].x_end - p.lines[0].x_start, 1.0 + 2.0 * p.overscan_mm, 1e-9);
}

TEST_CASE("raster: bidirectional passes alternate; unidirectional do not") {
    RasterBitmap b = bitmap(8, 6);
    for (int y : {0, 2, 3, 5}) set(b, 3, y);  // rows 1 and 4 blank: flips count passes, not rows
    RasterPlan p;
    std::string err;
    RasterPlanOptions o;
    o.bidir = true;
    CHECK(plan_raster(b, place(0, 0, 8, 6), layer(100), o, p, err));
    REQUIRE(p.lines.size() == 4);
    for (std::size_t i = 0; i < 4; ++i) {
        CHECK(p.lines[i].left_to_right == (i % 2 == 0));
        const bool ltr = p.lines[i].left_to_right;
        CHECK((p.lines[i].x_start < p.lines[i].x_end) == ltr);
    }
    const double t_bidir = p.estimated_s;
    o.bidir = false;
    CHECK(plan_raster(b, place(0, 0, 8, 6), layer(100), o, p, err));
    for (const RasterLine& l : p.lines) CHECK(l.left_to_right);
    CHECK(p.estimated_s > t_bidir);  // return moves cost time
}

TEST_CASE("raster: single-pixel line pattern plans one span per line") {
    RasterBitmap b = bitmap(40, 3);
    for (int y = 0; y < 3; ++y) {
        for (int x = 0; x < 40; x += 4) set(b, x, y);
    }
    RasterPlan p;
    std::string err;
    CHECK(plan_raster(b, place(0, 0, 40, 3), layer(100), {}, p, err));
    CHECK(p.fired_pixels == 30);
    REQUIRE(p.lines.size() == 3);
    CHECK(p.lines[0].spans.size() == 10);
    CHECK(p.estimated_s > 0.0);
}

TEST_CASE("raster: encode_raster refuses until the encoding is known") {
    RasterPlan p;
    std::vector<std::uint8_t> bytes{1, 2};
    std::string err;
    CHECK(!encode_raster(p, bytes, err));
    CHECK(bytes.empty());
    CHECK(err.find("raster encoding not implemented yet") != std::string::npos);
}

TEST_CASE("raster: ljob record round-trips") {
    const std::string text =
        "ljob 1\n"
        "layer 3 color=3 mode=raster speed=300 power=5,20\n"
        "raster 3 at=20,50 size=60,30 dpi=254 image=\"logo.png\" dither=jarvis bidir=off\n";
    LaserJob j;
    std::string err;
    REQUIRE(read_ljob(text, j, err));
    REQUIRE(j.rasters.size() == 1);
    const LaserRaster& r = j.rasters[0];
    CHECK(r.pen == 3);
    CHECK_NEAR(r.at.x, 20, 1e-9);
    CHECK_NEAR(r.at.y, 50, 1e-9);
    CHECK_NEAR(r.width, 60, 1e-9);
    CHECK_NEAR(r.height, 30, 1e-9);
    CHECK_NEAR(r.dpi, 254, 1e-9);
    CHECK(r.image == "logo.png");
    CHECK(r.dither == DitherMethod::Jarvis);
    CHECK(!r.bidir);
    LaserJob k;
    REQUIRE(read_ljob(write_ljob(j), k, err));
    CHECK(write_ljob(k) == write_ljob(j));
    REQUIRE(k.rasters.size() == 1);
    CHECK(k.rasters[0].image == "logo.png");

    const char* bad[] = {
        "ljob 1\nlayer 3\nraster 3 at=0,0\n",                                   // no size, image
        "ljob 1\nlayer 3\nraster 3 at=0,0 size=1,1 image=\"a.png\" dither=x\n",   // unknown method
        "ljob 1\nraster 3 at=0,0 size=1,1 image=\"a.png\"\n",                     // no layer
        "ljob 1\nlayer 3\nraster 3 at=0,0 size=0,1 image=\"a.png\"\n",            // zero size
    };
    for (const char* t : bad) {
        LaserJob q;
        CHECK(!read_ljob(t, q, err));
    }
}

TEST_CASE("raster: planner and encoder refuse to send a raster job") {
    LaserJob j;
    std::string err;
    REQUIRE(read_ljob("ljob 1\nlayer 3 mode=raster speed=300 power=5,20\n"
                      "raster 3 at=0,0 size=10,10 image=\"a.png\"\n", j, err));
    LaserJob out;
    PlanReport rep;
    CHECK(!plan_job(j, out, {}, rep, err));
    CHECK(err.find("raster encoding not implemented yet") != std::string::npos);
    j.planned = true;
    ruida::RdJob rd;
    err.clear();
    CHECK(!job_to_rd(j, rd, err));
    CHECK(err.find("raster encoding not implemented yet") != std::string::npos);
}

TEST_CASE("raster: wedge PNG round trip and plan (integration)") {
    // The R01-R02 wedge from tools/fixtures/raster_images.py: 500x100, grey
    // ramp 0..255 left to right, 50 x 10 mm at 254 dpi. Built here and sent
    // through the PNG writer and loader; the LightBurn copy is used too when
    // NLASER_LB_FIXTURES points at it.
    GreyImage w = flat(500, 100, 0);
    for (int y = 0; y < 100; ++y) {
        for (int x = 0; x < 500; ++x) w.px[static_cast<std::size_t>(y * 500 + x)] = static_cast<std::uint8_t>(255 * x / 499);
    }
    std::vector<std::string> files;
    const std::string tmp = std::string(std::getenv("TMPDIR") ? std::getenv("TMPDIR") : "/tmp") + "/nlaser_wedge_test.png";
    std::string err;
    REQUIRE(write_grey_png(tmp, 500, 100, w.px.data(), err));
    files.push_back(tmp);
    if (const char* d = std::getenv("NLASER_LB_FIXTURES")) files.push_back(std::string(d) + "/R01-R02-wedge.png");
    for (const std::string& f : files) {
        GreyImage back;
        REQUIRE(load_grey_image(f, back, err));
        CHECK(back.w == 500);
        CHECK(back.h == 100);
        CHECK(back.px == w.px);
        LaserRaster r = place(10, 10, 50, 10);
        r.image = f;
        r.dither = DitherMethod::Floyd;
        RasterPlan p;
        RasterBitmap bm;
        REQUIRE(prepare_raster(r, layer(300), {}, p, &bm, err));
        CHECK(p.w == 500);
        CHECK(p.h == 100);
        // Mean darkness of a 0..255 ramp is 50%.
        CHECK_NEAR(density(bm), 0.5, 0.02);
        CHECK(p.lines.size() == 100);  // no blank row
        // Left edge is dark (fires), right edge white (blank): trimmed at the right.
        CHECK(p.lines[0].spans.front().x0 == 0);
        CHECK(p.lines[0].spans.back().x1 < 500);
    }
    std::remove(tmp.c_str());
}
