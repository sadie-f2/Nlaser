// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// Findings from the LightBurn fixtures in tests/fixtures/lightburn (see
// notes.txt there and docs/lightburn-fixtures.md): how passes, origins and
// circles come out. These pin what LightBurn wrote; they do not need our
// encoder.

#include "test.hpp"

#include "ncad/laser/ruida_codec.hpp"
#include "ncad/laser/ruida_decode.hpp"

#include <cmath>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

using namespace ncad::laser::ruida;

namespace {

struct Fixture {
    std::string plain;
    std::vector<RdCommand> cmds;
    RdSummary s;
    bool ok{false};
};

// `plain` must outlive `cmds`, so the fixture is filled in place.
void load(const char* name, Fixture& f) {
    const std::string path = std::string(NLASER_FIXTURES) + "/lightburn/" + name;
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        ::nototest::fail(__FILE__, __LINE__, std::string("fixture missing: ") + name);
        return;
    }
    const std::string file((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    f.plain = Swizzle(0x88).decode(file);
    f.cmds = decode_rd(f.plain);
    f.s = summarize_rd(f.cmds);
    f.ok = true;
}

std::size_t count_named(const Fixture& f, const char* name) {
    std::size_t n = 0;
    for (const RdCommand& c : f.cmds) {
        if (c.spec && std::strcmp(c.spec->name, name) == 0) ++n;
    }
    return n;
}

// The cut polylines, in mm, one per Move.
std::vector<std::vector<std::pair<double, double>>> polylines(const Fixture& f) {
    std::vector<std::vector<std::pair<double, double>>> out;
    std::int64_t x = 0, y = 0;
    for (const RdCommand& c : f.cmds) {
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
        const std::pair<double, double> p{static_cast<double>(x) / 1000.0, static_cast<double>(y) / 1000.0};
        if (m == RdMotion::MoveAbs || m == RdMotion::MoveRel) {
            out.push_back({p});
        } else if (!out.empty()) {
            out.back().push_back(p);
        }
    }
    return out;
}

}  // namespace

TEST_CASE("lightburn fixtures: every file decodes clean with a matching file sum") {
    for (const char* name : {"f01.rd", "f02-p2.rd", "f02-p3.rd", "f05.rd", "f06-BR.rd", "f07.rd"}) {
        Fixture f;
        load(name, f);
        REQUIRE(f.ok);
        CHECK(f.s.unknown == 0);
        CHECK(f.s.bad_length == 0);
        CHECK(f.s.stray == 0);
        CHECK(f.s.has_eof);
        CHECK(f.s.file_sum_stated == f.s.file_sum_computed);
        // Every sample starts from the current position (Ref Point 0).
        CHECK(f.s.origin_x == 5200);
        CHECK(f.s.origin_y == 31200);
    }
}

TEST_CASE("lightburn fixtures: passes repeat the whole layer body, and the cut-length counter follows") {
    // 40 mm square = 4 cuts of 40 mm. Each extra pass repeats the block from
    // "Work Mode 6" to the last cut; the Move is written once.
    struct Want { const char* name; std::size_t passes; };
    for (const Want w : {Want{"f01.rd", 1}, Want{"f02-p2.rd", 2}, Want{"f02-p3.rd", 3}}) {
        Fixture f;
        load(w.name, f);
        REQUIRE(f.ok);
        CHECK(count_named(f, "Work Mode 6") == w.passes);
        CHECK(count_named(f, "Layer End") == 1 + 2 * w.passes);
        CHECK(f.s.moves == 1);
        CHECK(f.s.cuts == 4 * w.passes);
        CHECK_NEAR(f.s.cut_mm, 160.0 * static_cast<double>(w.passes), 1e-6);
        for (const RdCommand& c : f.cmds) {
            if (c.spec && std::strcmp(c.spec->name, "Set Setting / Reply mem 06 20") == 0) {
                CHECK(c.fields[0].i == static_cast<std::int64_t>(160 * w.passes));
            }
        }
    }
}

TEST_CASE("lightburn fixtures: f05, f06-BR and f07 were saved with Passes still at 2") {
    Fixture a, b, c;
    load("f05.rd", a);
    load("f06-BR.rd", b);
    load("f07.rd", c);
    REQUIRE(a.ok);
    REQUIRE(b.ok);
    REQUIRE(c.ok);
    CHECK(a.s.cuts == 8);
    CHECK(b.s.cuts == 8);
    CHECK(count_named(a, "Work Mode 6") == 2);
    CHECK(count_named(b, "Work Mode 6") == 2);
    CHECK(c.s.cuts == 2 * 480);
}

TEST_CASE("lightburn fixtures: origin = Document Min Point - Process Top Left holds for centre and corner origins") {
    Fixture f01, f05, f06;
    load("f01.rd", f01);
    load("f05.rd", f05);
    load("f06-BR.rd", f06);
    REQUIRE(f01.ok);
    REQUIRE(f05.ok);
    REQUIRE(f06.ok);

    // Centre dot: the box is symmetric about the origin and no corner fits.
    CHECK(f05.s.box_min_x == -20000);
    CHECK(f05.s.box_min_y == -20000);
    CHECK(f05.s.box_max_x == 20000);
    CHECK(f05.s.box_max_y == 20000);
    CHECK(std::strcmp(f05.s.origin_corner, "other") == 0);

    // These are top-right-origin machines, so file axes are the design's
    // negated. Raw (file-frame) naming is 180 degrees from LightBurn's label:
    // f01 (LightBurn top-left) has box x -W..0, y 0..H, raw "bottom-right";
    // f06-BR has box x 0..W, y -H..0, raw "top-left". Given the machine,
    // the summary names them as LightBurn does.
    const RdMachine tr{MachineCorner::TopRight, false, false};
    CHECK(std::strcmp(f01.s.origin_corner_file, "bottom-right") == 0);
    CHECK(std::strcmp(f06.s.origin_corner_file, "top-left") == 0);
    CHECK(f01.s.box_min_x == -40000);
    CHECK(f01.s.box_max_y == 40000);
    CHECK(f06.s.box_min_y == -40000);
    CHECK(f06.s.box_max_x == 40000);
    CHECK(std::strcmp(summarize_rd(f01.cmds, tr).origin_corner, "top-left") == 0);
    CHECK(std::strcmp(summarize_rd(f06.cmds, tr).origin_corner, "bottom-right") == 0);
    CHECK(std::strcmp(summarize_rd(f05.cmds, tr).origin_corner, "other") == 0);
}

TEST_CASE("lightburn fixtures: f08, f09, f10 path structure (what LightBurn did to curves, small segments and nesting)") {
    Fixture f8, f9, f10;
    load("f08.rd", f8);
    load("f09.rd", f9);
    load("f10.rd", f10);
    REQUIRE(f8.ok);
    REQUIRE(f9.ok);
    REQUIRE(f10.ok);
    auto sizes = [](const Fixture& f) {
        std::vector<std::size_t> n;
        for (const auto& pl : polylines(f)) n.push_back(pl.size() - 1);
        return n;
    };
    // f08: the cubic S-curve in 81 chords, then the semicircle (r 15) in 64,
    // i.e. half of the 128 a full circle of that radius would get.
    CHECK((sizes(f8) == std::vector<std::size_t>{81, 64}));
    // f09: the 61-chord run is the 200-step polyline of 0.05 mm steps, which
    // LightBurn thinned; every Move is absolute, the 8.000 mm one included.
    CHECK((sizes(f9) == std::vector<std::size_t>{1, 1, 1, 1, 61, 1, 1}));
    // f10: inner first. The 5 mm circle (48 chords), the 20 mm hole, the 60 mm
    // square; each closed path returns to where it started.
    CHECK((sizes(f10) == std::vector<std::size_t>{48, 4, 4}));
    for (const auto& pl : polylines(f10)) CHECK(pl.front() == pl.back());
}

TEST_CASE("lightburn fixtures: f07 circles are power-of-two polygons, traced once per pass") {
    Fixture f;
    load("f07.rd", f);
    REQUIRE(f.ok);
    const auto lines = polylines(f);
    REQUIRE(lines.size() == 4);

    // DXF diameter and centre x; the centre y is 100. The origin is design (0,200), the
    // top-left, at the head (5.2,31.2): file x = 5.2 - dxf x, file y = 231.2 - dxf y.
    struct Circle { double radius, cx; std::size_t per_pass; };
    const Circle want[4] = {{25.0, 57.0, 128}, {5.0, 17.0, 64}, {1.0, 1.0, 32}, {100.0, 192.0, 256}};
    // The order LightBurn wrote them is the file order, not the DXF order.
    for (const auto& line : lines) {
        const Circle* match = nullptr;
        for (const Circle& c : want) {
            if (line.size() == 2 * c.per_pass + 1) match = &c;
        }
        REQUIRE(match != nullptr);
        const double cx = 5.2 - match->cx, cy = 231.2 - 100.0;
        // Two passes: the polyline returns to its start exactly halfway.
        CHECK(line[match->per_pass] == line.front());
        CHECK(line.back() == line.front());

        const auto n = static_cast<double>(match->per_pass);
        double max_sag = 0.0, max_r_err = 0.0;
        for (std::size_t i = 0; i < match->per_pass; ++i) {
            const double mx = (line[i].first + line[i + 1].first) / 2.0 - cx;
            const double my = (line[i].second + line[i + 1].second) / 2.0 - cy;
            max_sag = std::fmax(max_sag, match->radius - std::hypot(mx, my));
            max_r_err = std::fmax(max_r_err, std::fabs(std::hypot(line[i].first - cx, line[i].second - cy) - match->radius));
        }
        // Vertices lie on the circle (to about 30 um, the file's 1 um grid
        // plus LightBurn's own rounding); the deepest chord sits under 8.5 um.
        CHECK(max_r_err < 0.03);
        CHECK(max_sag < 0.0085);
        // And n is the smallest power of two whose ideal sagitta is within
        // 0.0192 mm (the next size down for the 2 mm circle), so the
        // tolerance is between 0.0075 and 0.0192 mm, probably 0.01.
        const double ideal = match->radius * (1.0 - std::cos(3.14159265358979323846 / n));
        CHECK(ideal < 0.0076);
        CHECK(match->radius * (1.0 - std::cos(3.14159265358979323846 / (n / 2.0))) > 0.0192 - 1e-4);
    }
}
