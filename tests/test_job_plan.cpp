// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// .ljob text, the planner, and the whole path from a raw job to .rd bytes.

#include "test.hpp"

#include "ncad/laser/job.hpp"
#include "ncad/laser/plan.hpp"
#include "ncad/laser/ruida_decode.hpp"
#include "ncad/laser/ruida_encode.hpp"
#include "ncad/laser/ruida_job.hpp"

#include <cmath>
#include <cstring>
#include <map>
#include <string>
#include <vector>

using namespace ncad::laser;

namespace {

LaserPath line(int pen, double x0, double y0, double x1, double y1) {
    LaserPath p;
    p.pen = pen;
    p.pts = {{x0, y0}, {x1, y1}};
    return p;
}

LaserPath rect(int pen, double x0, double y0, double x1, double y1) {
    LaserPath p;
    p.pen = pen;
    p.closed = true;
    p.pts = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
    return p;
}

LaserLayer layer(int pen, int order) {
    LaserLayer l;
    l.pen = pen;
    l.color = pen;
    l.speed = 20;
    l.min_power = 60;
    l.max_power = 65;
    l.order = order;
    return l;
}

// A tag plate as DXF from another program delivers it: the outline as four
// separate LINEs, drawn last, a closed hole, and a score line on another pen.
LaserJob tag_plate() {
    LaserJob j;
    j.name = "tag-plate";
    j.source = "tag.dxf";
    j.origin = {0, 0};
    j.layers = {layer(1, 2), layer(5, 1)};
    j.layers[1].speed = 150;
    j.layers[1].min_power = j.layers[1].max_power = 18;
    j.paths.push_back(line(1, 10, 10, 90, 10));
    j.paths.push_back(line(1, 90, 40, 10, 40));  // out of order and reversed
    j.paths.push_back(line(1, 90, 10, 90, 40));
    j.paths.push_back(line(1, 10, 40, 10, 10));
    j.paths.push_back(rect(1, 15, 20, 25, 30));  // the hole
    j.paths.push_back(line(5, 30, 25, 80, 25));  // score
    return j;
}

}  // namespace

// --- .ljob -------------------------------------------------------------------

TEST_CASE("ljob: writes the format the spec shows") {
    LaserJob j;
    j.name = "tag-plate";
    j.source = "tag.dxf";
    j.bed_w = 900;
    j.bed_h = 600;
    j.layers = {layer(5, 1)};
    j.layers[0].speed = 150;
    j.layers[0].min_power = j.layers[0].max_power = 18;
    j.paths.push_back(rect(5, 10, 10, 90, 40));
    CHECK(write_ljob(j) ==
          "ljob 1 name=\"tag-plate\" source=\"tag.dxf\"\n"
          "bed 900 600\n"
          "origin 0.000 0.000\n"
          "layer 5 color=5 mode=auto speed=150 power=18,18 passes=1 air=on order=1\n"
          "path 5 closed\n"
          "  10.000 10.000\n"
          "  90.000 10.000\n"
          "  90.000 40.000\n"
          "  10.000 40.000\n"
          "end\n");
}

TEST_CASE("ljob: round trip, including quotes, power2 and planned") {
    LaserJob j = tag_plate();
    j.planned = true;
    j.name = "a \"quoted\" name";
    j.layers[0].has_power2 = true;
    j.layers[0].min_power2 = 10;
    j.layers[0].max_power2 = 20;
    j.layers[1].air = false;
    j.layers[1].mode = LayerMode::Vector;
    j.origin = {-0.5, 32.3};
    const std::string text = write_ljob(j);
    LaserJob back;
    std::string error;
    REQUIRE(read_ljob(text, back, error));
    CHECK(write_ljob(back) == text);
    CHECK(back.planned);
    CHECK(back.name == "a \"quoted\" name");
    CHECK(back.layers[0].has_power2);
    CHECK(!back.layers[1].air);
}

TEST_CASE("ljob: start=current|absolute on the header line; current is the default and unwritten") {
    LaserJob j;
    std::string error;
    REQUIRE(read_ljob("ljob 1 planned start=absolute\nlayer 7\npath 7 open\n0 0\n1 1\nend\n", j, error));
    CHECK(j.start == StartMode::Absolute);
    const std::string text = write_ljob(j);
    CHECK(text.rfind("ljob 1 planned start=absolute\n", 0) == 0);
    LaserJob back;
    REQUIRE(read_ljob(text, back, error));
    CHECK(back.start == StartMode::Absolute);

    REQUIRE(read_ljob("ljob 1 start=current\nlayer 7\npath 7 open\n0 0\n1 1\nend\n", j, error));
    CHECK(j.start == StartMode::Current);
    CHECK(write_ljob(j).rfind("ljob 1\n", 0) == 0);
    CHECK(!read_ljob("ljob 1 start=nearby\n", back, error));
    CHECK(error.find("start") != std::string::npos);
}

TEST_CASE("ljob: comments and blank lines; origin defaults to 0,0") {
    LaserJob j;
    std::string error;
    REQUIRE(read_ljob("# a job\nljob 1\n\nlayer 7  # black\npath 7 open\n 0 0\n 1 1 # end\nend\n", j, error));
    CHECK(j.origin.x == 0.0 && j.origin.y == 0.0);
    REQUIRE(j.paths.size() == 1);
    CHECK(j.paths[0].pts.size() == 2);
    CHECK(j.layers[0].color == 7);  // colour defaults to the pen
}

TEST_CASE("ljob: errors name the line and leave the job untouched") {
    struct Bad { const char* text; const char* says; };
    const Bad bad[] = {
        {"", "no 'ljob'"},
        {"layer 1\n", "line 1"},
        {"ljob 2\n", "version 1"},
        {"ljob 1\nlayer 1 speed=fast\n", "line 2: bad value for 'speed'"},
        {"ljob 1\nlayer 1\nlayer 1\n", "defined twice"},
        {"ljob 1\nlayer 1\npath 1 open\n0 0\n", "has no 'end'"},
        {"ljob 1\nlayer 1\npath 1 open\n0 0\nend\n", "line 5: a path needs at least two points"},
        {"ljob 1\npath 3 open\n0 0\n1 1\nend\n", "pen 3"},
        {"ljob 1\nraster 3 at=0,0\n", "line 2: raster"},
        {"ljob 1 name=\"open\n", "unterminated"},
        {"ljob 1\nwidget 4\n", "unknown record"},
        {"ljob 1\nlayer 1 passes=0\n", "passes"},
    };
    for (const Bad& b : bad) {
        LaserJob j = tag_plate();
        std::string error;
        CHECK(!read_ljob(b.text, j, error));
        if (error.find(b.says) == std::string::npos) {
            ::nototest::fail(__FILE__, __LINE__, std::string("error '") + error + "' lacks '" + b.says + "'");
        }
        CHECK(j.name == "tag-plate");
    }
}

TEST_CASE("ljob: ACI colours for the panel") {
    CHECK(aci_rgb(1) == 0xFF0000);
    CHECK(aci_rgb(5) == 0x0000FF);
    CHECK(aci_rgb(7) == 0x000000);
    CHECK(aci_rgb(250) == 0x212121);
    CHECK(aci_rgb(10) == 0xFF0000);  // hue 0, full value and saturation
    CHECK(aci_rgb(-3) == aci_rgb(3));  // a layer that is off keeps its colour
}

// --- planner -----------------------------------------------------------------

TEST_CASE("plan: four separate lines join into one closed rectangle") {
    const LaserJob j = tag_plate();
    std::vector<LaserPath> pen1;
    for (const LaserPath& p : j.paths) {
        if (p.pen == 1 && !p.closed) pen1.push_back(p);
    }
    PlanReport r;
    const auto out = join_paths(pen1, 0.005, &r);
    REQUIRE(out.size() == 1);
    CHECK(out[0].closed);
    CHECK(out[0].pts.size() == 4);
    CHECK(r.joins == 3);
    CHECK(r.closed_by_join == 1);
}

TEST_CASE("plan: join respects the tolerance") {
    std::vector<LaserPath> v = {line(1, 0, 0, 10, 0), line(1, 10.004, 0, 20, 0), line(1, 20.006, 0, 30, 0)};
    const auto out = join_paths(v, 0.005, nullptr);
    CHECK(out.size() == 2);  // the 4 um gap joins, the 6 um gap does not
}

TEST_CASE("plan: containment finds the hole and the score inside the outline") {
    const std::vector<LaserPath> v = {rect(1, 0, 0, 100, 100), rect(1, 10, 10, 20, 20), line(1, 50, 50, 60, 50),
                                      rect(1, 200, 0, 210, 10)};
    const auto c = containment(v);
    REQUIRE(c.size() == 4);
    CHECK(c[0].size() == 2);
    CHECK(c[1].empty());
    CHECK(c[3].empty());
}

TEST_CASE("plan: inner before outer, even when the outer is nearer") {
    // The head starts at the outline's corner; the hole is far away inside it.
    const std::vector<LaserPath> v = {rect(1, 0, 0, 100, 100), rect(1, 80, 80, 90, 90)};
    Pt head{0, 0};
    const auto out = order_paths(v, head, PlanOptions{}, nullptr);
    REQUIRE(out.size() == 2);
    CHECK(out[0].pts[0].x >= 80.0);  // the hole first
    // Then the outline, from its corner nearest where the hole ended (80,80).
    CHECK(out[1].pts[0].x == 100.0 && out[1].pts[0].y == 100.0);
}

TEST_CASE("plan: nearest neighbour among free paths; open paths may reverse") {
    const std::vector<LaserPath> v = {line(1, 50, 0, 60, 0), line(1, 30, 0, 20, 0), line(1, 0, 5, 10, 5)};
    Pt head{0, 0};
    PlanReport r;
    const auto out = order_paths(v, head, PlanOptions{}, &r);
    REQUIRE(out.size() == 3);
    CHECK(out[0].pts[0].x == 0.0);
    CHECK(out[1].pts[0].x == 20.0);  // reversed to start at its near end
    CHECK(out[2].pts[0].x == 50.0);
    CHECK(head.x == 60.0);
    CHECK_NEAR(r.cut_mm, 30.0, 1e-9);
}

TEST_CASE("plan: a closed path starts at a corner rather than mid-run") {
    // A square whose bottom edge has an extra collinear vertex at (2,0). From
    // (1.5,-0.5) that mid-edge vertex is nearest (0.71 mm) and the corner at
    // the origin is 0.87 mm farther -- inside the 2 mm slack, so it wins.
    LaserPath sq;
    sq.pen = 1;
    sq.closed = true;
    sq.pts = {{0, 0}, {2, 0}, {10, 0}, {10, 10}, {0, 10}};
    Pt head{1.5, -0.5};
    auto out = order_paths({sq}, head, PlanOptions{}, nullptr);
    CHECK(out[0].pts[0].x == 0.0 && out[0].pts[0].y == 0.0);

    // With no slack the nearest vertex is taken, corner or not.
    PlanOptions strict;
    strict.corner_slack = 0.0;
    head = {1.5, -0.5};
    out = order_paths({sq}, head, strict, nullptr);
    CHECK(out[0].pts[0].x == 2.0);
}

TEST_CASE("plan: layers by order, skip dropped, raster refused") {
    LaserJob j = tag_plate();
    LaserJob planned;
    PlanReport r;
    std::string error;
    REQUIRE(plan_job(j, planned, PlanOptions{}, r, error));
    CHECK(planned.planned);
    REQUIRE(planned.paths.size() == 3);
    CHECK(planned.paths[0].pen == 5);  // order=1: the score goes first
    CHECK(planned.paths[1].pen == 1);
    // Then the hole, from its corner nearest the score's end at (80,25).
    CHECK(planned.paths[1].pts[0].x == 25.0 && planned.paths[1].pts[0].y == 20.0);
    CHECK(planned.paths[2].closed);           // then the joined outline
    CHECK(r.paths_in == 6);
    CHECK(r.paths_out == 3);

    j.layers[1].mode = LayerMode::Skip;
    REQUIRE(plan_job(j, planned, PlanOptions{}, r, error));
    CHECK(planned.paths.size() == 2);
    CHECK(r.skipped == 1);

    j.layers[1].mode = LayerMode::Raster;
    CHECK(!plan_job(j, planned, PlanOptions{}, r, error));
    CHECK(error.find("raster") != std::string::npos);
}

TEST_CASE("plan: the planned job survives its own text form") {
    LaserJob planned;
    PlanReport r;
    std::string error;
    REQUIRE(plan_job(tag_plate(), planned, PlanOptions{}, r, error));
    LaserJob back;
    REQUIRE(read_ljob(write_ljob(planned), back, error));
    CHECK(write_ljob(back) == write_ljob(planned));
    CHECK(back.planned);
}

// --- end to end --------------------------------------------------------------

TEST_CASE("end to end: raw tag plate -> plan -> .rd that decodes to the plan") {
    LaserJob planned;
    PlanReport r;
    std::string error;
    REQUIRE(plan_job(tag_plate(), planned, PlanOptions{}, r, error));

    ruida::RdJob rd;
    REQUIRE(job_to_rd(planned, rd, error));
    std::string plain;
    REQUIRE(encode_rd(rd, plain, error));

    const auto cmds = ruida::decode_rd(plain);
    const ruida::RdSummary s = ruida::summarize_rd(cmds);
    CHECK(s.unknown == 0);
    CHECK(s.bad_length == 0);
    CHECK(s.file_sum_stated == s.file_sum_computed);
    CHECK(s.has_eof);
    CHECK_NEAR(s.cut_mm, r.cut_mm, 1e-6);
    CHECK_NEAR(s.travel_mm, r.travel_mm, 1e-6);
    CHECK(s.cut_min_x == 10.0 && s.cut_max_x == 90.0);
    CHECK(s.origin_x == 0 && s.origin_y == 0);

    // Two layers, score first: its panel colour is ACI 5, blue (0xBBGGRR).
    std::size_t parts = 0;
    for (const auto& c : cmds) {
        if (c.spec && std::strcmp(c.spec->name, "Layer Color Part") == 0) {
            if (parts == 0) CHECK(c.fields[1].i == 0xFF0000);
            ++parts;
        }
    }
    CHECK(parts == 2);
}

TEST_CASE("end to end: an unplanned job is refused by the driver") {
    ruida::RdJob rd;
    std::string error;
    CHECK(!job_to_rd(tag_plate(), rd, error));
    CHECK(error.find("plan") != std::string::npos);
}

// --- time estimate and 2-opt ---------------------------------------------------

TEST_CASE("plan: segment time matches the trapezoid closed form") {
    // 1 mm at up to 400 mm/s, 3000 mm/s^2: needs 53.3 mm to reach speed, so it
    // is a triangle, t = 2 sqrt(L/a).
    CHECK_NEAR(segment_time_s(1.0, 400.0, 3000.0), 2.0 * std::sqrt(1.0 / 3000.0), 1e-12);
    // 100 mm at 20 mm/s: ramps take 0.133 mm, so it cruises, t = L/v + v/a.
    CHECK_NEAR(segment_time_s(100.0, 20.0, 3000.0), 100.0 / 20.0 + 20.0 / 3000.0, 1e-12);
    // Exactly on the boundary both forms agree.
    const double edge = 20.0 * 20.0 / 3000.0;
    CHECK_NEAR(segment_time_s(edge, 20.0, 3000.0), 2.0 * std::sqrt(edge / 3000.0), 1e-12);
    CHECK(segment_time_s(0.0, 20.0, 3000.0) == 0.0);
    CHECK_NEAR(segment_time_s(10.0, 20.0, 0.0), 0.5, 1e-12);  // no accel: constant speed
}

TEST_CASE("plan: time estimate for a 100 mm square at 20 mm/s") {
    LaserJob j;
    j.layers = {layer(1, 0)};
    j.paths.push_back(rect(1, 0, 0, 100, 100));
    LaserJob planned;
    PlanReport r;
    std::string error;
    REQUIRE(plan_job(j, planned, PlanOptions{}, r, error));
    // Four stop-to-stop sides; the head starts on the corner, so no travel.
    CHECK_NEAR(r.cut_s, 4.0 * (100.0 / 20.0 + 20.0 / 3000.0), 1e-9);
    CHECK_NEAR(r.travel_s, 0.0, 1e-12);
    CHECK_NEAR(r.estimated_s, r.cut_s + r.travel_s, 1e-12);

    // Moved 300 mm away: the travel leg runs at travel speed.
    j.paths[0] = rect(1, 300, 0, 400, 100);
    REQUIRE(plan_job(j, planned, PlanOptions{}, r, error));
    CHECK_NEAR(r.travel_s, 300.0 / 400.0 + 400.0 / 3000.0, 1e-9);
}

namespace {

// A path that is a point, so direction cannot matter and the layout is a
// plain travelling-salesman instance.
LaserPath dot(double x, double y) { return line(1, x, y, x, y); }

}  // namespace

TEST_CASE("plan: 2-opt repairs a nearest-neighbour order that is worse") {
    // Found by search: from the origin nearest-neighbour visits 1,3,2,0,4 for
    // 149.2 mm; the optimum, 1,3,4,2,0, is 134.2 mm.
    const std::vector<LaserPath> v = {dot(55, 55), dot(30, 5), dot(40, 50), dot(45, 5), dot(15, 40)};
    PlanOptions off;
    off.two_opt = false;
    Pt h1{0, 0}, h2{0, 0};
    PlanReport r_off, r_on;
    const auto a = order_paths(v, h1, off, &r_off);
    const auto b = order_paths(v, h2, PlanOptions{}, &r_on);
    REQUIRE(a.size() == 5);
    REQUIRE(b.size() == 5);
    CHECK_NEAR(r_off.travel_mm, 149.2221453, 1e-6);
    CHECK_NEAR(r_on.travel_mm, 134.2487473, 1e-6);
    CHECK(b[2].pts[0].x == 15.0);  // 4 comes before 2
    CHECK(b[4].pts[0].x == 55.0);
}

TEST_CASE("plan: 2-opt never takes a move that puts a path after its container") {
    const std::vector<LaserPath> v = {dot(55, 55), dot(30, 5), dot(40, 50), dot(45, 5), dot(15, 40)};
    const std::vector<std::size_t> nn = {1, 3, 2, 0, 4};
    const Pt head{0, 0};
    const PlanOptions opts;
    // Free of constraints, 2-opt moves 4 ahead of 2 and gains.
    const auto unconstrained = two_opt_order(v, nn, head, opts, {});
    CHECK(unconstrained == (std::vector<std::size_t>{1, 3, 4, 2, 0}));
    CHECK(order_travel(v, unconstrained, head, opts) < order_travel(v, nn, head, opts));
    // Say 4 contains 2: 2 must stay first, so that move is refused. Whatever
    // it does take keeps 2 before 4 and never costs more.
    std::vector<std::vector<std::size_t>> contains(5);
    contains[4] = {2};
    const auto held = two_opt_order(v, nn, head, opts, contains);
    std::size_t pos2 = 0, pos4 = 0;
    for (std::size_t k = 0; k < held.size(); ++k) {
        if (held[k] == 2) pos2 = k;
        if (held[k] == 4) pos4 = k;
    }
    CHECK(pos2 < pos4);
    CHECK(order_travel(v, held, head, opts) <= order_travel(v, nn, head, opts));
}

TEST_CASE("plan: 2-opt on real nested paths keeps holes before the outline") {
    const std::vector<LaserPath> v = {rect(1, 0, 0, 100, 100), rect(1, 60, 60, 70, 70), rect(1, 20, 20, 30, 30),
                                      rect(1, 40, 80, 50, 90)};
    Pt head{0, 0};
    const auto out = order_paths(v, head, PlanOptions{}, nullptr);
    REQUIRE(out.size() == 4);
    // Every hole starts strictly inside the outline; the outline is last.
    for (std::size_t k = 0; k < 3; ++k) {
        CHECK(out[k].pts[0].x > 0.0);
        CHECK(out[k].pts[0].x < 100.0);
    }
}

TEST_CASE("plan: 2-opt leaves a layer over the path cap alone") {
    std::vector<LaserPath> v;
    for (int i = 0; i < 250; ++i) v.push_back(dot(static_cast<double>((i * 37) % 101), static_cast<double>(i)));
    Pt h1{0, 0}, h2{0, 0};
    PlanOptions off;
    off.two_opt = false;
    PlanReport a, b;
    order_paths(v, h1, off, &a);
    order_paths(v, h2, PlanOptions{}, &b);
    CHECK_NEAR(a.travel_mm, b.travel_mm, 1e-9);
}

// --- regions, cut-free avoidance, overrides ------------------------------------

namespace {

LaserPath with_id(LaserPath p, int id) {
    p.id = id;
    return p;
}

std::vector<int> ids_of(const std::vector<LaserPath>& v) {
    std::vector<int> out;
    for (const LaserPath& p : v) out.push_back(p.id);
    return out;
}

std::size_t position_of(const std::vector<LaserPath>& v, int id) {
    for (std::size_t i = 0; i < v.size(); ++i) {
        if (v[i].id == id) return i;
    }
    return v.size();
}

}  // namespace

TEST_CASE("plan: a region is finished before the next is begun") {
    // A big outline with a hole near the head and one far from it, and a lone
    // part off to the side that sits closer to the far hole than the near hole
    // is: a single pool would hop out to it and come back.
    const std::vector<LaserPath> v = {with_id(rect(1, 0, 0, 100, 100), 0), with_id(rect(1, 5, 5, 10, 10), 1),
                                      with_id(rect(1, 90, 90, 95, 95), 2), with_id(rect(1, 110, 40, 120, 50), 3)};
    PlanOptions pool;
    pool.by_region = false;
    pool.two_opt = false;
    pool.cutfree_weight = 0.0;
    Pt h1{0, 0};
    PlanReport pr;
    const auto pooled = order_paths(v, h1, pool, &pr);
    CHECK(position_of(pooled, 3) < position_of(pooled, 2));  // the outline's hole waits while the lone part is cut

    PlanOptions regions = pool;
    regions.by_region = true;
    Pt h2{0, 0};
    PlanReport rr;
    const auto grouped = order_paths(v, h2, regions, &rr);
    CHECK(rr.regions == 2);
    // Holes, then the outline, then the lone part.
    CHECK(ids_of(grouped) == std::vector<int>({1, 2, 0, 3}));

    // 2-opt reorders whole regions, never the inside of one.
    regions.two_opt = true;
    Pt h3{0, 0};
    const auto opt = order_paths(v, h3, regions, nullptr);
    const std::size_t a = position_of(opt, 1), b = position_of(opt, 2), c = position_of(opt, 0);
    CHECK(c == a + 2);
    CHECK(b == a + 1);
}

TEST_CASE("plan: travel across a part already cut free is avoided, not routed around") {
    // After the big part is cut (starting and ending at the head), one open
    // line lies across it and one lies clear of it, a little farther away.
    const std::vector<LaserPath> v = {with_id(rect(1, 0, 0, 100, 100), 0),
                                      with_id(line(1, 105, 50, 110, 50), 1),
                                      with_id(line(1, -120, 0, -125, 0), 2)};
    PlanOptions plain;
    plain.cutfree_weight = 0.0;
    Pt h1{0, 0};
    PlanReport pr;
    const auto nn = order_paths(v, h1, plain, &pr);
    CHECK(ids_of(nn) == std::vector<int>({0, 1, 2}));  // nearest first, crossing the part
    CHECK(pr.crossing_mm > 150.0);

    Pt h2{0, 0};
    PlanReport ar;
    const auto avoid = order_paths(v, h2, PlanOptions{}, &ar);
    // Cutting the part last leaves nothing free to cross: the order changed,
    // and no path was routed anywhere it was not going.
    CHECK(ids_of(avoid) == std::vector<int>({2, 1, 0}));
    CHECK(ar.crossing_mm < 1e-9);
    CHECK(ar.travel_mm > pr.travel_mm);  // a longer way was accepted for it
}

TEST_CASE("plan: a hole is not a part, so crossing it costs nothing") {
    PlanReport r;
    Pt head{0, 0};
    const std::vector<LaserPath> v = {with_id(rect(1, 0, 0, 100, 100), 0), with_id(rect(1, 40, 40, 60, 60), 1),
                                      with_id(line(1, 200, 50, 210, 50), 2)};
    order_paths(v, head, PlanOptions{}, &r);
    // Only the outline is a part; the hole's travel to the outline's start is
    // along the outline's own inside, which counts, but the hole itself adds none.
    PlanOptions off;
    off.cutfree_weight = 0.0;
    Pt h2{0, 0};
    PlanReport r0;
    order_paths(v, h2, off, &r0);
    CHECK(r.crossing_mm <= r0.crossing_mm + 1e-9);
}

namespace {

LaserJob lines_job() {
    LaserJob j;
    j.layers = {layer(1, 0)};
    for (int i = 0; i < 6; ++i) j.paths.push_back(with_id(line(1, 10.0 * (i + 1), 0, 10.0 * (i + 1), 1), i));
    return j;
}

}  // namespace

TEST_CASE("plan: fixed paths keep their order and the rest is optimised around them") {
    const LaserJob raw = lines_job();
    LaserJob out;
    PlanReport rep;
    std::string err;
    PlanOptions opts;
    CHECK(plan_job(raw, out, opts, rep, err));
    CHECK(ids_of(out.paths) == std::vector<int>({0, 1, 2, 3, 4, 5}));

    opts.overrides.steps = {{5, {}, false, false}, {0, {}, false, false}};
    LaserJob fixed;
    CHECK(plan_job(raw, fixed, opts, rep, err));
    CHECK(ids_of(fixed.paths) == std::vector<int>({1, 2, 3, 4, 5, 0}));
    CHECK(rep.warnings.empty());

    // An override that names nothing in the job is said so, and ignored.
    opts.overrides.steps = {{42, {}, false, false}};
    CHECK(plan_job(raw, fixed, opts, rep, err));
    CHECK(rep.warnings.size() == 1);
    CHECK(ids_of(fixed.paths) == std::vector<int>({0, 1, 2, 3, 4, 5}));
}

TEST_CASE("plan: a fixed start and direction are honoured, with the order left free") {
    LaserJob raw;
    raw.layers = {layer(1, 0)};
    raw.paths.push_back(with_id(rect(1, 0, 0, 10, 10), 7));
    LaserJob out;
    PlanReport rep;
    std::string err;
    PlanOptions opts;
    FixedStep s;
    s.id = 7;
    s.start = 2;
    s.reverse = true;
    s.free_order = true;
    opts.overrides.steps = {s};
    CHECK(plan_job(raw, out, opts, rep, err));
    REQUIRE(out.paths.size() == 1);
    const auto& p = out.paths[0].pts;
    CHECK(p[0].x == 10.0);
    CHECK(p[0].y == 10.0);
    CHECK(p[1].x == 10.0);  // vertex 1, not vertex 3: the other way round
    CHECK(p[1].y == 0.0);
}

TEST_CASE("plan: a fixed order that cuts an outline before what is inside it is honoured, with a warning") {
    LaserJob raw;
    raw.layers = {layer(1, 0)};
    raw.paths.push_back(with_id(rect(1, 0, 0, 100, 100), 0));
    raw.paths.push_back(with_id(rect(1, 40, 40, 60, 60), 1));
    LaserJob out;
    PlanReport rep;
    std::string err;
    PlanOptions opts;
    CHECK(plan_job(raw, out, opts, rep, err));
    CHECK(ids_of(out.paths) == std::vector<int>({1, 0}));
    CHECK(rep.warnings.empty());

    opts.overrides.steps = {{0, {}, false, false}, {1, {}, false, false}};
    CHECK(plan_job(raw, out, opts, rep, err));
    CHECK(ids_of(out.paths) == std::vector<int>({0, 1}));
    REQUIRE(rep.warnings.size() == 1);
    CHECK(rep.warnings[0].find("path 0 is cut before path 1") != std::string::npos);

    // Fixing only the outline still puts the free hole before it, silently.
    opts.overrides.steps = {{0, {}, false, false}};
    CHECK(plan_job(raw, out, opts, rep, err));
    CHECK(ids_of(out.paths) == std::vector<int>({1, 0}));
    CHECK(rep.warnings.empty());
}

TEST_CASE("plan: joining keeps the first member's id, and overrides follow a chain") {
    std::vector<LaserPath> v = {with_id(line(1, 0, 0, 10, 0), 4), with_id(line(1, 20, 0, 10, 0), 9)};
    std::map<int, int> alias;
    const auto j = join_paths(v, 0.005, nullptr, &alias);
    REQUIRE(j.size() == 1);
    CHECK(j[0].id == 4);
    CHECK(alias.at(9) == 4);

    LaserJob raw;
    raw.layers = {layer(1, 0)};
    raw.paths = v;
    raw.paths.push_back(with_id(line(1, 50, 0, 60, 0), 5));
    LaserJob out;
    PlanReport rep;
    std::string err;
    PlanOptions opts;
    opts.overrides.steps = {{5, {}, false, false}, {9, {}, false, false}};  // 9 is part of chain 4
    CHECK(plan_job(raw, out, opts, rep, err));
    CHECK(ids_of(out.paths) == std::vector<int>({5, 4}));
}

TEST_CASE("ljob: path ids round trip, and files without ids stay valid") {
    LaserJob j;
    j.layers = {layer(1, 0)};
    j.paths.push_back(with_id(rect(1, 0, 0, 5, 5), 3));
    j.paths.push_back(line(1, 0, 0, 1, 1));  // no id
    const std::string text = write_ljob(j);
    CHECK(text.find("path 1 closed id=3\n") != std::string::npos);
    CHECK(text.find("path 1 open\n") != std::string::npos);

    LaserJob back;
    std::string err;
    REQUIRE(read_ljob(text, back, err));
    REQUIRE(back.paths.size() == 2);
    CHECK(back.paths[0].id == 3);
    CHECK(back.paths[1].id == -1);
    CHECK(write_ljob(back) == text);

    CHECK(!read_ljob("ljob 1\nlayer 1\npath 1 open id=x\n 0 0\n 1 1\nend\n", back, err));
    CHECK(!read_ljob("ljob 1\nlayer 1\npath 1 open nope=1\n 0 0\n 1 1\nend\n", back, err));
}
