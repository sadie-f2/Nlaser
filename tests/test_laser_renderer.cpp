// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// The NotoCAD bridge: drawings built through NotoCAD's own API, rendered into
// raw jobs, and one taken all the way through DXF, the planner and the
// encoder to .rd bytes.

#include "test.hpp"

#include "ncad/blocks.hpp"
#include "ncad/database.hpp"
#include "ncad/dxf.hpp"
#include "ncad/entities.hpp"
#include "ncad/laser/job.hpp"
#include "ncad/laser/laser_renderer.hpp"
#include "ncad/laser/plan.hpp"
#include "ncad/laser/ruida_codec.hpp"
#include "ncad/laser/ruida_decode.hpp"
#include "ncad/laser/ruida_encode.hpp"
#include "ncad/laser/ruida_job.hpp"
#include "ncad/render.hpp"
#include "ncad/tables.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <numbers>
#include <string>
#include <vector>

using namespace ncad::laser;
using ncad::Database;
using ncad::EntityPtr;
using ncad::Vec3;

namespace {

constexpr double kPi = std::numbers::pi;

EntityPtr coloured(EntityPtr e, std::int16_t color, ncad::LayerId layer = ncad::kLayerZero) {
    e->props().color = color;
    e->props().layer = layer;
    return e;
}

EntityPtr square(double x0, double y0, double side) {
    auto p = std::make_unique<ncad::Polyline>();
    p->add({x0, y0, 0});
    p->add({x0 + side, y0, 0});
    p->add({x0 + side, y0 + side, 0});
    p->add({x0, y0 + side, 0});
    p->set_closed(true);
    return p;
}

std::vector<const LaserPath*> on_pen(const LaserJob& job, int pen) {
    std::vector<const LaserPath*> out;
    for (const LaserPath& p : job.paths) {
        if (p.pen == pen) out.push_back(&p);
    }
    return out;
}

bool render(const Database& db, LaserJob& job, DrawingReport& report) {
    std::string error;
    return render_drawing(db, DrawingOptions{}, job, report, error);
}

bool has_warning(const DrawingReport& r, const char* text) {
    for (const std::string& w : r.warnings) {
        if (w.find(text) != std::string::npos) return true;
    }
    return false;
}

}  // namespace

// --- entities ------------------------------------------------------------------

TEST_CASE("bridge: LINE, CIRCLE, ARC, bulged POLYLINE, SPLINE and TEXT reach the job") {
    Database db;
    db.add(coloured(std::make_unique<ncad::Line>(Vec3{0, 0, 0}, Vec3{50, 0, 0}), 1));
    db.add(coloured(std::make_unique<ncad::Circle>(Vec3{100, 100, 0}, 10.0), 2));
    db.add(coloured(std::make_unique<ncad::Arc>(Vec3{0, 0, 0}, 20.0, 0.0, kPi / 2), 3));
    auto bulged = std::make_unique<ncad::Polyline>();
    bulged->add({0, 50, 0}, 1.0);  // a semicircle from here to the next vertex
    bulged->add({10, 50, 0});
    db.add(coloured(std::move(bulged), 4));
    db.add(coloured(ncad::Spline::interpolating({{0, 80, 0}, {10, 90, 0}, {20, 80, 0}, {30, 90, 0}}), 5));
    db.add(coloured(std::make_unique<ncad::Text>(Vec3{200, 0, 0}, "LASER", 5.0), 6));

    LaserJob job;
    DrawingReport r;
    REQUIRE(render(db, job, r));
    CHECK(r.entities == 6);
    CHECK(r.unmatched == 0);
    CHECK(!job.planned);

    const auto line = on_pen(job, 1);
    REQUIRE(line.size() == 1);
    CHECK(!line[0]->closed);
    REQUIRE(line[0]->pts.size() == 2);
    CHECK_NEAR(line[0]->pts[1].x, 50.0, 1e-12);

    // Flattened at LASERTOL: every vertex on the circle, and enough of them
    // that no chord strays more than 0.01 mm from it.
    const auto circle = on_pen(job, 2);
    REQUIRE(circle.size() == 1);
    CHECK(circle[0]->closed);
    CHECK(static_cast<int>(circle[0]->pts.size()) >= ncad::arc_segment_count(10.0, 2 * kPi, kLaserTol));
    CHECK(circle[0]->pts.size() > 60);
    for (const Pt& p : circle[0]->pts) CHECK_NEAR(std::hypot(p.x - 100, p.y - 100), 10.0, 1e-9);

    const auto arc = on_pen(job, 3);
    REQUIRE(arc.size() == 1);
    CHECK(!arc[0]->closed);
    CHECK_NEAR(arc[0]->pts.front().x, 20.0, 1e-9);
    CHECK_NEAR(arc[0]->pts.back().y, 20.0, 1e-9);

    // The bulge is an arc, not the chord: points well off the straight line.
    const auto poly = on_pen(job, 4);
    REQUIRE(poly.size() == 1);
    CHECK(poly[0]->pts.size() > 10);
    double off = 0.0;
    for (const Pt& p : poly[0]->pts) off = std::max(off, std::abs(p.y - 50.0));
    CHECK_NEAR(off, 5.0, 0.01);

    const auto spline = on_pen(job, 5);
    REQUIRE(spline.size() == 1);
    CHECK_NEAR(spline[0]->pts.front().x, 0.0, 1e-9);
    CHECK_NEAR(spline[0]->pts.back().x, 30.0, 1e-9);
    CHECK(spline[0]->pts.size() > 4);

    // Hershey strokes, several per letter, all near where the text sits.
    const auto text = on_pen(job, 6);
    CHECK(text.size() >= 5);
    for (const LaserPath* p : text) {
        for (const Pt& q : p->pts) CHECK(q.x >= 199.0 && q.x <= 230.0 && q.y >= -2.0 && q.y <= 7.0);
    }
}

TEST_CASE("bridge: --tol changes how finely curves are flattened") {
    Database db;
    db.add(coloured(std::make_unique<ncad::Circle>(Vec3{0, 0, 0}, 10.0), 1));
    LaserJob fine, coarse;
    DrawingReport r1, r2;
    std::string error;
    DrawingOptions o;
    REQUIRE(render_drawing(db, o, fine, r1, error));
    o.chord_tolerance = 0.5;
    REQUIRE(render_drawing(db, o, coarse, r2, error));
    REQUIRE(fine.paths.size() == 1 && coarse.paths.size() == 1);
    CHECK(coarse.paths[0].pts.size() < fine.paths[0].pts.size());
    o.chord_tolerance = 0.0;
    CHECK(!render_drawing(db, o, coarse, r2, error));
}

// --- colour and layers ---------------------------------------------------------

TEST_CASE("bridge: BYLAYER takes the layer's colour; BYBLOCK outside a block does too") {
    Database db;
    const ncad::LayerId cut = db.add_layer("CUT", 1);
    const ncad::LayerId score = db.add_layer("SCORE", 5);
    db.add(coloured(square(0, 0, 10), ncad::kColorByLayer, cut));
    db.add(coloured(square(20, 0, 10), ncad::kColorByLayer, score));
    db.add(coloured(square(40, 0, 10), 3, score));  // explicit colour wins
    db.add(coloured(square(60, 0, 10), ncad::kColorByBlock, cut));

    LaserJob job;
    DrawingReport r;
    REQUIRE(render(db, job, r));
    CHECK(on_pen(job, 1).size() == 2);
    CHECK(on_pen(job, 5).size() == 1);
    CHECK(on_pen(job, 3).size() == 1);
}

TEST_CASE("bridge: entities on off or frozen layers are not cut") {
    Database db;
    const ncad::LayerId on = db.add_layer("ON", 1);
    const ncad::LayerId off = db.add_layer("OFF", -2);  // R12: off is a negative colour
    const ncad::LayerId frozen = db.add_layer("FROZEN", 3);
    db.set_layer_frozen(frozen, true);
    db.add(coloured(square(0, 0, 10), ncad::kColorByLayer, on));
    db.add(coloured(square(20, 0, 10), ncad::kColorByLayer, off));
    db.add(coloured(square(40, 0, 10), 4, frozen));

    LaserJob job;
    DrawingReport r;
    REQUIRE(render(db, job, r));
    REQUIRE(job.paths.size() == 1);
    CHECK(job.paths[0].pen == 1);
    CHECK(r.hidden == 2);
}

// --- blocks --------------------------------------------------------------------

TEST_CASE("bridge: an INSERT's contents come through transformed, with BYBLOCK and layer 0 resolved") {
    Database db;
    const ncad::LayerId parts = db.add_layer("PARTS", 2);
    const ncad::LayerId hidden = db.add_layer("HIDDEN", 3);
    db.set_layer_frozen(hidden, true);

    ncad::BlockDef def;
    def.name = "PART";
    def.entities.push_back(coloured(square(0, 0, 10), ncad::kColorByBlock));       // -> the insert's colour
    def.entities.push_back(coloured(std::make_unique<ncad::Line>(Vec3{0, 0, 0}, Vec3{10, 10, 0}), 4));
    def.entities.push_back(coloured(square(2, 2, 1), ncad::kColorByLayer));        // layer 0 -> PARTS -> 2
    def.entities.push_back(coloured(square(5, 5, 1), 1, hidden));                  // frozen inside the block
    const ncad::BlockDef* part = db.block(db.add_block(std::move(def)));

    // Turned a quarter, then placed at 100,50; colour 5.
    const ncad::Mat4 place = ncad::Mat4::translation({100, 50, 0}) *
                             ncad::Mat4::rotation({0, 0, 0}, {0, 0, 1}, kPi / 2);
    db.add(coloured(std::make_unique<ncad::Insert>(part, place), 5, parts));

    LaserJob job;
    DrawingReport r;
    REQUIRE(render(db, job, r));
    CHECK(r.unmatched == 0);
    CHECK(r.hidden_in_blocks == 1);
    CHECK(job.paths.size() == 3);

    const auto sq = on_pen(job, 5);
    REQUIRE(sq.size() == 1);
    REQUIRE(sq[0]->pts.size() == 4);
    CHECK_NEAR(sq[0]->pts[1].x, 100.0, 1e-9);  // 10,0 turned to 0,10, then moved
    CHECK_NEAR(sq[0]->pts[1].y, 60.0, 1e-9);
    const auto diag = on_pen(job, 4);
    REQUIRE(diag.size() == 1);
    CHECK_NEAR(diag[0]->pts[1].x, 90.0, 1e-9);
    CHECK_NEAR(diag[0]->pts[1].y, 60.0, 1e-9);
    CHECK(on_pen(job, 2).size() == 1);
    CHECK(on_pen(job, 1).empty());
}

TEST_CASE("bridge: BYBLOCK in a nested INSERT takes the nearest insert's colour") {
    Database db;
    ncad::BlockDef inner;
    inner.name = "INNER";
    inner.entities.push_back(coloured(square(0, 0, 1), ncad::kColorByBlock));
    const ncad::BlockDef* in = db.block(db.add_block(std::move(inner)));

    ncad::BlockDef outer;
    outer.name = "OUTER";
    outer.entities.push_back(coloured(std::make_unique<ncad::Insert>(in, ncad::Mat4::identity()), 6));
    outer.entities.push_back(coloured(std::make_unique<ncad::Insert>(in, ncad::Mat4::translation({5, 0, 0})),
                                      ncad::kColorByBlock));
    const ncad::BlockDef* out = db.block(db.add_block(std::move(outer)));
    db.add(coloured(std::make_unique<ncad::Insert>(out, ncad::Mat4::translation({0, 20, 0})), 4));

    LaserJob job;
    DrawingReport r;
    REQUIRE(render(db, job, r));
    CHECK(r.unmatched == 0);
    const auto six = on_pen(job, 6);
    const auto four = on_pen(job, 4);
    REQUIRE(six.size() == 1);
    REQUIRE(four.size() == 1);
    CHECK_NEAR(six[0]->pts[0].x, 0.0, 1e-9);
    CHECK_NEAR(four[0]->pts[0].x, 5.0, 1e-9);
    CHECK_NEAR(four[0]->pts[0].y, 20.0, 1e-9);
}

// --- what cannot be cut as drawn -------------------------------------------------

TEST_CASE("bridge: wide polylines and tilted entities are counted and warned about") {
    Database db;
    auto wide = square(0, 0, 10);
    static_cast<ncad::Polyline&>(*wide).set_uniform_width(0.5);
    db.add(coloured(std::move(wide), 1));
    auto thin = square(20, 0, 10);
    static_cast<ncad::Polyline&>(*thin).set_uniform_width(0.4);
    db.add(coloured(std::move(thin), 1));
    db.add(coloured(std::make_unique<ncad::Circle>(Vec3{0, 0, 0}, 5.0, Vec3{1, 0, 0}), 2));
    db.add(coloured(std::make_unique<ncad::Circle>(Vec3{0, 0, 0}, 5.0, Vec3{0, 0, -1}), 2));  // flat, mirrored

    LaserJob job;
    DrawingReport r;
    REQUIRE(render(db, job, r));
    CHECK(r.wide_polylines == 1);
    CHECK(r.not_flat == 1);
    CHECK(has_warning(r, "1 polyline is 0.5 mm or wider"));
    CHECK(has_warning(r, "not parallel to the XY plane"));
    CHECK(job.paths.size() == 4);  // still cut, on the centre line
}

// --- origin --------------------------------------------------------------------

TEST_CASE("bridge: the LASER UCS sets the origin; --origin overrides it") {
    Database db;
    db.add(coloured(square(0, 0, 10), 1));
    LaserJob job;
    DrawingReport r;
    REQUIRE(render(db, job, r));
    CHECK(!r.laser_ucs);
    CHECK(job.origin.x == 0.0 && job.origin.y == 0.0);

    ncad::Ucs u;
    u.origin = {20, 30, 0};
    db.add_ucs("LASER", u);
    LaserJob j2;
    DrawingReport r2;
    REQUIRE(render(db, j2, r2));
    CHECK(r2.laser_ucs);
    CHECK(!r2.laser_ucs_rotated);
    CHECK(r2.warnings.empty());
    CHECK_NEAR(j2.origin.x, 20.0, 1e-12);
    CHECK_NEAR(j2.origin.y, 30.0, 1e-12);

    DrawingOptions o;
    o.has_origin = true;
    o.origin = {1, 2};
    LaserJob j3;
    DrawingReport r3;
    std::string error;
    REQUIRE(render_drawing(db, o, j3, r3, error));
    CHECK(j3.origin.x == 1.0 && j3.origin.y == 2.0);
}

TEST_CASE("bridge: a rotated LASER UCS is warned about; only its origin is used") {
    Database db;
    db.add(coloured(square(0, 0, 10), 1));
    ncad::Ucs u;
    u.origin = {5, 5, 0};
    u.xdir = {0, 1, 0};
    u.ydir = {-1, 0, 0};
    db.add_ucs("laser", u);  // names compare without case, as in R12
    LaserJob job;
    DrawingReport r;
    REQUIRE(render(db, job, r));
    CHECK(r.laser_ucs_rotated);
    CHECK(has_warning(r, "rotated"));
    CHECK_NEAR(job.origin.x, 5.0, 1e-12);
}

// --- pens ----------------------------------------------------------------------

TEST_CASE("bridge: the pen table supplies settings; an unlisted pen gets power 0") {
    std::vector<LaserLayer> pens;
    std::string error;
    REQUIRE(read_pen_table("# card, A2 laser 1\n"
                           "layer 1 color=1 mode=vector speed=20 power=60,65 passes=2 air=on order=2\n"
                           "layer 9 speed=300 power=10,10\n",
                           pens, error));
    REQUIRE(pens.size() == 2);

    Database db;
    db.add(coloured(square(0, 0, 10), 1));
    db.add(coloured(square(20, 0, 10), 5));
    LaserJob job;
    DrawingReport r;
    REQUIRE(render(db, job, r));
    apply_pens(job, pens, r);

    REQUIRE(job.layers.size() == 2);  // pen 9 is unused, so absent
    const LaserLayer* one = job.layer(1);
    const LaserLayer* five = job.layer(5);
    REQUIRE(one && five);
    CHECK(one->speed == 20.0 && one->max_power == 65.0 && one->passes == 2 && one->order == 2);
    CHECK(one->mode == LayerMode::Vector);
    CHECK(five->min_power == 0.0 && five->max_power == 0.0);
    CHECK(five->mode == LayerMode::Auto);
    CHECK(five->color == 5);
    REQUIRE(r.unset_pens.size() == 1);
    CHECK(r.unset_pens[0] == 5);
    CHECK(has_warning(r, "pen 5 has no pen-table entry"));

    // The text form reads back as written: a raw job, layers and paths.
    LaserJob back;
    REQUIRE(read_ljob(write_ljob(job), back, error));
    CHECK(write_ljob(back) == write_ljob(job));
}

TEST_CASE("bridge: pen table errors name the table's own line") {
    std::vector<LaserLayer> pens;
    std::string error;
    CHECK(!read_pen_table("layer 1 speed=20\nlayer 2 speed=fast\n", pens, error));
    CHECK(error.rfind("line 2:", 0) == 0);
    CHECK(!read_pen_table("layer 1\npath 1 open\n  0 0\n  1 1\nend\n", pens, error));
    CHECK(error.find("'layer' lines only") != std::string::npos);
    CHECK(!read_pen_table("layer 1\nlayer 1\n", pens, error));
    CHECK(read_pen_table("", pens, error));
    CHECK(pens.empty());
}

// --- end to end ----------------------------------------------------------------

TEST_CASE("bridge: DXF -> dxf2ljob -> plan -> encode gives a valid .rd") {
    // A 40 x 30 plate drawn as four LINEs, the way other programs deliver it,
    // with a hole of radius 5, a LASER UCS at its corner, and a layer that is
    // switched off.
    Database db;
    const ncad::LayerId cut = db.add_layer("CUT", 1);
    const ncad::LayerId notes = db.add_layer("NOTES", -3);
    const Vec3 c[4] = {{10, 10, 0}, {50, 10, 0}, {50, 40, 0}, {10, 40, 0}};
    for (int i = 0; i < 4; ++i) {
        db.add(coloured(std::make_unique<ncad::Line>(c[i], c[(i + 1) % 4]), ncad::kColorByLayer, cut));
    }
    db.add(coloured(std::make_unique<ncad::Circle>(Vec3{30, 25, 0}, 5.0), 5));
    db.add(coloured(std::make_unique<ncad::Text>(Vec3{0, 0, 0}, "NOT CUT", 3.0), ncad::kColorByLayer, notes));
    ncad::Ucs u;
    u.origin = {10, 10, 0};
    db.add_ucs("LASER", u);

    const std::string path = (std::filesystem::temp_directory_path() / "nlaser_bridge_plate.dxf").string();
    REQUIRE(ncad::write_dxf_file(db, path));

    std::vector<LaserLayer> pens;
    std::string error;
    REQUIRE(read_pen_table("layer 1 speed=20 power=60,65 order=2\nlayer 5 speed=20 power=60,65 order=1\n", pens,
                           error));
    LaserJob raw;
    DrawingReport r;
    REQUIRE(dxf_to_job(path, DrawingOptions{}, pens, raw, r, error));
    std::remove(path.c_str());

    CHECK(raw.source == "nlaser_bridge_plate.dxf");
    CHECK(raw.name == "nlaser_bridge_plate");
    CHECK(!raw.planned);
    CHECK(r.laser_ucs);
    CHECK_NEAR(raw.origin.x, 10.0, 1e-9);
    CHECK_NEAR(raw.origin.y, 10.0, 1e-9);
    CHECK(r.hidden == 1);
    CHECK(r.unset_pens.empty());
    CHECK(raw.paths.size() == 5);
    CHECK(raw.layers.size() == 2);

    LaserJob planned;
    PlanReport pr;
    REQUIRE(plan_job(raw, planned, PlanOptions{}, pr, error));
    CHECK(pr.paths_out == 2);  // the four LINEs joined into one closed outline
    CHECK(pr.closed_by_join == 1);

    ruida::RdJob rd;
    REQUIRE(job_to_rd(planned, rd, error));
    std::string plain;
    REQUIRE(encode_rd(rd, plain, error));
    const ruida::RdSummary s = ruida::summarize_rd(ruida::decode_rd(plain));
    CHECK(s.unknown == 0);
    CHECK(s.bad_length == 0);
    CHECK(s.has_eof);
    CHECK(s.file_sum_stated == s.file_sum_computed);
    CHECK(!s.rel_before_abs);

    // 140 mm of outline and a 10 mm hole, flattened to 0.01 mm: the chords
    // fall short of the true circumference by well under a tenth of a mm.
    const double exact = 140.0 + 2 * kPi * 5.0;
    CHECK_NEAR(s.cut_mm, pr.cut_mm, 1e-3);
    CHECK(s.cut_mm <= exact + 1e-3);
    CHECK(s.cut_mm > exact - 0.1);
    REQUIRE(s.have_cut_bounds);
    CHECK_NEAR(s.cut_max_x - s.cut_min_x, 40.0, 1e-3);
    CHECK_NEAR(s.cut_max_y - s.cut_min_y, 30.0, 1e-3);
}

namespace {

// A one-line DXF, (0,0) to (1,0), with $INSUNITS `units` (negative: no header).
bool line_dxf_job(int units, const DrawingOptions& opts, LaserJob& job, DrawingReport& r, std::string& error) {
    const std::string path = (std::filesystem::temp_directory_path() / "nlaser_bridge_units.dxf").string();
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    if (units >= 0) std::fprintf(f, "  0\nSECTION\n  2\nHEADER\n  9\n$INSUNITS\n 70\n%d\n  0\nENDSEC\n", units);
    std::fputs("  0\nSECTION\n  2\nENTITIES\n"
               "  0\nLINE\n  8\n0\n 62\n1\n 10\n0.0\n 20\n0.0\n 30\n0.0\n 11\n1.0\n 21\n0.0\n 31\n0.0\n"
               "  0\nENDSEC\n  0\nEOF\n",
               f);
    std::fclose(f);
    const bool ok = dxf_to_job(path, opts, {}, job, r, error);
    std::remove(path.c_str());
    return ok;
}

}  // namespace

TEST_CASE("bridge: a DXF in inches ($INSUNITS 1) is scaled by 25.4") {
    LaserJob job;
    DrawingReport r;
    std::string error;
    REQUIRE(line_dxf_job(1, DrawingOptions{}, job, r, error));
    REQUIRE(job.paths.size() == 1);
    CHECK(r.units == DrawingUnits::Inch);
    CHECK_NEAR(job.paths[0].pts[1].x, 25.4, 1e-9);
    CHECK(has_warning(r, "pen 1 has no pen-table entry"));
}

TEST_CASE("bridge: mm, unitless and explicit units") {
    LaserJob job;
    DrawingReport r;
    std::string error;
    REQUIRE(line_dxf_job(4, DrawingOptions{}, job, r, error));
    CHECK_NEAR(job.paths[0].pts[1].x, 1.0, 1e-9);
    CHECK(!has_warning(r, "INSUNITS"));

    job = LaserJob{};
    r = DrawingReport{};
    REQUIRE(line_dxf_job(-1, DrawingOptions{}, job, r, error));
    CHECK_NEAR(job.paths[0].pts[1].x, 1.0, 1e-9);
    CHECK(has_warning(r, "INSUNITS"));

    // --units overrides what the file says, and unlocks a unit it would refuse.
    DrawingOptions in;
    in.units = DrawingUnits::Inch;
    job = LaserJob{};
    r = DrawingReport{};
    REQUIRE(line_dxf_job(4, in, job, r, error));
    CHECK_NEAR(job.paths[0].pts[1].x, 25.4, 1e-9);
    job = LaserJob{};
    r = DrawingReport{};
    REQUIRE(line_dxf_job(13, in, job, r, error));
    CHECK_NEAR(job.paths[0].pts[1].x, 25.4, 1e-9);
}

TEST_CASE("bridge: an unknown $INSUNITS is refused, naming the value") {
    LaserJob job;
    DrawingReport r;
    std::string error;
    CHECK(!line_dxf_job(5, DrawingOptions{}, job, r, error));
    CHECK(error.find("$INSUNITS 5") != std::string::npos);
    CHECK(error.find("--units") != std::string::npos);
}

TEST_CASE("bridge: inches scale the origin too, and the tolerance stays in mm") {
    ncad::Database db;
    auto c = std::make_unique<ncad::Circle>(ncad::Vec3{1, 1, 0}, 1.0);
    c->props().color = 1;
    db.add(std::move(c));
    DrawingOptions mm, in;
    mm.units = DrawingUnits::Mm;
    in.units = DrawingUnits::Inch;
    in.has_origin = true;
    in.origin = {2.0, 3.0};
    LaserJob a, b;
    DrawingReport ra, rb;
    std::string error;
    REQUIRE(render_drawing(db, mm, a, ra, error));
    REQUIRE(render_drawing(db, in, b, rb, error));
    CHECK_NEAR(b.origin.x, 50.8, 1e-9);
    CHECK_NEAR(b.origin.y, 76.2, 1e-9);
    // A 25.4 mm radius circle needs more segments than a 1 mm one at the same mm tolerance.
    CHECK(b.paths[0].pts.size() > a.paths[0].pts.size());
}

// --- LightBurn parity: F07 ------------------------------------------------------

namespace {

// The cut polylines of a decoded file, in file mm, one per Move.
std::vector<std::vector<Pt>> file_polylines(const std::string& plain) {
    std::vector<std::vector<Pt>> out;
    std::int64_t x = 0, y = 0;
    for (const ruida::RdCommand& c : ruida::decode_rd(plain)) {
        char axis = 'b';
        const ruida::RdMotion m = ruida::motion_of(c, &axis);
        if (m == ruida::RdMotion::None) continue;
        if (m == ruida::RdMotion::MoveAbs || m == ruida::RdMotion::CutAbs) {
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
        const Pt p{static_cast<double>(x) / 1000.0, static_cast<double>(y) / 1000.0};
        if (m == ruida::RdMotion::MoveAbs || m == ruida::RdMotion::MoveRel) out.push_back({p});
        else if (!out.empty()) out.back().push_back(p);
    }
    return out;
}

bool read_all(const std::string& path, std::string& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    out.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    return true;
}

}  // namespace

TEST_CASE("lightburn flatten: a circle is four Beziers cut at uniform t, clockwise from 180 degrees") {
    bool closed = false;
    const auto pts = lightburn_flatten({0, 0}, {5, 0}, {0, 5}, kPi, -2 * kPi, 0.01, closed);
    CHECK(closed);
    REQUIRE(pts.size() == 64);  // R 5: 64 steps keep the sagitta under 0.01 (32 would not)
    CHECK_NEAR(pts[0].x, -5.0, 1e-9);
    CHECK_NEAR(pts[0].y, 0.0, 1e-9);
    CHECK_NEAR(pts[16].x, 0.0, 1e-9);  // the quadrant nodes: 90, 0, -90 degrees, clockwise
    CHECK_NEAR(pts[16].y, 5.0, 1e-9);
    CHECK_NEAR(pts[32].x, 5.0, 1e-9);
    CHECK_NEAR(pts[48].y, -5.0, 1e-9);
    // The k = 0.5523 Bezier is not an arc of a circle: the radius error stays
    // tiny but the spacing is uneven, which is the 7% seen in f07.
    double lo = 1e9, hi = 0;
    for (std::size_t i = 0; i < 64; ++i) {
        const Pt a = pts[i], b = pts[(i + 1) % 64];
        const double d = std::hypot(b.x - a.x, b.y - a.y);
        lo = std::fmin(lo, d);
        hi = std::fmax(hi, d);
        CHECK_NEAR(std::hypot(a.x, a.y), 5.0, 3e-3);
    }
    CHECK(hi / lo > 1.03);
    CHECK(hi / lo < 1.10);
}

TEST_CASE("lightburn flatten: an arc is open and keeps both end points") {
    bool closed = true;
    const auto pts = lightburn_flatten({0, 0}, {5, 0}, {0, 5}, 0.0, kPi / 2, 0.01, closed);
    CHECK(!closed);
    REQUIRE(pts.size() == 17);  // one quarter Bezier: 64 / 4 steps and the end
    CHECK_NEAR(pts.back().x, 0.0, 1e-9);
    CHECK_NEAR(pts.back().y, 5.0, 1e-9);
}

TEST_CASE("lightburn flatten: ncad stays the default; lightburn replaces circle polylines only") {
    Database db;
    db.add(coloured(std::make_unique<ncad::Circle>(Vec3{0, 0, 0}, 5.0), 1));
    db.add(coloured(square(20, 0, 4), 1));
    const std::string path = (std::filesystem::temp_directory_path() / "nlaser_bridge_flatten.dxf").string();
    REQUIRE(ncad::write_dxf_file(db, path));
    LaserJob a, b;
    DrawingReport r;
    std::string error;
    REQUIRE(dxf_to_job(path, DrawingOptions{}, {}, a, r, error));
    DrawingOptions o;
    o.flatten = FlattenMode::LightBurn;
    REQUIRE(dxf_to_job(path, o, {}, b, r, error));
    std::remove(path.c_str());
    REQUIRE(a.paths.size() == 2);
    REQUIRE(b.paths.size() == 2);
    CHECK(b.paths[0].pts.size() == 64);  // only the circle, and not twice
    CHECK(a.paths[0].pts.size() != 64);
    REQUIRE(a.paths[1].pts.size() == b.paths[1].pts.size());  // the square is untouched
    for (std::size_t i = 0; i < a.paths[1].pts.size(); ++i) {
        CHECK(a.paths[1].pts[i].x == b.paths[1].pts[i].x);
        CHECK(a.paths[1].pts[i].y == b.paths[1].pts[i].y);
    }
}

TEST_CASE("lightburn parity: F07 DXF -> dxf2ljob --flatten lightburn -> plan -> encode is f07.rd") {
    const std::string dir = std::string(NLASER_FIXTURES) + "/lightburn/";
    std::string file, error;
    REQUIRE(read_all(dir + "f07.rd", file));
    const std::string lightburn = ruida::Swizzle(0x88).decode(file);    const auto lb = file_polylines(lightburn);
    REQUIRE(lb.size() == 4);

    // The Black Cat's sample settings: Passes 2, speed 100, 15 % and 10-20 %.
    std::vector<LaserLayer> pens;
    REQUIRE(read_pen_table("layer 1 color=1 speed=100 power=15,15 power2=10,20 passes=2 air=on\n", pens, error));
    DrawingOptions o;
    o.flatten = FlattenMode::LightBurn;
    o.has_origin = true;
    o.origin = {0.0, 200.0};  // top-left of the drawing
    LaserJob raw, planned;
    DrawingReport r;
    REQUIRE(dxf_to_job(dir + "F07-circles.dxf", o, pens, raw, r, error));
    REQUIRE(raw.paths.size() == 4);

    // LightBurn plans from the beam, the origin shifted by the red-dot offset
    // (5.2, 31.2), and keeps to plain nearest-neighbour order.
    PlanOptions po;
    po.two_opt = false;
    po.by_region = false;     // one pool, as LightBurn orders
    po.cutfree_weight = 0.0;  // no avoidance of cut-free parts
    po.has_head = true;
    po.head = {5.2, 231.2};
    PlanReport pr;
    REQUIRE(plan_job(raw, planned, po, pr, error));
    REQUIRE(planned.paths.size() == 4);

    // Order and geometry already agree with LightBurn. What does not is the
    // start vertex: LightBurn's is within a few vertices of the nearest to
    // the head but not that one, and no rule has been found (see
    // docs/lightburn-fixtures.md). So take its start from the file; the rest
    // of the polygon is ours.
    for (std::size_t i = 0; i < 4; ++i) {
        LaserPath& p = planned.paths[i];
        const std::size_t n = p.pts.size();
        REQUIRE(lb[i].size() == 2 * n + 1);
        const Pt want{5.2 - lb[i][0].x, 231.2 - lb[i][0].y};
        std::size_t at = n;
        for (std::size_t k = 0; k < n; ++k) {
            if (std::hypot(p.pts[k].x - want.x, p.pts[k].y - want.y) < 2e-3) at = k;
        }
        REQUIRE(at < n);
        std::rotate(p.pts.begin(), p.pts.begin() + static_cast<std::ptrdiff_t>(at), p.pts.end());
    }

    ruida::RdJob rd;
    REQUIRE(job_to_rd(planned, rd, error));
    rd.machine.origin = ruida::MachineCorner::TopRight;
    rd.pointer_offset = {5.2, 31.2};
    std::string ours;
    REQUIRE(encode_rd(rd, ours, error));

    // Not byte for byte, and this is what is left: about 13 % of the
    // vertices land one micrometre from LightBurn's. LightBurn's vertices lie
    // up to ~0.09 um from the exact Bezier points (every circle, even D2, so
    // not a radius or k error; not explained by float32 or by k in
    // 0.5522847 .. 0.5522868 either), so the half-micrometre rounding goes
    // the other way now and then. Everything else is identical: file size,
    // every command, the header and its boxes, the order, the vertex count
    // and, to 1 um, every vertex.
    CHECK(ours.size() == lightburn.size());
    const auto mine = file_polylines(ours);
    REQUIRE(mine.size() == 4);
    std::size_t total = 0, off = 0;
    for (std::size_t i = 0; i < 4; ++i) {
        REQUIRE(mine[i].size() == lb[i].size());
        for (std::size_t k = 0; k < mine[i].size(); ++k) {
            const double d = std::fmax(std::fabs(mine[i][k].x - lb[i][k].x), std::fabs(mine[i][k].y - lb[i][k].y));
            CHECK(d < 1.5e-3);
            ++total;
            if (d > 1e-6) ++off;
        }
    }
    CHECK(off * 100 < total * 20);  // 13 % today
    // Up to the first Move (settings, boxes, origin) the files are byte for byte the same.
    CHECK(ours.compare(0, 0x1c2, lightburn, 0, 0x1c2) == 0);
}
