// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// SVGIN through NotoCAD's CommandEngine: entity types, layers, coordinates,
// one undo step, and that the drawing it makes cuts the same geometry as
// svg2ljob of the same file (through a DXF round trip, as dxf2ljob reads it).

#include "test.hpp"

#include "ncad/command.hpp"
#include "ncad/commands.hpp"
#include "ncad/database.hpp"
#include "ncad/dxf.hpp"
#include "ncad/entities.hpp"
#include "ncad/input_text.hpp"
#include "ncad/laser/laser_command.hpp"
#include "ncad/laser/laser_renderer.hpp"
#include "ncad/laser/svg_import.hpp"
#include "ncad/sysvar.hpp"

#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace ncad::laser;
using ncad::CommandEngine;
using ncad::Database;
using ncad::EngineStatus;
using ncad::EntityType;

namespace {

namespace fs = std::filesystem;

// Two layers: a wide-stroked rectangle on "Cut", and a curve plus a line on
// "Engrave" in another colour. 100 x 50 mm page, user units are mm.
const char* kArt =
    "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"100mm\" height=\"50mm\" viewBox=\"0 0 100 50\">"
    "<g id=\"Cut\" stroke=\"#ff0000\" stroke-width=\"1\" fill=\"none\">"
    "<path d=\"M 10 10 L 40 10 L 40 30 L 10 30 Z\"/></g>"
    "<g id=\"Engrave\" stroke=\"#0000ff\" stroke-width=\"0.1\" fill=\"none\">"
    "<path d=\"M 50 40 C 60 10 80 10 90 40\"/>"
    "<path d=\"M 50 45 L 90 45\"/></g>"
    "</svg>";

struct Fixture {
    Database db;
    CommandEngine engine{db};
    std::shared_ptr<LaserSession> session;
    fs::path dir;

    Fixture() {
        static int serial = 0;
        dir = fs::temp_directory_path() / ("nlaser_svgin_" + std::to_string(::getpid()) + "_" + std::to_string(++serial));
        fs::create_directories(dir);
        std::ofstream(dir / "art.svg", std::ios::binary) << kArt;
        session = install_laser(LaserOptions{});
    }
    ~Fixture() {
        ncad::clear_registered_commands();
        ncad::clear_registered_sysvars();
        std::error_code ec;
        fs::remove_all(dir, ec);
    }

    EngineStatus run(const std::string& file) {
        ncad::TextInputSource src({file});
        src.set_database(&db);
        engine.begin(ncad::make_command("SVGIN"));
        return engine.run(src);
    }

    std::string svg_path() const { return (dir / "art.svg").string(); }
};

// Every path of a job as a sorted list of rounded points, so two jobs can be
// compared as geometry. Closed or open and order along the path are kept; the
// drawing order of paths is not.
double max_dev_between(const LaserJob& a, const LaserJob& b) {
    // Each point of one lies on the other's polyline within the result.
    auto dist_to = [](const Pt& p, const LaserJob& j) {
        double best = 1e18;
        for (const LaserPath& path : j.paths) {
            const std::size_t n = path.pts.size();
            const std::size_t segs = path.closed ? n : n - 1;
            for (std::size_t i = 0; i < segs; ++i) {
                const Pt& s = path.pts[i];
                const Pt& e = path.pts[(i + 1) % n];
                const double dx = e.x - s.x, dy = e.y - s.y;
                const double l2 = dx * dx + dy * dy;
                double t = l2 > 0 ? ((p.x - s.x) * dx + (p.y - s.y) * dy) / l2 : 0.0;
                t = std::max(0.0, std::min(1.0, t));
                best = std::min(best, std::hypot(p.x - (s.x + t * dx), p.y - (s.y + t * dy)));
            }
        }
        return best;
    };
    double worst = 0.0;
    for (const LaserPath& path : a.paths) {
        for (const Pt& p : path.pts) worst = std::max(worst, dist_to(p, b));
    }
    for (const LaserPath& path : b.paths) {
        for (const Pt& p : path.pts) worst = std::max(worst, dist_to(p, a));
    }
    return worst;
}

}  // namespace

TEST_CASE("svgin: registered with its variable, default 96 px per inch") {
    Fixture f;
    REQUIRE(f.session != nullptr);
    CHECK(ncad::make_command("SVGIN") != nullptr);
    CHECK_NEAR(f.db.sysvars().get_real(f.session->svg_px), 96.0, 1e-12);
}

TEST_CASE("svgin: makes polylines and splines on named layers, in one undo step") {
    Fixture f;
    REQUIRE(f.run(f.svg_path()) == EngineStatus::Finished);
    CHECK(f.engine.message().find("2 polylines, 1 splines") != std::string::npos);

    std::size_t polylines = 0, splines = 0;
    const ncad::Polyline* rect = nullptr;
    const ncad::Spline* curve = nullptr;
    for (const ncad::Handle h : f.db.order()) {
        const ncad::Entity* e = f.db.get(h);
        if (e->type() == EntityType::Polyline) {
            ++polylines;
            if (!rect) rect = static_cast<const ncad::Polyline*>(e);
        } else if (e->type() == EntityType::Spline) {
            ++splines;
            curve = static_cast<const ncad::Spline*>(e);
        }
    }
    // The rectangle and the open line are polylines, the arc a spline.
    CHECK(polylines == 2);
    CHECK(splines == 1);
    REQUIRE(rect != nullptr);
    REQUIRE(curve != nullptr);

    CHECK(rect->closed());
    CHECK(rect->size() == 4);
    CHECK(rect->has_width());
    CHECK_NEAR(rect->vertices()[0].start_width, 1.0, 1e-3);
    CHECK_NEAR(rect->vertices()[0].pos.x, 10.0, 1e-3);
    CHECK_NEAR(rect->vertices()[0].pos.y, 40.0, 1e-3);  // SVG y 10 of 50, flipped

    CHECK(curve->degree() == 3);
    CHECK(curve->valid());
    REQUIRE(curve->control_points().size() == 4);
    CHECK(curve->knots().size() == 8);
    CHECK_NEAR(curve->control_points()[0].x, 50.0, 1e-3);
    CHECK_NEAR(curve->control_points()[0].y, 10.0, 1e-3);
    CHECK_NEAR(curve->control_points()[1].y, 40.0, 1e-3);
    CHECK_NEAR(curve->control_points()[3].x, 90.0, 1e-3);

    const ncad::LayerId cut = f.db.find_layer("Cut");
    const ncad::LayerId engrave = f.db.find_layer("Engrave");
    REQUIRE(cut != ncad::kInvalidLayer);
    REQUIRE(engrave != ncad::kInvalidLayer);
    CHECK(f.db.layer(cut).color == 1);
    CHECK(f.db.layer(engrave).color == 5);
    CHECK(rect->props().layer == cut);
    CHECK(curve->props().layer == engrave);
    CHECK(rect->props().color == ncad::kColorByLayer);

    // One command, one undo step: everything it added goes in one UNDO.
    CHECK(f.db.journal().undo_depth() == 1);
    CHECK(f.db.journal().undo(f.db));
    CHECK(f.db.empty());
    CHECK(f.db.find_layer("Cut") == ncad::kInvalidLayer);
}

TEST_CASE("svgin: a missing file fails and draws nothing") {
    Fixture f;
    CHECK(f.run((f.dir / "nope.svg").string()) == EngineStatus::Failed);
    CHECK(f.db.empty());
}

TEST_CASE("svgin: the drawing cuts the same geometry as svg2ljob") {
    Fixture f;
    REQUIRE(f.run(f.svg_path()) == EngineStatus::Finished);

    // Through DXF, as dxf2ljob reads it. R2000, because R12 has no SPLINE.
    const std::string dxf = (f.dir / "art.dxf").string();
    REQUIRE(ncad::write_dxf_file(f.db, dxf, ncad::DxfVersion::R2000));
    DrawingOptions opts;
    opts.chord_tolerance = 0.01;
    LaserJob from_dxf;
    DrawingReport report;
    std::string error;
    REQUIRE(dxf_to_job(dxf, opts, {}, from_dxf, report, error));

    SvgImport svg;
    REQUIRE(import_svg_file(f.svg_path(), SvgOptions{}, svg, error));
    LaserJob from_svg;
    std::vector<std::string> warnings;
    REQUIRE(svg_to_job(svg, 0.01, {}, f.svg_path(), from_svg, warnings, error));

    CHECK(from_dxf.paths.size() == from_svg.paths.size());
    // Both flatten at 0.01 mm, from the same curve, so each polyline stays
    // within about twice that of the other.
    CHECK(max_dev_between(from_dxf, from_svg) < 0.03);

    // Pens carry over: the layers' colours.
    std::vector<int> pens_dxf, pens_svg;
    for (const LaserPath& p : from_dxf.paths) pens_dxf.push_back(p.pen);
    for (const LaserPath& p : from_svg.paths) pens_svg.push_back(p.pen);
    std::sort(pens_dxf.begin(), pens_dxf.end());
    std::sort(pens_svg.begin(), pens_svg.end());
    CHECK(pens_dxf == pens_svg);
}
