// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// SVG import without NotoCAD: units, the Y flip, colours, groups, widths,
// fills, curves and the flattened job. Inline SVGs, small enough to check by
// hand.

#include "test.hpp"

#include "ncad/laser/svg_import.hpp"

#include <cmath>
#include <string>
#include <vector>

using namespace ncad::laser;

namespace {

const char* kNs = "xmlns=\"http://www.w3.org/2000/svg\" xmlns:inkscape=\"http://www.inkscape.org/namespaces/inkscape\"";

std::string svg(const std::string& attrs, const std::string& body) {
    return std::string("<svg ") + kNs + " " + attrs + ">" + body + "</svg>";
}

bool load(const std::string& text, SvgImport& out, double px_per_in = 96.0) {
    SvgOptions o;
    o.px_per_in = px_per_in;
    std::string error;
    const bool ok = import_svg(text, o, out, error);
    if (!ok) std::printf("       import error: %s\n", error.c_str());
    return ok;
}

bool has_warning(const SvgImport& s, const char* part) {
    for (const std::string& w : s.warnings) {
        if (w.find(part) != std::string::npos) return true;
    }
    return false;
}

}  // namespace

TEST_CASE("svg: millimetres with a viewBox scale, Y flipped to Y-up") {
    SvgImport s;
    REQUIRE(load(svg("width=\"100mm\" height=\"50mm\" viewBox=\"0 0 200 100\"",
                     "<path d=\"M 20 10 L 120 10 L 120 60\" stroke=\"#ff0000\" fill=\"none\"/>"),
                 s));
    CHECK_NEAR(s.width_mm, 100.0, 1e-3);
    CHECK_NEAR(s.height_mm, 50.0, 1e-3);
    REQUIRE(s.shapes.size() == 1);
    const SvgShape& p = s.shapes[0];
    CHECK(!p.curve);
    CHECK(!p.closed);
    REQUIRE(p.pts.size() == 3);
    CHECK_NEAR(p.pts[0].x, 10.0, 1e-3);
    CHECK_NEAR(p.pts[0].y, 45.0, 1e-3);
    CHECK_NEAR(p.pts[1].x, 60.0, 1e-3);
    CHECK_NEAR(p.pts[1].y, 45.0, 1e-3);
    CHECK_NEAR(p.pts[2].x, 60.0, 1e-3);
    CHECK_NEAR(p.pts[2].y, 20.0, 1e-3);
    CHECK(s.warnings.empty());
}

TEST_CASE("svg: inches") {
    SvgImport s;
    REQUIRE(load(svg("width=\"2in\" height=\"1in\" viewBox=\"0 0 2 1\"",
                     "<path d=\"M 0 0 L 2 1\" stroke=\"#000000\" fill=\"none\"/>"),
                 s));
    CHECK_NEAR(s.width_mm, 50.8, 1e-3);
    REQUIRE(s.shapes.size() == 1);
    CHECK_NEAR(s.shapes[0].pts[0].x, 0.0, 1e-3);
    CHECK_NEAR(s.shapes[0].pts[0].y, 25.4, 1e-3);
    CHECK_NEAR(s.shapes[0].pts[1].x, 50.8, 1e-3);
    CHECK_NEAR(s.shapes[0].pts[1].y, 0.0, 1e-3);
}

TEST_CASE("svg: bare px are 96 to the inch, and --px-per-in changes that") {
    const std::string body = "<path d=\"M 0 0 L 96 0\" stroke=\"#000000\" fill=\"none\"/>";
    SvgImport s;
    REQUIRE(load(svg("width=\"96\" height=\"96\"", body), s));
    REQUIRE(s.shapes.size() == 1);
    CHECK_NEAR(s.shapes[0].pts[1].x - s.shapes[0].pts[0].x, 25.4, 1e-3);
    CHECK_NEAR(s.height_mm, 25.4, 1e-3);

    SvgImport t;
    REQUIRE(load(svg("width=\"96\" height=\"96\"", body), t, 72.0));
    REQUIRE(t.shapes.size() == 1);
    CHECK_NEAR(t.shapes[0].pts[1].x - t.shapes[0].pts[0].x, 96.0 / 72.0 * 25.4, 1e-3);

    // No size at all: the viewBox's units are px.
    SvgImport u;
    REQUIRE(load(svg("viewBox=\"0 0 96 96\"", body), u));
    REQUIRE(u.shapes.size() == 1);
    CHECK_NEAR(u.shapes[0].pts[1].x - u.shapes[0].pts[0].x, 25.4, 1e-3);
}

TEST_CASE("svg: stroke colour maps to ACI, exact first and then nearest") {
    SvgImport s;
    REQUIRE(load(svg("width=\"10mm\" height=\"10mm\" viewBox=\"0 0 10 10\"",
                     "<path d=\"M 0 0 L 1 1\" stroke=\"#ff0000\" fill=\"none\"/>"
                     "<path d=\"M 0 0 L 1 2\" stroke=\"#808080\" fill=\"none\"/>"
                     "<path d=\"M 0 0 L 1 3\" stroke=\"#ffffff\" fill=\"none\"/>"
                     "<path d=\"M 0 0 L 1 4\" stroke=\"#000000\" fill=\"none\"/>"),
                 s));
    REQUIRE(s.shapes.size() == 4);
    CHECK(s.shapes[0].aci == 1);
    CHECK(s.shapes[1].aci == 8);
    CHECK(s.shapes[2].aci == 7);
    CHECK(s.shapes[3].aci == 7);
    CHECK(s.warnings.empty());

    SvgImport n;
    REQUIRE(load(svg("width=\"10mm\" height=\"10mm\" viewBox=\"0 0 10 10\"",
                     "<path d=\"M 0 0 L 1 1\" stroke=\"#fe0202\" fill=\"none\"/>"),
                 n));
    REQUIRE(n.shapes.size() == 1);
    CHECK(n.shapes[0].aci == 1);
    CHECK(has_warning(n, "nearest"));
}

TEST_CASE("svg: rgb_to_aci is the inverse of aci_rgb") {
    for (int i = 1; i <= 255; ++i) {
        bool exact = false;
        const int back = rgb_to_aci(aci_rgb(i), &exact);
        CHECK(exact);
        CHECK(aci_rgb(back) == aci_rgb(i));
    }
    CHECK(rgb_to_aci(0xFFFFFF) == 7);
    CHECK(rgb_to_aci(0x000000) == 7);
}

TEST_CASE("svg: named groups become layers; generated ids do not") {
    SvgImport s;
    REQUIRE(load(svg("width=\"10mm\" height=\"10mm\" viewBox=\"0 0 10 10\"",
                     "<g id=\"Cut\" stroke=\"#00ff00\" fill=\"none\">"
                     "<path id=\"path1\" d=\"M 0 0 L 1 1\"/></g>"
                     "<g inkscape:label=\"Engrave\" stroke=\"#0000ff\" fill=\"none\">"
                     "<g id=\"g123\"><path d=\"M 0 0 L 2 2\"/></g></g>"
                     "<g id=\"g45\" stroke=\"#ff0000\" fill=\"none\"><path d=\"M 0 0 L 3 3\"/></g>"),
                 s));
    REQUIRE(s.shapes.size() == 3);
    CHECK(s.shapes[0].layer == "Cut");
    CHECK(s.shapes[0].aci == 3);
    CHECK(s.shapes[1].layer == "Engrave");
    CHECK(s.shapes[1].aci == 5);
    CHECK(s.shapes[2].layer == "0");
    REQUIRE(s.layers.size() == 3);
    CHECK(s.layers[0].name == "Cut");
    CHECK(s.layers[0].aci == 3);
}

TEST_CASE("svg: strokes of 0.5 mm and more become the polyline width") {
    SvgImport s;
    REQUIRE(load(svg("width=\"10mm\" height=\"10mm\" viewBox=\"0 0 10 10\"",
                     "<path d=\"M 0 0 L 1 1\" stroke=\"#ff0000\" stroke-width=\"1\" fill=\"none\"/>"
                     "<path d=\"M 0 0 L 1 2\" stroke=\"#ff0000\" stroke-width=\"0.1\" fill=\"none\"/>"
                     "<path d=\"M 0 0 L 1 3\" stroke=\"#ff0000\" stroke-width=\"0.5\" fill=\"none\"/>"),
                 s));
    REQUIRE(s.shapes.size() == 3);
    CHECK_NEAR(s.shapes[0].width, 1.0, 1e-3);
    CHECK_NEAR(s.shapes[1].width, 0.0, 1e-9);
    CHECK_NEAR(s.shapes[2].width, 0.5, 1e-3);

    // The width follows the page scale, not the raw number.
    SvgImport t;
    REQUIRE(load(svg("width=\"10mm\" height=\"10mm\" viewBox=\"0 0 100 100\"",
                     "<path d=\"M 0 0 L 10 10\" stroke=\"#ff0000\" stroke-width=\"10\" fill=\"none\"/>"),
                 t));
    REQUIRE(t.shapes.size() == 1);
    CHECK_NEAR(t.shapes[0].width, 1.0, 1e-3);
}

TEST_CASE("svg: a fill is warned about and its outline imported") {
    SvgImport s;
    REQUIRE(load(svg("width=\"10mm\" height=\"10mm\" viewBox=\"0 0 10 10\"",
                     "<rect x=\"1\" y=\"1\" width=\"4\" height=\"3\" fill=\"#0000ff\" stroke=\"none\"/>"),
                 s));
    CHECK(s.filled == 1);
    REQUIRE(s.shapes.size() == 1);
    CHECK(s.shapes[0].closed);
    CHECK(s.shapes[0].pts.size() == 4);
    CHECK(s.shapes[0].aci == 5);  // no stroke: the fill's colour
    CHECK(has_warning(s, "raster"));
}

TEST_CASE("svg: cubic Beziers stay exact, lines and curves split, closed rects close") {
    SvgImport s;
    REQUIRE(load(svg("width=\"10mm\" height=\"10mm\" viewBox=\"0 0 10 10\"",
                     "<path d=\"M 0 0 C 0 10 10 10 10 0\" stroke=\"#ff0000\" fill=\"none\"/>"),
                 s));
    REQUIRE(s.shapes.size() == 1);
    REQUIRE(s.shapes[0].curve);
    REQUIRE(s.shapes[0].pts.size() == 4);
    CHECK_NEAR(s.shapes[0].pts[0].x, 0.0, 1e-3);
    CHECK_NEAR(s.shapes[0].pts[0].y, 10.0, 1e-3);
    CHECK_NEAR(s.shapes[0].pts[1].y, 0.0, 1e-3);
    CHECK_NEAR(s.shapes[0].pts[2].x, 10.0, 1e-3);
    CHECK_NEAR(s.shapes[0].pts[3].x, 10.0, 1e-3);
    CHECK_NEAR(s.shapes[0].pts[3].y, 10.0, 1e-3);

    SvgImport m;
    REQUIRE(load(svg("width=\"20mm\" height=\"10mm\" viewBox=\"0 0 20 10\"",
                     "<path d=\"M 0 0 L 10 0 C 15 0 15 5 10 5 L 0 5 Z\" stroke=\"#ff0000\" fill=\"none\"/>"),
                 m));
    REQUIRE(m.shapes.size() == 3);
    CHECK(!m.shapes[0].curve);
    CHECK(m.shapes[1].curve);
    CHECK(!m.shapes[2].curve);
    CHECK(!m.shapes[0].closed);

    SvgImport r;
    REQUIRE(load(svg("width=\"10mm\" height=\"10mm\" viewBox=\"0 0 10 10\"",
                     "<path d=\"M 0 0 L 10 0 L 10 10 L 0 10 Z\" stroke=\"#ff0000\" fill=\"none\"/>"),
                 r));
    REQUIRE(r.shapes.size() == 1);
    CHECK(r.shapes[0].closed);
    CHECK(r.shapes[0].pts.size() == 4);
}

TEST_CASE("svg: svg_to_job flattens curves within the tolerance and gives unknown pens power 0") {
    SvgImport s;
    REQUIRE(load(svg("width=\"10mm\" height=\"10mm\" viewBox=\"0 0 10 10\"",
                     "<path d=\"M 0 0 C 0 10 10 10 10 0\" stroke=\"#ff0000\" stroke-width=\"0.1\" fill=\"none\"/>"),
                 s));
    LaserJob job;
    std::vector<std::string> warnings;
    std::string error;
    REQUIRE(svg_to_job(s, 0.01, {}, "a/b/curve.svg", job, warnings, error));
    CHECK(job.name == "curve");
    CHECK(job.source == "curve.svg");
    REQUIRE(job.paths.size() == 1);
    CHECK(job.paths[0].pen == 1);
    CHECK(job.paths[0].pts.size() > 8);
    // The exact curve's lowest point is y = 2.5 at x = 5 (flipped).
    double lowest = 1e9;
    for (const Pt& p : job.paths[0].pts) lowest = std::min(lowest, p.y);
    CHECK(lowest >= 2.5 - 1e-6);
    CHECK(lowest <= 2.5 + 0.01);
    REQUIRE(job.layers.size() == 1);
    CHECK_NEAR(job.layers[0].max_power, 0.0, 1e-12);
    bool warned = false;
    for (const std::string& w : warnings) warned = warned || w.find("power 0") != std::string::npos;
    CHECK(warned);

    LaserLayer pen;
    pen.pen = 1;
    pen.color = 1;
    pen.min_power = pen.max_power = 40.0;
    LaserJob j2;
    REQUIRE(svg_to_job(s, 0.01, {pen}, "curve.svg", j2, warnings, error));
    CHECK_NEAR(j2.layers[0].max_power, 40.0, 1e-12);
    CHECK(warnings.empty());

    CHECK(!svg_to_job(s, 0.0, {}, "curve.svg", j2, warnings, error));
}
