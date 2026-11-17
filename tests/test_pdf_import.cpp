// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// The PDF content-stream interpreter, on small hand-written streams. No qpdf:
// the container layer is tested only by its refusals.

#include "test.hpp"

#include "ncad/laser/pdf_import.hpp"

#include <unistd.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>

using namespace ncad::laser;

namespace {

constexpr double kMm = 25.4 / 72.0;

// A 100 x 200 pt page at the origin unless a test says otherwise.
bool load(const std::string& stream, SvgImport& out, double x0 = 0, double y0 = 0, double x1 = 100, double y1 = 200,
          const std::map<std::string, std::string>& props = {}) {
    PdfPageData p;
    p.box[0] = x0;
    p.box[1] = y0;
    p.box[2] = x1;
    p.box[3] = y1;
    p.content = stream;
    p.properties = props;
    std::string error;
    const bool ok = import_pdf_content(p, out, error);
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

TEST_CASE("pdf: a line, in mm, the page's size from the MediaBox") {
    SvgImport s;
    REQUIRE(load("10 20 m 30 20 l S", s));
    CHECK_NEAR(s.width_mm, 100 * kMm, 1e-9);
    CHECK_NEAR(s.height_mm, 200 * kMm, 1e-9);
    REQUIRE(s.shapes.size() == 1u);
    const SvgShape& sh = s.shapes[0];
    CHECK(!sh.closed);
    CHECK(!sh.curve);
    REQUIRE(sh.pts.size() == 2u);
    CHECK_NEAR(sh.pts[0].x, 10 * kMm, 1e-9);
    CHECK_NEAR(sh.pts[0].y, 20 * kMm, 1e-9);
    CHECK_NEAR(sh.pts[1].x, 30 * kMm, 1e-9);
    CHECK(s.warnings.empty());
}

TEST_CASE("pdf: the MediaBox's lower-left corner is the origin") {
    SvgImport s;
    REQUIRE(load("110 120 m 130 120 l S", s, 100, 100, 200, 300));
    REQUIRE(s.shapes.size() == 1u);
    CHECK_NEAR(s.shapes[0].pts[0].x, 10 * kMm, 1e-9);
    CHECK_NEAR(s.shapes[0].pts[0].y, 20 * kMm, 1e-9);
    // A box given with its corners swapped is the same box.
    SvgImport t;
    REQUIRE(load("110 120 m 130 120 l S", t, 200, 300, 100, 100));
    CHECK_NEAR(t.shapes[0].pts[0].x, 10 * kMm, 1e-9);
    CHECK_NEAR(t.width_mm, 100 * kMm, 1e-9);
}

TEST_CASE("pdf: an empty MediaBox is an error") {
    SvgImport s;
    PdfPageData p;
    p.box[2] = 0;
    std::string error;
    CHECK(!import_pdf_content(p, s, error));
    CHECK(error.find("MediaBox") != std::string::npos);
}

TEST_CASE("pdf: re is a closed rectangle of four points, s closes an open path") {
    SvgImport s;
    REQUIRE(load("10 10 20 30 re S", s));
    REQUIRE(s.shapes.size() == 1u);
    CHECK(s.shapes[0].closed);
    REQUIRE(s.shapes[0].pts.size() == 4u);
    CHECK_NEAR(s.shapes[0].pts[2].x, 30 * kMm, 1e-9);
    CHECK_NEAR(s.shapes[0].pts[2].y, 40 * kMm, 1e-9);

    SvgImport t;
    REQUIRE(load("0 0 m 10 0 l 10 10 l s  20 0 m 30 0 l 30 10 l S  40 0 m 50 0 l 50 10 l h S", t));
    REQUIRE(t.shapes.size() == 3u);
    CHECK(t.shapes[0].closed);   // s
    CHECK(!t.shapes[1].closed);  // S
    CHECK(t.shapes[2].closed);   // h then S
    CHECK(t.shapes[0].pts.size() == 3u);
}

TEST_CASE("pdf: a path that returns to its start closes without repeating it") {
    SvgImport s;
    REQUIRE(load("0 0 m 10 0 l 10 10 l 0 0 l h S", s));
    REQUIRE(s.shapes.size() == 1u);
    CHECK(s.shapes[0].closed);
    CHECK(s.shapes[0].pts.size() == 3u);
}

TEST_CASE("pdf: c, v and y are cubic Beziers with the right control points") {
    SvgImport s;
    REQUIRE(load("0 0 m 10 20 30 40 50 60 c S", s));
    REQUIRE(s.shapes.size() == 1u);
    REQUIRE(s.shapes[0].curve);
    REQUIRE(s.shapes[0].pts.size() == 4u);
    CHECK_NEAR(s.shapes[0].pts[1].x, 10 * kMm, 1e-9);
    CHECK_NEAR(s.shapes[0].pts[3].y, 60 * kMm, 1e-9);

    SvgImport v;  // v: the first control point is the current point
    REQUIRE(load("5 5 m 30 40 50 60 v S", v));
    REQUIRE(v.shapes[0].pts.size() == 4u);
    CHECK_NEAR(v.shapes[0].pts[1].x, 5 * kMm, 1e-9);
    CHECK_NEAR(v.shapes[0].pts[2].x, 30 * kMm, 1e-9);
    CHECK_NEAR(v.shapes[0].pts[3].x, 50 * kMm, 1e-9);

    SvgImport y;  // y: the second control point is the end point
    REQUIRE(load("5 5 m 30 40 50 60 y S", y));
    CHECK_NEAR(y.shapes[0].pts[1].x, 30 * kMm, 1e-9);
    CHECK_NEAR(y.shapes[0].pts[2].x, 50 * kMm, 1e-9);
    CHECK_NEAR(y.shapes[0].pts[3].x, 50 * kMm, 1e-9);
}

TEST_CASE("pdf: a closed single curve is a closed spline; lines and curves split into runs") {
    SvgImport s;
    REQUIRE(load("0 0 m 10 20 30 20 40 0 c 30 -20 10 -20 0 0 c h S", s));
    REQUIRE(s.shapes.size() == 1u);
    CHECK(s.shapes[0].curve);
    SvgImport u;
    REQUIRE(load("0 0 m 10 20 30 20 40 0 c 30 -20 10 -20 0 0 c h S", u));
    REQUIRE(u.shapes.size() == 1u);
    CHECK(u.shapes[0].closed);
    CHECK(u.shapes[0].pts.size() == 7u);

    SvgImport t;
    REQUIRE(load("0 0 m 10 0 l 20 10 30 10 40 0 c S", t));
    REQUIRE(t.shapes.size() == 2u);
    CHECK(!t.shapes[0].curve);
    CHECK(t.shapes[1].curve);
    CHECK_NEAR(t.shapes[1].pts[0].x, 10 * kMm, 1e-9);  // the runs share an end point
}

TEST_CASE("pdf: cm and q/Q nest; Q restores the matrix") {
    SvgImport s;
    REQUIRE(load("q 2 0 0 2 10 10 cm 0 0 m 5 0 l S "
                 "q 1 0 0 1 100 0 cm 0 0 m 1 0 l S Q "
                 "0 0 m 1 0 l S Q "
                 "0 0 m 1 0 l S",
                 s, 0, 0, 1000, 1000));
    REQUIRE(s.shapes.size() == 4u);
    CHECK_NEAR(s.shapes[0].pts[0].x, 10 * kMm, 1e-9);
    CHECK_NEAR(s.shapes[0].pts[1].x, 20 * kMm, 1e-9);  // 2 x 5 + 10
    CHECK_NEAR(s.shapes[1].pts[0].x, (10 + 2 * 100) * kMm, 1e-9);
    CHECK_NEAR(s.shapes[2].pts[0].x, 10 * kMm, 1e-9);  // inner Q undone
    CHECK_NEAR(s.shapes[3].pts[0].x, 0.0, 1e-9);       // outer Q undone
}

TEST_CASE("pdf: cm concatenates in PDF order (the new matrix applies first)") {
    SvgImport s;
    // Translate then scale: a point at 1,0 goes to (1+10)*2 = 22.
    REQUIRE(load("2 0 0 2 0 0 cm 1 0 0 1 10 0 cm 1 0 m 2 0 l S", s, 0, 0, 1000, 1000));
    CHECK_NEAR(s.shapes[0].pts[0].x, 22 * kMm, 1e-9);
    // A 90 degree rotation sends x to y.
    SvgImport r;
    REQUIRE(load("0 1 -1 0 0 0 cm 5 0 m 6 0 l S", r, -100, 0, 100, 100));
    CHECK_NEAR(r.shapes[0].pts[0].x, 100 * kMm, 1e-9);  // x = 0 - (-100)
    CHECK_NEAR(r.shapes[0].pts[0].y, 5 * kMm, 1e-9);
}

TEST_CASE("pdf: fills are outlined and warned, and take the fill colour") {
    SvgImport s;
    REQUIRE(load("1 0 0 rg 0 0 10 10 re f", s));
    REQUIRE(s.shapes.size() == 1u);
    CHECK(s.shapes[0].closed);
    CHECK(s.shapes[0].aci == 1);
    CHECK(s.filled == 1u);
    CHECK(has_warning(s, "has a fill"));
    for (const char* op : {"F", "f*"}) {
        SvgImport t;
        REQUIRE(load(std::string("0 0 10 10 re ") + op, t));
        CHECK(t.filled == 1u);
    }
}

TEST_CASE("pdf: B and b paint with the stroke colour and count one fill; n paints nothing") {
    SvgImport s;
    REQUIRE(load("0 1 0 rg 0 0 1 RG 0 0 10 10 re B", s));
    CHECK(s.shapes[0].aci == 5);
    CHECK(s.filled == 1u);
    SvgImport t;
    REQUIRE(load("0 0 m 10 0 l 10 10 l b*", t));
    CHECK(t.shapes[0].closed);
    SvgImport u;
    REQUIRE(load("0 0 10 10 re n", u));
    CHECK(u.shapes.empty());
    CHECK(u.filled == 0u);
}

TEST_CASE("pdf: stroke colours in each device space map to ACI") {
    struct Row {
        const char* ops;
        int aci;
    };
    const Row rows[] = {
        {"1 0 0 RG", 1},   // red
        {"0 0 1 RG", 5},   // blue
        {"0 1 0 RG", 3},   // green
        {"1 1 0 RG", 2},   // yellow
        {"0 g", 7},        // black, which ACI 7 stands for
        {"1 G", 7},        // white too
        {"0 1 1 0 K", 1},  // CMYK red: no cyan, full magenta + yellow
        {"1 1 0 0 K", 5},  // cyan + magenta is blue
        {"0 0 0 1 K", 7},  // CMYK black
        {"0 0 0 0 k", 7},  // CMYK white
        {"/DeviceRGB CS 1 0 0 SC", 1},
        {"/DeviceGray CS 1 SC", 7},
    };
    for (const Row& r : rows) {
        SvgImport s;
        REQUIRE(load(std::string(r.ops) + " 0 0 m 1 0 l S", s));
        REQUIRE(s.shapes.size() == 1u);
        if (s.shapes[0].aci != r.aci) std::printf("       %s -> %d, wanted %d\n", r.ops, s.shapes[0].aci, r.aci);
        CHECK(s.shapes[0].aci == r.aci);
    }
}

TEST_CASE("pdf: colour state is saved by q and restored by Q") {
    SvgImport s;
    REQUIRE(load("1 0 0 RG q 0 0 1 RG 0 0 m 1 0 l S Q 0 0 m 2 0 l S", s));
    REQUIRE(s.shapes.size() == 2u);
    CHECK(s.shapes[0].aci == 5);
    CHECK(s.shapes[1].aci == 1);
}

TEST_CASE("pdf: a named colour space is a best-effort guess, warned") {
    SvgImport s;
    REQUIRE(load("/Cs8 CS 1 0 0 SCN 0 0 m 1 0 l S", s));
    CHECK(s.shapes[0].aci == 1);
    CHECK(has_warning(s, "Cs8"));
    // A tint of 1 in a spot space is full ink: dark.
    SvgImport t;
    REQUIRE(load("/Sep CS 1 SCN 0 0 m 1 0 l S", t));
    CHECK(t.shapes[0].aci == 7);
    CHECK(has_warning(t, "Sep"));
    // A pattern has no colour at all.
    SvgImport p;
    REQUIRE(load("/Pattern CS /P1 SCN 0 0 m 1 0 l S", p));
    CHECK(p.shapes[0].aci == 7);
    CHECK(has_warning(p, "no plain colour"));
}

TEST_CASE("pdf: line width is scaled by the CTM and 0.5 mm makes a stroke wide") {
    SvgImport s;
    // 1 pt = 0.3528 mm: hairline.
    REQUIRE(load("1 w 0 0 m 10 0 l S", s));
    CHECK_NEAR(s.shapes[0].width, 0.0, 1e-12);
    // 2 pt = 0.7056 mm: wide, and carried in mm.
    SvgImport t;
    REQUIRE(load("2 w 0 0 m 10 0 l S", t));
    CHECK_NEAR(t.shapes[0].width, 2 * kMm, 1e-9);
    // 1 pt under a 2x CTM is 2 pt.
    SvgImport u;
    REQUIRE(load("1 w 2 0 0 2 0 0 cm 0 0 m 10 0 l S", u));
    CHECK_NEAR(u.shapes[0].width, 2 * kMm, 1e-9);
    // The threshold is inclusive: 0.5 mm is 1.417 pt.
    SvgImport v;
    REQUIRE(load("1.4173228346 w 0 0 m 10 0 l S", v));
    CHECK(v.shapes[0].width > 0.0);
    // A wide curve cuts on its centre line, warned.
    SvgImport c;
    REQUIRE(load("3 w 0 0 m 10 20 30 20 40 0 c S", c));
    CHECK(has_warning(c, "centre line"));
}

TEST_CASE("pdf: optional content groups name layers; others share layer 0") {
    SvgImport s;
    REQUIRE(load("0 0 m 1 0 l S "
                 "/OC /MC0 BDC 1 0 0 RG 0 1 m 1 1 l S "
                 "  /OC /MC1 BDC 0 0 1 RG 0 2 m 1 2 l S EMC "
                 "  0 3 m 1 3 l S "
                 "EMC "
                 "/Span <</MCID 0>> BDC 0 4 m 1 4 l S EMC "
                 "0 5 m 1 5 l S",
                 s, 0, 0, 100, 200, {{"MC0", "Cut"}, {"MC1", "Engrave"}}));
    REQUIRE(s.shapes.size() == 6u);
    CHECK(s.shapes[0].layer == "0");
    CHECK(s.shapes[1].layer == "Cut");
    CHECK(s.shapes[2].layer == "Engrave");
    CHECK(s.shapes[3].layer == "Cut");  // back in the outer group
    CHECK(s.shapes[4].layer == "0");    // a non-OC BDC names no layer
    CHECK(s.shapes[5].layer == "0");
    REQUIRE(s.layers.size() == 3u);
    CHECK(s.layers[0].name == "0");
    CHECK(s.layers[1].name == "Cut");
    CHECK(s.layers[1].aci == 1);  // the colour of its first shape
    CHECK(s.layers[2].aci == 5);
}

TEST_CASE("pdf: an unresolved OC property keeps the enclosing layer") {
    SvgImport s;
    REQUIRE(load("/OC /Nope BDC 0 0 m 1 0 l S EMC", s));
    CHECK(s.shapes[0].layer == "0");
}

TEST_CASE("pdf: unknown operators are skipped with one warning naming them") {
    SvgImport s;
    REQUIRE(load("0 0 m foo 1 0 l bar S zap", s));
    REQUIRE(s.shapes.size() == 1u);
    CHECK(has_warning(s, "unknown operators skipped: bar foo zap"));
}

TEST_CASE("pdf: an unbalanced Q is ignored and warned; an unclosed q is harmless") {
    SvgImport s;
    REQUIRE(load("Q Q 0 0 m 1 0 l S q q", s));
    CHECK(s.shapes.size() == 1u);
    CHECK(has_warning(s, "2 Qs with no matching q"));
    SvgImport t;
    REQUIRE(load("0 0 m 1 0 l S EMC", t));
    CHECK(has_warning(t, "EMC"));
}

TEST_CASE("pdf: strings and comments hide operator-like text") {
    SvgImport s;
    REQUIRE(load("BT /F1 12 Tf (S f m l 1 0 0 RG \\) q) Tj ET "
                 "% 0 0 m 5 5 l S\n"
                 "<53 66> Tj [(f) 10 (S)] TJ "
                 "0 0 m 1 0 l S",
                 s));
    CHECK(s.shapes.size() == 1u);
    CHECK(s.shapes[0].aci == 7);
    CHECK(s.filled == 0u);
    CHECK(has_warning(s, "text is not imported"));
    CHECK(!has_warning(s, "unknown"));
}

TEST_CASE("pdf: an inline image is skipped, even with EI-like bytes inside") {
    SvgImport s;
    REQUIRE(load(std::string("q 10 0 0 10 0 0 cm BI /W 2 /H 2 /BPC 8 /CS /G ID ") + "ab EI x\x01\x02" +
                 " EI Q 0 0 m 1 0 l S",
                 s));
    CHECK(s.shapes.size() == 1u);
    CHECK(has_warning(s, "inline image"));
}

TEST_CASE("pdf: malformed operands skip the operator, not the page") {
    SvgImport s;
    REQUIRE(load("5 m 0 0 m 1 0 l S 1 2 3 cm /X w", s));
    CHECK(s.shapes.size() == 1u);
    CHECK(has_warning(s, "bad operands"));
}

TEST_CASE("pdf: a segment with no current point is skipped; l after h starts at the subpath start") {
    SvgImport s;
    REQUIRE(load("5 5 l S", s));
    CHECK(s.shapes.empty());
    CHECK(has_warning(s, "no paths"));
    SvgImport t;
    REQUIRE(load("10 10 m 20 10 l 20 20 l h 30 30 l S", t));
    CHECK(t.shapes.size() == 2u);
    CHECK_NEAR(t.shapes[1].pts[0].x, 10 * kMm, 1e-9);
}

TEST_CASE("pdf: numbers in odd forms, hex strings, dictionaries and names with escapes") {
    SvgImport s;
    REQUIRE(load("+5 -.5 m 1.5. 0 l S  <</K (a ) b)>> BDC EMC  /A#20B gs  0 0 m 1 0 l S", s));
    REQUIRE(s.shapes.size() == 2u);
    CHECK_NEAR(s.shapes[0].pts[0].x, 5 * kMm, 1e-9);
    CHECK_NEAR(s.shapes[0].pts[0].y, -0.5 * kMm, 1e-9);
}

TEST_CASE("pdf: XObjects, shadings and clips are warned, not followed") {
    SvgImport s;
    REQUIRE(load("0 0 10 10 re W n /Fm0 Do /Sh0 sh 0 0 m 1 0 l S", s));
    CHECK(has_warning(s, "XObject"));
    CHECK(has_warning(s, "shading"));
    CHECK(has_warning(s, "clipping"));
}

TEST_CASE("pdf: postscript Illustrator files are refused with a message, with or without qpdf") {
    CHECK(looks_like_postscript("%!PS-Adobe-3.0 EPSF-3.0\n"));
    CHECK(looks_like_postscript("\n  %!PS-Adobe-2.0"));
    CHECK(!looks_like_postscript("%PDF-1.5\n"));

    namespace fs = std::filesystem;
    const fs::path dir = fs::temp_directory_path() / ("nlaser_pdf_" + std::to_string(::getpid()));
    fs::create_directories(dir);
    const fs::path ai = dir / "old.ai";
    std::ofstream(ai, std::ios::binary) << "%!PS-Adobe-3.0\n%%Creator: Adobe Illustrator(R) 8.0\n";
    SvgImport s;
    std::string error;
    CHECK(!import_pdf_file(ai.string(), PdfOptions{}, s, error));
    CHECK(error.find("PostScript") != std::string::npos);

    // Anything else: refused without qpdf, handed to it with.
    const fs::path pdf = dir / "x.pdf";
    std::ofstream(pdf, std::ios::binary) << "%PDF-1.4\n";
    error.clear();
    CHECK(!import_pdf_file(pdf.string(), PdfOptions{}, s, error));
    if (!pdf_container_available()) CHECK(error.find("built without qpdf") != std::string::npos);
    CHECK(!error.empty());

    CHECK(!import_pdf_file((dir / "missing.pdf").string(), PdfOptions{}, s, error));
    CHECK(error.find("cannot read") != std::string::npos);
    std::error_code ec;
    fs::remove_all(dir, ec);
}
