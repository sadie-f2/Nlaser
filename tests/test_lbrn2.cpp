// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// The XML subset reader, and LightBurn's .lbrn2 in and out, against the seven
// samples in tests/fixtures/lightburn (docs/lbrn2-format.md).

#include "test.hpp"

#include "ncad/laser/lbrn2.hpp"
#include "ncad/laser/plan.hpp"
#include "ncad/laser/ruida_codec.hpp"
#include "ncad/laser/ruida_decode.hpp"
#include "ncad/laser/ruida_encode.hpp"
#include "ncad/laser/ruida_job.hpp"
#include "ncad/laser/xml.hpp"

#include <cmath>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

using namespace ncad::laser;

namespace {

std::string slurp(const std::string& rel) {
    std::ifstream in(std::string(NLASER_FIXTURES) + "/lightburn/" + rel, std::ios::binary);
    if (!in) {
        ::nototest::fail(__FILE__, __LINE__, "fixture missing: " + rel);
        return {};
    }
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

Lbrn2Options top_right(StartMode start = StartMode::Current, JobOrigin jo = JobOrigin::TopLeft) {
    Lbrn2Options o;
    o.machine.origin = ruida::MachineCorner::TopRight;
    o.start = start;
    o.job_origin = jo;
    return o;
}

bool parse_x(const std::string& s, xml::Node& n, std::string& err) { return xml::parse(s, n, err); }

bool rejects(const std::string& s, const std::string& expect) {
    xml::Node n;
    std::string err;
    if (parse_x(s, n, err)) return false;
    return err.find(expect) != std::string::npos;
}

// A minimal project around `shapes`, one layer index 2.
std::string project(const std::string& shapes, const std::string& attrs = "FormatVersion=\"1\"") {
    return "<LightBurnProject " + attrs + ">\n<CutSetting type=\"Cut\"><index Value=\"2\"/><speed Value=\"50\"/>"
           "<minPower Value=\"10\"/><maxPower Value=\"20\"/></CutSetting>\n" + shapes + "</LightBurnProject>";
}

bool read_err(const std::string& text, std::string& err) {
    LaserJob j;
    Lbrn2Info i;
    return !read_lbrn2(text, j, i, Lbrn2Options{}, err);
}

}  // namespace

// --- XML -----------------------------------------------------------------------

TEST_CASE("xml: elements, attributes, text, entities, comments, self-closing tags") {
    xml::Node r;
    std::string err;
    REQUIRE(parse_x("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<!-- c -->\n"
                    "<a x='1' y=\"a&amp;b &lt;&gt; &quot;&apos; &#65;&#x42;\">\n  <b/>\n  <c k=\"v\"> t &amp; u <!-- in --> </c>\n</a>\n",
                    r, err));
    CHECK(r.name == "a");
    CHECK(r.line == 3);
    REQUIRE(r.attr("x") != nullptr);
    CHECK(*r.attr("x") == "1");
    CHECK(*r.attr("y") == "a&b <> \"' AB");
    CHECK(r.attr("z") == nullptr);
    REQUIRE(r.children.size() == 2);
    CHECK(r.children[0].name == "b");
    CHECK(r.children[0].line == 4);
    CHECK(r.child("c") != nullptr);
    CHECK(r.child("c")->text == "t & u");
    CHECK(r.child("zz") == nullptr);
}

TEST_CASE("xml: numeric references become UTF-8") {
    xml::Node r;
    std::string err;
    REQUIRE(parse_x("<a>&#xE9;&#8364;</a>", r, err));
    CHECK(r.text == "\xC3\xA9\xE2\x82\xAC");
}

TEST_CASE("xml: what is outside the subset is refused with a line number") {
    CHECK(rejects("<a>\n<b>\n</a>", "line 3: </a> closes <b> (opened on line 2)"));
    CHECK(rejects("<a>\n<b/>", "<a> (line 1) is not closed"));
    CHECK(rejects("<!DOCTYPE a>\n<a/>", "line 1"));
    CHECK(rejects("<a>\n<![CDATA[x]]></a>", "line 2: CDATA"));
    CHECK(rejects("<a>\n\n<?php x ?></a>", "line 3: processing"));
    CHECK(rejects("<a x=\"1\" x=\"2\"/>", "given twice"));
    CHECK(rejects("<a x=1/>", "quoted"));
    CHECK(rejects("<a>&nbsp;</a>", "unknown entity"));
    CHECK(rejects("<a>&#0;</a>", "U+0000"));
    CHECK(rejects("<a>&#x110000;</a>", "beyond"));
    CHECK(rejects("<a>&amp</a>", "';'"));
    CHECK(rejects("<a/>\n<b/>", "line 2: content after the root"));
    CHECK(rejects("<a/> x", "content after the root"));
    CHECK(rejects("", "no root"));
    CHECK(rejects("text", "root element"));
    CHECK(rejects("<a x=\"<\"/>", "'<' inside"));
    CHECK(rejects("<a><!-- open </a>", "comment"));
    CHECK(rejects("<a", "not closed"));
    std::string deep;
    for (int i = 0; i < 300; ++i) deep += "<a>";
    CHECK(rejects(deep, "nested"));
}

TEST_CASE("xml: a failed parse leaves the output alone") {
    xml::Node r;
    r.name = "keep";
    std::string err;
    CHECK(!parse_x("<a>", r, err));
    CHECK(r.name == "keep");
}

TEST_CASE("xml: escape round-trips through parse") {
    xml::Node r;
    std::string err;
    const std::string raw = "a&b<c>\"d'e";
    REQUIRE(parse_x("<a v=\"" + xml::escape(raw) + "\"/>", r, err));
    CHECK(*r.attr("v") == raw);
}

TEST_CASE("xml: every sample parses") {
    for (const char* f : {"f01", "f02-p2", "f02-p3", "f03", "f05", "f06-BR", "f07"}) {
        xml::Node r;
        std::string err;
        REQUIRE(parse_x(slurp(std::string(f) + ".lbrn2"), r, err));
        CHECK(r.name == "LightBurnProject");
    }
}

// --- reading the samples -------------------------------------------------------

TEST_CASE("lbrn2: layer settings of every sample") {
    struct Want { const char* name; int passes; };
    for (const Want w : {Want{"f01", 1}, Want{"f02-p2", 2}, Want{"f02-p3", 3}, Want{"f03", 2}, Want{"f05", 2},
                         Want{"f06-BR", 2}, Want{"f07", 2}}) {
        LaserJob job;
        Lbrn2Info info;
        std::string err;
        REQUIRE(read_lbrn2(slurp(std::string(w.name) + ".lbrn2"), job, info, top_right(), err));
        REQUIRE(job.layers.size() == 1);
        const LaserLayer& l = job.layers[0];
        CHECK(l.pen == 2);
        CHECK(l.color == 1);  // the .rd's "Layer Color #FF0000"
        CHECK(l.mode == LayerMode::Vector);
        CHECK_NEAR(l.speed, 100.0, 1e-9);
        CHECK_NEAR(l.min_power, 15.0, 1e-9);
        CHECK_NEAR(l.max_power, 15.0, 1e-9);
        CHECK(l.has_power2);
        CHECK_NEAR(l.min_power2, 10.0, 1e-9);
        CHECK_NEAR(l.max_power2, 20.0, 1e-9);
        CHECK(l.passes == w.passes);
        CHECK(l.air);
        CHECK(info.app_version == "1.4.03");
        CHECK(info.format_version == 1);
        CHECK(info.mirror_x);
        CHECK(info.mirror_y);
        CHECK(info.warnings.empty());
        REQUIRE(info.layer_names.size() == 1);
        CHECK(info.layer_names[0].second == "Mark");
        CHECK(!info.placement_from_file);
    }
}

TEST_CASE("lbrn2: f01 is the 40 mm square, mirrored into the design frame") {
    LaserJob job;
    Lbrn2Info info;
    std::string err;
    REQUIRE(read_lbrn2(slurp("f01.lbrn2"), job, info, top_right(), err));
    REQUIRE(job.paths.size() == 1);
    const LaserPath& p = job.paths[0];
    CHECK(p.closed);
    CHECK(p.pen == 2);
    REQUIRE(p.pts.size() == 4);
    // Workspace 517.27362..557.27362 / 261.70001..301.70001; design negates both.
    CHECK_NEAR(info.workspace_min.x, 517.27362, 1e-6);
    CHECK_NEAR(info.workspace_max.y, 301.70001, 1e-6);
    CHECK_NEAR(p.pts[0].x, -557.27362, 1e-6);
    CHECK_NEAR(p.pts[0].y, -301.70001, 1e-6);
    CHECK_NEAR(p.pts[2].x, -517.27362, 1e-6);
    CHECK_NEAR(p.pts[2].y, -261.70001, 1e-6);
    // Job origin top-left of the design box.
    CHECK_NEAR(job.origin.x, -557.27362, 1e-6);
    CHECK_NEAR(job.origin.y, -261.70001, 1e-6);
    CHECK(job.start == StartMode::Current);
}

TEST_CASE("lbrn2: with the bed known the design frame is bed coordinates (f03)") {
    Lbrn2Options o = top_right(StartMode::Absolute);
    o.machine.bed_w_mm = 900;
    o.machine.bed_h_mm = 600;
    LaserJob job;
    Lbrn2Info info;
    std::string err;
    REQUIRE(read_lbrn2(slurp("f03.lbrn2"), job, info, o, err));
    // docs/lightburn-fixtures.md: bed x 638.334..678.334, y 348.976..388.976.
    double x0 = 1e9, x1 = -1e9, y0 = 1e9, y1 = -1e9;
    for (const Pt& q : job.paths[0].pts) {
        x0 = std::min(x0, q.x);
        x1 = std::max(x1, q.x);
        y0 = std::min(y0, q.y);
        y1 = std::max(y1, q.y);
    }
    CHECK_NEAR(x0, 638.33389, 1e-4);
    CHECK_NEAR(x1, 678.33389, 1e-4);
    CHECK_NEAR(y0, 348.9756, 1e-4);
    CHECK_NEAR(y1, 388.9756, 1e-4);
    CHECK(job.start == StartMode::Absolute);
    CHECK_NEAR(job.bed_w, 900.0, 1e-9);
    // The head is assumed at machine zero, the bed's top-right.
    CHECK_NEAR(job.origin.x, 900.0, 1e-9);
    CHECK_NEAR(job.origin.y, 600.0, 1e-9);
}

TEST_CASE("lbrn2: f07's ellipses flatten to the segment counts LightBurn's .rd shows") {
    LaserJob job;
    Lbrn2Info info;
    std::string err;
    REQUIRE(read_lbrn2(slurp("f07.lbrn2"), job, info, top_right(), err));
    REQUIRE(job.paths.size() == 4);
    const std::size_t want[4] = {32, 64, 128, 256};
    const double radius[4] = {1, 5, 25, 100};
    for (std::size_t i = 0; i < 4; ++i) {
        const LaserPath& p = job.paths[i];
        CHECK(p.closed);
        CHECK(p.pts.size() == want[i]);
        double cx = 0, cy = 0;
        for (const Pt& q : p.pts) {
            cx += q.x;
            cy += q.y;
        }
        cx /= static_cast<double>(p.pts.size());
        cy /= static_cast<double>(p.pts.size());
        for (const Pt& q : p.pts) CHECK_NEAR(std::hypot(q.x - cx, q.y - cy), radius[i], 1e-6);
    }
}

TEST_CASE("lbrn2: the samples through plan and encode reproduce the .rd byte for byte") {
    struct Want { const char* name; StartMode start; JobOrigin jo; };
    for (const Want w : {Want{"f01", StartMode::Current, JobOrigin::TopLeft},
                         Want{"f02-p2", StartMode::Current, JobOrigin::TopLeft},
                         Want{"f02-p3", StartMode::Current, JobOrigin::TopLeft},
                         Want{"f05", StartMode::Current, JobOrigin::Centre},
                         Want{"f06-BR", StartMode::Current, JobOrigin::BottomRight},
                         Want{"f03", StartMode::Absolute, JobOrigin::TopLeft}}) {
        Lbrn2Options o = top_right(w.start, w.jo);
        if (w.start == StartMode::Absolute) {
            o.machine.bed_w_mm = 900;
            o.machine.bed_h_mm = 600;
        }
        LaserJob raw, planned;
        Lbrn2Info info;
        PlanReport rep;
        std::string err;
        REQUIRE(read_lbrn2(slurp(std::string(w.name) + ".lbrn2"), raw, info, o, err));
        REQUIRE(plan_job(raw, planned, PlanOptions{}, rep, err));
        ruida::RdJob rd;
        REQUIRE(job_to_rd(planned, rd, err));
        rd.machine = o.machine;
        rd.pointer_offset = {5.2, 31.2};
        std::string plain;
        REQUIRE(encode_rd(rd, plain, err));
        const std::string file = slurp(std::string(w.name) + ".rd");
        const std::string want = ruida::Swizzle(0x88).decode(file);
        if (plain != want) ::nototest::fail(__FILE__, __LINE__, std::string(w.name) + ": encoded bytes differ from the .rd");
    }
}

TEST_CASE("lbrn2: f07 through plan and encode matches in structure, not in start points") {
    LaserJob raw, planned;
    Lbrn2Info info;
    PlanReport rep;
    std::string err;
    Lbrn2Options o = top_right();
    REQUIRE(read_lbrn2(slurp("f07.lbrn2"), raw, info, o, err));
    REQUIRE(plan_job(raw, planned, PlanOptions{}, rep, err));
    ruida::RdJob rd;
    REQUIRE(job_to_rd(planned, rd, err));
    rd.machine = o.machine;
    rd.pointer_offset = {5.2, 31.2};
    std::string plain;
    REQUIRE(encode_rd(rd, plain, err));
    const ruida::RdSummary ours = ruida::summarize_rd(ruida::decode_rd(plain));
    const std::string want_plain = ruida::Swizzle(0x88).decode(slurp("f07.rd"));
    const ruida::RdSummary theirs = ruida::summarize_rd(ruida::decode_rd(want_plain));
    CHECK(ours.cuts == theirs.cuts);
    CHECK(ours.moves == theirs.moves);
    // LightBurn's vertices are not evenly spaced, so its length is 0.014 % longer.
    CHECK_NEAR(ours.cut_mm, theirs.cut_mm, 0.5);
    CHECK(rd.layers.size() == 1);
}

// --- reading: refusals ---------------------------------------------------------

TEST_CASE("lbrn2: what the samples do not show is refused by name and line") {
    std::string err;
    const std::string xf = "<XForm>1 0 0 1 0 0</XForm>";
    CHECK(read_err(project("<Shape Type=\"Rect\" CutIndex=\"2\" W=\"1\" H=\"1\">" + xf + "</Shape>\n"), err));
    CHECK(err.find("line 3") != std::string::npos);
    CHECK(err.find("Rect") != std::string::npos);
    CHECK(read_err(project("<Shape Type=\"Group\" CutIndex=\"2\">" + xf + "</Shape>\n"), err));
    CHECK(err.find("Group") != std::string::npos);
    CHECK(read_err(project("<Shape Type=\"Ellipse\" CutIndex=\"2\" Rx=\"1\" Ry=\"1\"></Shape>\n"), err));
    CHECK(err.find("XForm") != std::string::npos);
    CHECK(read_err(project("<Shape Type=\"Ellipse\" CutIndex=\"5\" Rx=\"1\" Ry=\"1\">" + xf + "</Shape>\n"), err));
    CHECK(err.find("layer 5") != std::string::npos);
    CHECK(read_err(project("", "FormatVersion=\"2\""), err));
    CHECK(err.find("FormatVersion") != std::string::npos);
    CHECK(read_err("<Other/>", err));
    CHECK(read_err("<LightBurnProject FormatVersion=\"1\"><CutSetting type=\"Scan\"><index Value=\"0\"/></CutSetting></LightBurnProject>", err));
    CHECK(err.find("Scan") != std::string::npos);
    CHECK(read_err(project("<Shape Type=\"Path\" CutIndex=\"2\">" + xf +
                           "<VertList>V0 0V1 0</VertList><PrimList>Spline</PrimList></Shape>\n"), err));
    CHECK(err.find("PrimList") != std::string::npos);
    CHECK(read_err(project("<Shape Type=\"Path\" CutIndex=\"2\">" + xf +
                           "<VertList>V0 0V1</VertList><PrimList>LineClosed</PrimList></Shape>\n"), err));
    CHECK(err.find("VertList") != std::string::npos);
    CHECK(read_err("<LightBurnProject FormatVersion=\"1\"><CutSetting type=\"Cut\"><index Value=\"1\"/>"
                   "<speed Value=\"1\"/><minPower Value=\"1\"/><maxPower Value=\"101\"/></CutSetting></LightBurnProject>", err));
    CHECK(err.find("power") != std::string::npos);
}

TEST_CASE("lbrn2: unknown project elements are reported, not silently dropped") {
    LaserJob job;
    Lbrn2Info info;
    std::string err;
    REQUIRE(read_lbrn2("<LightBurnProject FormatVersion=\"1\"><Mystery/></LightBurnProject>", job, info, Lbrn2Options{}, err));
    REQUIRE(info.warnings.size() == 1);
    CHECK(info.warnings[0].find("Mystery") != std::string::npos);
}

TEST_CASE("lbrn2: a Bézier primitive is flattened to the tolerance (primitive spelling unverified)") {
    const std::string xf = "<XForm>1 0 0 1 0 0</XForm>";
    const std::string text = project("<Shape Type=\"Path\" CutIndex=\"2\">" + xf +
                                     "<VertList>V0 0c0x0c0y10c1x0c1y0V10 0c0x0c0y0c1x10c1y10</VertList>"
                                     "<PrimList>B0 1</PrimList></Shape>\n");
    LaserJob job;
    Lbrn2Info info;
    std::string err;
    Lbrn2Options o;
    o.tolerance = 0.01;
    REQUIRE(read_lbrn2(text, job, info, o, err));
    REQUIRE(job.paths.size() == 1);
    const LaserPath& p = job.paths[0];
    CHECK(!p.closed);
    CHECK(p.pts.size() > 8);
    CHECK_NEAR(p.pts.front().x, 0.0, 1e-9);
    CHECK_NEAR(p.pts.back().x, 10.0, 1e-9);
    // The curve peaks at (5, 7.5); no vertex may be above it, and some are near it.
    double top = 0;
    for (const Pt& q : p.pts) top = std::max(top, q.y);
    CHECK(top <= 7.5 + 1e-9);
    CHECK(top > 7.5 - 0.1);
    // A finer tolerance gives more points.
    Lbrn2Options fine = o;
    fine.tolerance = 0.001;
    LaserJob job2;
    REQUIRE(read_lbrn2(text, job2, info, fine, err));
    CHECK(job2.paths[0].pts.size() > p.pts.size());
    // A Bézier without its control points is refused.
    CHECK(read_err(project("<Shape Type=\"Path\" CutIndex=\"2\">" + xf +
                           "<VertList>V0 0V10 0</VertList><PrimList>B0 1</PrimList></Shape>\n"), err));
}

TEST_CASE("lbrn2: explicit L primitives make open paths and chains") {
    const std::string xf = "<XForm>1 0 0 1 100 0</XForm>";
    const std::string text = project("<Shape Type=\"Path\" CutIndex=\"2\">" + xf +
                                     "<VertList>V0 0V1 0V1 1V5 5V6 5</VertList>"
                                     "<PrimList>L0 1L1 2L3 4</PrimList></Shape>\n");
    LaserJob job;
    Lbrn2Info info;
    std::string err;
    REQUIRE(read_lbrn2(text, job, info, Lbrn2Options{}, err));
    REQUIRE(job.paths.size() == 2);
    CHECK(!job.paths[0].closed);
    CHECK(job.paths[0].pts.size() == 3);
    CHECK_NEAR(job.paths[0].pts[0].x, 100.0, 1e-9);
    CHECK(job.paths[1].pts.size() == 2);
}

// --- writing -------------------------------------------------------------------

namespace {

LaserJob sample_job() {
    LaserJob j;
    j.start = StartMode::Current;
    LaserLayer a;
    a.pen = 2;
    a.color = 1;
    a.mode = LayerMode::Vector;
    a.speed = 150.5;
    a.min_power = 12;
    a.max_power = 18.5;
    a.passes = 3;
    a.order = 1;
    LaserLayer b;
    b.pen = 5;
    b.color = 7;
    b.mode = LayerMode::Vector;
    b.speed = 20;
    b.min_power = 60;
    b.max_power = 65;
    b.has_power2 = true;
    b.min_power2 = 30;
    b.max_power2 = 35;
    j.layers = {a, b};
    j.paths.push_back({2, true, {{10, 10}, {90, 10}, {90, 40.25}, {10, 40.25}}});
    j.paths.push_back({5, false, {{20, 20}, {30, 25.5}, {40.125, 20}}});
    j.paths.push_back({2, true, {{50, 20}, {60, 20}, {55, 30}}});
    j.origin = {10, 10};
    return j;
}

void same_job(const LaserJob& a, const LaserJob& b) {
    REQUIRE(a.layers.size() == b.layers.size());
    for (std::size_t i = 0; i < a.layers.size(); ++i) {
        const LaserLayer &x = a.layers[i], &y = b.layers[i];
        CHECK(x.pen == y.pen);
        CHECK(x.color == y.color);
        CHECK(x.mode == y.mode);
        CHECK_NEAR(x.speed, y.speed, 1e-9);
        CHECK_NEAR(x.min_power, y.min_power, 1e-9);
        CHECK_NEAR(x.max_power, y.max_power, 1e-9);
        CHECK(x.has_power2 == y.has_power2);
        if (x.has_power2) {
            CHECK_NEAR(x.min_power2, y.min_power2, 1e-9);
            CHECK_NEAR(x.max_power2, y.max_power2, 1e-9);
        }
        CHECK(x.passes == y.passes);
        CHECK(x.air == y.air);
        CHECK(x.order == y.order);
    }
    REQUIRE(a.paths.size() == b.paths.size());
    for (std::size_t i = 0; i < a.paths.size(); ++i) {
        CHECK(a.paths[i].pen == b.paths[i].pen);
        CHECK(a.paths[i].closed == b.paths[i].closed);
        REQUIRE(a.paths[i].pts.size() == b.paths[i].pts.size());
        for (std::size_t k = 0; k < a.paths[i].pts.size(); ++k) {
            CHECK_NEAR(a.paths[i].pts[k].x, b.paths[i].pts[k].x, 1e-5);
            CHECK_NEAR(a.paths[i].pts[k].y, b.paths[i].pts[k].y, 1e-5);
        }
    }
}

}  // namespace

TEST_CASE("lbrn2: write then read gives the same job, for each machine corner") {
    using ruida::MachineCorner;
    for (const MachineCorner mc : {MachineCorner::BottomLeft, MachineCorner::BottomRight, MachineCorner::TopLeft,
                                   MachineCorner::TopRight}) {
        for (const bool bed : {false, true}) {
            Lbrn2Options o;
            o.machine.origin = mc;
            if (bed) {
                o.machine.bed_w_mm = 900;
                o.machine.bed_h_mm = 600;
            }
            const LaserJob in = sample_job();
            std::string text, err;
            REQUIRE(write_lbrn2(in, o, text, err));
            LaserJob out;
            Lbrn2Info info;
            REQUIRE(read_lbrn2(text, out, info, o, err));
            CHECK(info.warnings.empty());
            same_job(in, out);
            CHECK(info.mirror_x == (o.machine.sign_x() < 0));
        }
    }
}

TEST_CASE("lbrn2: a sample read, written and read again is unchanged, names included") {
    for (const char* f : {"f01", "f02-p3", "f06-BR"}) {
        const Lbrn2Options o = top_right();
        LaserJob a, b;
        Lbrn2Info ia, ib;
        std::string err, text;
        REQUIRE(read_lbrn2(slurp(std::string(f) + ".lbrn2"), a, ia, o, err));
        REQUIRE(write_lbrn2(a, o, text, err, &ia));
        REQUIRE(read_lbrn2(text, b, ib, o, err));
        same_job(a, b);
        REQUIRE(ib.layer_names.size() == 1);
        CHECK(ib.layer_names[0].second == "Mark");
    }
}

TEST_CASE("lbrn2: the written file has the samples' structure") {
    // Same element order and attributes as f01 (one pass) and f02-p2 (numPasses).
    for (const char* f : {"f01", "f02-p2"}) {
        const Lbrn2Options o = top_right();
        LaserJob job;
        Lbrn2Info info;
        std::string err, text;
        REQUIRE(read_lbrn2(slurp(std::string(f) + ".lbrn2"), job, info, o, err));
        REQUIRE(write_lbrn2(job, o, text, err, &info));
        xml::Node mine, theirs;
        REQUIRE(parse_x(text, mine, err));
        REQUIRE(parse_x(slurp(std::string(f) + ".lbrn2"), theirs, err));
        // Root attributes, and the sequence of root children.
        CHECK(mine.attrs == theirs.attrs);
        REQUIRE(mine.children.size() == theirs.children.size());
        for (std::size_t i = 0; i < mine.children.size(); ++i) {
            const xml::Node &m = mine.children[i], &t = theirs.children[i];
            CHECK(m.name == t.name);
            CHECK(m.attrs.size() == t.attrs.size());
            CHECK(m.children.size() == t.children.size());
            for (std::size_t k = 0; k < m.children.size() && k < t.children.size(); ++k) {
                CHECK(m.children[k].name == t.children[k].name);
                // Settings carry the same values; the shape's own encoding may differ.
                if (m.name == "CutSetting" || m.name == "UIPrefs" || m.name == "VariableText") {
                    CHECK(m.children[k].attrs == t.children[k].attrs);
                }
            }
            if (m.name == "Shape") {
                CHECK(*m.attr("Type") == *t.attr("Type"));
                CHECK(*m.attr("CutIndex") == *t.attr("CutIndex"));
                CHECK(m.child("PrimList")->text == t.child("PrimList")->text);
            }
        }
    }
}

TEST_CASE("lbrn2: what cannot be spelled from the samples is refused on write") {
    const Lbrn2Options o;
    std::string text, err;
    LaserJob j = sample_job();
    j.layers[0].mode = LayerMode::Skip;
    CHECK(!write_lbrn2(j, o, text, err));
    CHECK(err.find("skip") != std::string::npos);
    j = sample_job();
    j.layers[0].air = false;
    CHECK(!write_lbrn2(j, o, text, err));
    CHECK(err.find("air") != std::string::npos);
    j = sample_job();
    j.layers[0].mode = LayerMode::Raster;
    CHECK(!write_lbrn2(j, o, text, err));
    j = sample_job();
    j.paths.push_back({9, true, {{0, 0}, {1, 0}, {1, 1}}});
    CHECK(!write_lbrn2(j, o, text, err));
    CHECK(err.find("pen 9") != std::string::npos);
}

TEST_CASE("lbrn2: layer names are escaped") {
    LaserJob j = sample_job();
    Lbrn2Info names;
    names.layer_names.emplace_back(2, "Cut & \"score\" <1>");
    std::string text, err;
    REQUIRE(write_lbrn2(j, Lbrn2Options{}, text, err, &names));
    LaserJob out;
    Lbrn2Info info;
    REQUIRE(read_lbrn2(text, out, info, Lbrn2Options{}, err));
    CHECK(info.layer_names[0].second == "Cut & \"score\" <1>");
}
