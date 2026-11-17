// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// LBRN2IN, LASER Save .lbrn2 and Pens Load .lbrn2, through NotoCAD's command
// engine, against the seven LightBurn samples (tests/fixtures/lightburn) and,
// where the DXFs they came from are at hand, those DXFs' geometry.

#include "test.hpp"

#include "ncad/command.hpp"
#include "ncad/commands.hpp"
#include "ncad/database.hpp"
#include "ncad/entities.hpp"
#include "ncad/input_text.hpp"
#include "ncad/laser/lbrn2.hpp"
#include "ncad/laser/laser_command.hpp"
#include "ncad/laser/laser_renderer.hpp"
#include "ncad/laser/ruida_codec.hpp"
#include "ncad/laser/ruida_decode.hpp"
#include "ncad/sysvar.hpp"

#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

using namespace ncad::laser;
using ncad::CommandEngine;
using ncad::Database;
using ncad::EngineStatus;
using ncad::EntityType;

namespace {

namespace fs = std::filesystem;

// Where the user's DXFs (the sources of the samples) live; absent on any other
// machine, in which case those comparisons are skipped, not failed.
constexpr const char* kDxfDir = "/Users/sadie/src/Nlaser/build/lightburn-fixtures/";

struct Fixture {
    Database db;
    CommandEngine engine{db};
    std::shared_ptr<LaserSession> session;
    fs::path dir;
    std::string transcript;

    explicit Fixture(const std::string& conf = "") {
        static int serial = 0;
        dir = fs::temp_directory_path() / ("nlaser_lbrn2in_" + std::to_string(::getpid()) + "_" + std::to_string(++serial));
        fs::create_directories(dir);
        LaserOptions opts;
        if (!conf.empty()) {
            std::ofstream(dir / "lasers.conf", std::ios::binary) << conf;
            opts.config_path = (dir / "lasers.conf").string();
        } else {
            // A path that does not exist: no devices, whatever the machine has.
            opts.config_path = (dir / "none.conf").string();
        }
        session = install_laser(opts);
    }
    ~Fixture() {
        ncad::clear_registered_commands();
        ncad::clear_registered_sysvars();
        std::error_code ec;
        fs::remove_all(dir, ec);
    }

    std::string path(const char* name) const { return (dir / name).string(); }
    static std::string sample(const std::string& name) {
        return std::string(NLASER_FIXTURES) + "/lightburn/" + name;
    }
    std::string read(const std::string& p) const {
        std::ifstream in(p, std::ios::binary);
        return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    }

    // Records each prompt before answering it: LASER reports through the next
    // prompt, so that is where a keyword's output is.
    struct Recording final : ncad::InputSource {
        ncad::TextInputSource inner;
        std::string& log;
        Recording(std::vector<std::string> t, std::string& l) : inner(std::move(t)), log(l) {}
        bool next_value(const ncad::Prompt& p, ncad::InputValue& out) override {
            log += p.message + "\n";
            return inner.next_value(p, out);
        }
    };

    EngineStatus run(const char* command, std::vector<std::string> tokens) {
        transcript.clear();
        Recording src(std::move(tokens), transcript);
        src.inner.set_database(&db);
        engine.begin(ncad::make_command(command));
        const EngineStatus st = engine.run(src);
        transcript += engine.message();
        return st;
    }
    EngineStatus import(const std::string& file) { return run("LBRN2IN", {file}); }
    // LASER, with Enters at the end that leave a sub-mode and then the mode.
    EngineStatus laser(std::vector<std::string> tokens) {
        tokens.insert(tokens.end(), 4, std::string());
        return run("LASER", std::move(tokens));
    }

    std::vector<const ncad::Entity*> entities(EntityType t) const {
        std::vector<const ncad::Entity*> out;
        for (const ncad::Handle h : db.order()) {
            if (db.get(h)->type() == t) out.push_back(db.get(h));
        }
        return out;
    }
};

bool contains(const std::string& s, const std::string& part) { return s.find(part) != std::string::npos; }

std::vector<Pt> vertices_of(const ncad::Polyline& pl) {
    std::vector<Pt> out;
    for (const auto& v : pl.vertices()) out.push_back({v.pos.x, v.pos.y});
    return out;
}

// All polyline vertices of the drawing, moved so the bounding box starts at 0,0.
std::vector<Pt> normalised(std::vector<Pt> pts) {
    if (pts.empty()) return pts;
    double mx = pts[0].x, my = pts[0].y;
    for (const Pt& p : pts) {
        mx = std::min(mx, p.x);
        my = std::min(my, p.y);
    }
    for (Pt& p : pts) {
        p.x -= mx;
        p.y -= my;
    }
    return pts;
}

// Every point of `a` has a point of `b` within `tol`, and the other way round.
bool same_points(const std::vector<Pt>& a, const std::vector<Pt>& b, double tol) {
    auto covered = [&](const std::vector<Pt>& x, const std::vector<Pt>& y) {
        for (const Pt& p : x) {
            bool hit = false;
            for (const Pt& q : y) hit = hit || std::hypot(p.x - q.x, p.y - q.y) <= tol;
            if (!hit) return false;
        }
        return true;
    };
    return covered(a, b) && covered(b, a);
}

// The vertices of the DXF's paths, or false when the DXF is not on this machine.
bool dxf_points(const char* name, std::vector<Pt>& pts, std::size_t& paths) {
    const std::string p = std::string(kDxfDir) + name;
    if (!fs::exists(p)) return false;
    LaserJob job;
    DrawingReport rep;
    std::string err;
    if (!dxf_to_job(p, DrawingOptions{}, {}, job, rep, err)) {
        ::nototest::fail(__FILE__, __LINE__, "cannot read " + p + ": " + err);
        return false;
    }
    paths = job.paths.size();
    pts.clear();
    for (const LaserPath& lp : job.paths) pts.insert(pts.end(), lp.pts.begin(), lp.pts.end());
    return true;
}

// The settings lines of an .rd's header, as text: speed, powers and colour of
// each part. Two files that agree on these cut each layer alike.
std::string layer_settings(const std::string& rd_file) {
    ruida::RdTextOptions o;
    o.offsets = o.hex = o.positions = o.summary = false;
    const std::string text = ruida::rd_to_text(ruida::Swizzle(0x88).decode(rd_file), o);
    std::string out;
    std::size_t at = 0;
    while (at < text.size()) {
        std::size_t nl = text.find('\n', at);
        if (nl == std::string::npos) nl = text.size();
        const std::string line = text.substr(at, nl - at);
        if (line.find(" Part ") != std::string::npos &&
            (line.find("Speed Laser 1") != std::string::npos || line.find("Power") != std::string::npos ||
             line.find("Layer Color") != std::string::npos)) {
            out += line + "\n";
        }
        at = nl + 1;
    }
    return out;
}

ruida::RdSummary summary_of(const std::string& rd_file) {
    return ruida::summarize_rd(ruida::decode_rd(ruida::Swizzle(0x88).decode(rd_file)));
}

struct Sample {
    const char* lbrn2;
    const char* rd;
    const char* dxf;  // its source, or null
    std::size_t polylines;
    std::size_t circles;
};

const Sample kSamples[] = {
    {"f01.lbrn2", "f01.rd", "F01-baseline.dxf", 1, 0},
    {"f02-p2.lbrn2", "f02-p2.rd", "F02-passes.dxf", 1, 0},
    {"f02-p3.lbrn2", "f02-p3.rd", "F02-passes.dxf", 1, 0},
    {"f03.lbrn2", "f03.rd", "F03-start-absolute.dxf", 1, 0},
    {"f05.lbrn2", "f05.rd", "F05-origin-centre.dxf", 1, 0},
    {"f06-BR.lbrn2", "f06-BR.rd", "F06-origin-corners.dxf", 1, 0},
    {"f07.lbrn2", "f07.rd", nullptr, 0, 4},
};

}  // namespace

TEST_CASE("lbrn2in: registered, LBRN2CORNER defaults to tr") {
    Fixture f;
    REQUIRE(f.session != nullptr);
    CHECK(ncad::make_command("LBRN2IN") != nullptr);
    CHECK(f.db.sysvars().get_string(f.session->lbrn2_corner) == "tr");
}

TEST_CASE("lbrn2in: every sample imports with the expected entities, layer and one undo step") {
    for (const Sample& smp : kSamples) {
        Fixture f;
        REQUIRE(f.import(Fixture::sample(smp.lbrn2)) == EngineStatus::Finished);
        CHECK(f.entities(EntityType::Polyline).size() == smp.polylines);
        CHECK(f.entities(EntityType::Circle).size() == smp.circles);
        CHECK(f.entities(EntityType::Ellipse).empty());

        // One layer, "Mark" (the name in every sample's CutSetting), red as LightBurn's layer 2.
        const ncad::LayerId lay = f.db.find_layer("Mark");
        REQUIRE(lay != ncad::kInvalidLayer);
        CHECK(f.db.layer(lay).color == 1);
        for (const ncad::Handle h : f.db.order()) CHECK(f.db.get(h)->props().layer == lay);

        // The CutSettings are the LASER session's pens, keyed by the ACI.
        REQUIRE(f.session->has_imported_pens);
        REQUIRE(f.session->imported_pens.size() == 1);
        CHECK(f.session->imported_pens[0].pen == 1);
        CHECK_NEAR(f.session->imported_pens[0].speed, 100.0, 1e-9);

        CHECK(f.db.journal().undo_depth() == 1);
        CHECK(f.db.journal().undo(f.db));
        CHECK(f.db.empty());
    }
}

TEST_CASE("lbrn2in: polylines match the source DXF after the corner conversion") {
    // Placement in the file is wherever the object sat on LightBurn's canvas,
    // so shape is compared from each one's own bounding-box corner. With the
    // top-right machine the conversion negates both axes; LightBurn's own
    // XForm (-1 0 0 -1) is the same turn, so the square comes back as drawn.
    for (const Sample& smp : kSamples) {
        if (!smp.dxf) continue;
        std::vector<Pt> want;
        std::size_t paths = 0;
        if (!dxf_points(smp.dxf, want, paths)) continue;
        Fixture f;
        REQUIRE(f.import(Fixture::sample(smp.lbrn2)) == EngineStatus::Finished);
        std::vector<Pt> got;
        for (const ncad::Entity* e : f.entities(EntityType::Polyline)) {
            const auto v = vertices_of(*static_cast<const ncad::Polyline*>(e));
            got.insert(got.end(), v.begin(), v.end());
        }
        CHECK(paths == 1);
        CHECK(same_points(normalised(got), normalised(want), 0.01));
    }
}

TEST_CASE("lbrn2in: f07 makes four circles, matching the source DXF's centres and radii") {
    Fixture f;
    REQUIRE(f.import(Fixture::sample("f07.lbrn2")) == EngineStatus::Finished);
    const std::vector<const ncad::Entity*> cs = f.entities(EntityType::Circle);
    REQUIRE(cs.size() == 4);
    std::vector<const ncad::Circle*> c;
    for (const ncad::Entity* e : cs) c.push_back(static_cast<const ncad::Circle*>(e));
    std::sort(c.begin(), c.end(), [](const ncad::Circle* a, const ncad::Circle* b) { return a->radius() < b->radius(); });
    // notes.txt: diameters 2, 10, 50, 200 centred at (1,100), (17,100), (57,100), (192,100).
    const double radius[4] = {1, 5, 25, 100};
    const double cx[4] = {1, 17, 57, 192};
    for (std::size_t i = 0; i < 4; ++i) {
        CHECK_NEAR(c[i]->radius(), radius[i], 1e-6);
        CHECK_NEAR(c[i]->center().x - c[0]->center().x, cx[i] - cx[0], 0.01);
        CHECK_NEAR(c[i]->center().y - c[0]->center().y, 0.0, 0.01);
    }
}

TEST_CASE("lbrn2in: a bed gives bed coordinates, the corner option changes the frame") {
    // f03's square is at workspace 221.666..261.666 x 211.024..251.024
    // (docs/lbrn2-format.md), which on a 900 x 600 bed counted from the top
    // right is bed x 638.334..678.334, y 348.976..388.976.
    Fixture f("[bench]\nhost = 192.0.2.1\nbed = 900 600\n");
    REQUIRE(f.import(Fixture::sample("f03.lbrn2")) == EngineStatus::Finished);
    const auto pls = f.entities(EntityType::Polyline);
    REQUIRE(pls.size() == 1);
    double lx = 1e9, ly = 1e9, hx = -1e9, hy = -1e9;
    for (const Pt& p : vertices_of(*static_cast<const ncad::Polyline*>(pls[0]))) {
        lx = std::min(lx, p.x);
        hx = std::max(hx, p.x);
        ly = std::min(ly, p.y);
        hy = std::max(hy, p.y);
    }
    CHECK_NEAR(lx, 638.334, 0.01);
    CHECK_NEAR(hx, 678.334, 0.01);
    CHECK_NEAR(ly, 348.976, 0.01);
    CHECK_NEAR(hy, 388.976, 0.01);

    // Bottom-left: the workspace is the design, unchanged.
    Fixture g;
    g.db.sysvars().set_string(g.session->lbrn2_corner, "bl");
    REQUIRE(g.import(Fixture::sample("f03.lbrn2")) == EngineStatus::Finished);
    const auto pg = g.entities(EntityType::Polyline);
    REQUIRE(pg.size() == 1);
    double gx = 1e9;
    for (const Pt& p : vertices_of(*static_cast<const ncad::Polyline*>(pg[0]))) gx = std::min(gx, p.x);
    CHECK_NEAR(gx, 221.666, 0.01);

    // A bad corner is refused and draws nothing.
    Fixture h;
    h.db.sysvars().set_string(h.session->lbrn2_corner, "middle");
    CHECK(h.import(Fixture::sample("f03.lbrn2")) == EngineStatus::Failed);
    CHECK(contains(h.transcript, "LBRN2CORNER"));
    CHECK(h.db.empty());
}

TEST_CASE("lbrn2in: layers are named and coloured from the project; ellipses, open paths, name clashes") {
    const std::string proj =
        "<LightBurnProject FormatVersion=\"1\" MirrorX=\"False\" MirrorY=\"False\">\n"
        "<CutSetting type=\"Cut\"><index Value=\"1\"/><name Value=\"Score\"/><speed Value=\"300\"/>"
        "<minPower Value=\"10\"/><maxPower Value=\"12\"/></CutSetting>\n"
        "<CutSetting type=\"Cut\"><index Value=\"3\"/><speed Value=\"40\"/>"
        "<minPower Value=\"30\"/><maxPower Value=\"35\"/><numPasses Value=\"2\"/></CutSetting>\n"
        "<CutSetting type=\"Cut\"><index Value=\"4\"/><name Value=\"Score\"/><speed Value=\"60\"/>"
        "<minPower Value=\"5\"/><maxPower Value=\"6\"/></CutSetting>\n"
        "<Shape Type=\"Ellipse\" CutIndex=\"1\" Rx=\"10\" Ry=\"5\"><XForm>1 0 0 1 50 60</XForm></Shape>\n"
        "<Shape Type=\"Path\" CutIndex=\"3\" VertID=\"0\"><XForm>1 0 0 1 0 0</XForm>"
        "<VertList>V0 0c0x1c1x1V10 0c0x1c1x1V10 10c0x1c1x1</VertList><PrimList>L0 1 L1 2</PrimList></Shape>\n"
        "<Shape Type=\"Ellipse\" CutIndex=\"4\" Rx=\"3\" Ry=\"3\"><XForm>1 0 0 1 5 5</XForm></Shape>\n"
        "</LightBurnProject>\n";
    Fixture f;
    std::ofstream(f.dir / "mix.lbrn2", std::ios::binary) << proj;
    f.db.sysvars().set_string(f.session->lbrn2_corner, "bl");
    REQUIRE(f.import(f.path("mix.lbrn2")) == EngineStatus::Finished);

    REQUIRE(f.entities(EntityType::Ellipse).size() == 1);
    REQUIRE(f.entities(EntityType::Circle).size() == 1);
    const auto open = f.entities(EntityType::Polyline);
    REQUIRE(open.size() == 1);
    CHECK(!static_cast<const ncad::Polyline*>(open[0])->closed());
    const auto* el = static_cast<const ncad::Ellipse*>(f.entities(EntityType::Ellipse)[0]);
    CHECK_NEAR(el->ratio(), 0.5, 1e-9);
    CHECK_NEAR(el->center().x, 50.0, 1e-9);
    CHECK_NEAR(el->major_axis().x, 10.0, 1e-9);

    // Named layer, an unnamed one as LB<index>, and the repeated name with its index.
    const ncad::LayerId score = f.db.find_layer("Score");
    const ncad::LayerId lb3 = f.db.find_layer("LB3");
    const ncad::LayerId score4 = f.db.find_layer("Score_4");
    REQUIRE(score != ncad::kInvalidLayer);
    REQUIRE(lb3 != ncad::kInvalidLayer);
    REQUIRE(score4 != ncad::kInvalidLayer);
    CHECK(f.db.layer(score).color == 5);  // layer 1: blue
    // Layer 3 is LightBurn's green; which ACI is nearest is the table's call,
    // but it is not the default 7.
    CHECK(f.db.layer(lb3).color != 7);
    CHECK(f.entities(EntityType::Ellipse)[0]->props().layer == score);
    CHECK(open[0]->props().layer == lb3);
    CHECK(f.entities(EntityType::Circle)[0]->props().layer == score4);

    // Pens: every layer keeps its own speed and passes, under its own ACI.
    REQUIRE(f.session->imported_pens.size() == 3);
    CHECK(f.session->imported_pens[1].pen == f.db.layer(lb3).color);
    CHECK(f.session->imported_pens[1].passes == 2);
    CHECK_NEAR(f.session->imported_pens[2].speed, 60.0, 1e-9);
    CHECK(f.session->imported_pens[0].pen != f.session->imported_pens[2].pen);
}

TEST_CASE("lbrn2in: an unreadable project is refused and leaves the drawing alone") {
    Fixture f;
    std::ofstream(f.dir / "bad.lbrn2") << "<LightBurnProject FormatVersion=\"1\"><Shape Type=\"Text\"/></LightBurnProject>";
    CHECK(f.import(f.path("bad.lbrn2")) == EngineStatus::Failed);
    CHECK(f.db.empty());
    CHECK(!f.session->has_imported_pens);
    CHECK(f.import(f.path("nonesuch.lbrn2")) == EngineStatus::Failed);
}

TEST_CASE("lbrn2in: LASER Job then Save .rd reproduces the sample's structure") {
    for (const Sample& smp : kSamples) {
        Fixture f;
        REQUIRE(f.import(Fixture::sample(smp.lbrn2)) == EngineStatus::Finished);
        REQUIRE(f.laser({"Job", "Extents"}) == EngineStatus::Finished);
        // The imported pens, not a table on disk, gave the job its power.
        CHECK(!contains(f.transcript, "power 0-0"));
        const std::string out = f.path("out.rd");
        REQUIRE(f.laser({"Save", out}) == EngineStatus::Finished);

        const std::string mine = f.read(out);
        const std::string theirs = f.read(Fixture::sample(smp.rd));
        REQUIRE(!mine.empty());
        const ruida::RdSummary a = summary_of(mine), b = summary_of(theirs);
        // LightBurn spaces circle vertices differently, so f07 is compared on
        // length to a fraction of a percent, not to the micron.
        const double len_tol = smp.circles ? 0.0005 * b.cut_mm : 0.01;
        CHECK_NEAR(a.cut_mm, b.cut_mm, len_tol);
        if (!smp.circles) CHECK(a.cuts == b.cuts);
        REQUIRE(a.have_cut_bounds);
        REQUIRE(b.have_cut_bounds);
        CHECK_NEAR(a.cut_max_x - a.cut_min_x, b.cut_max_x - b.cut_min_x, 0.02);
        CHECK_NEAR(a.cut_max_y - a.cut_min_y, b.cut_max_y - b.cut_min_y, 0.02);
        CHECK(layer_settings(mine) == layer_settings(theirs));
        CHECK(!layer_settings(mine).empty());
    }
}

TEST_CASE("lbrn2in: Save .lbrn2 round-trips through read_lbrn2") {
    Fixture f("[bench]\nhost = 192.0.2.1\norigin_corner = tr\nbed = 900 600\n");
    REQUIRE(f.import(Fixture::sample("f05.lbrn2")) == EngineStatus::Finished);
    REQUIRE(f.laser({"Job", "Extents"}) == EngineStatus::Finished);
    const std::string out = f.path("out.lbrn2");
    REQUIRE(f.laser({"Save", out}) == EngineStatus::Finished);
    CHECK(contains(f.transcript, "Saved"));

    LaserJob back;
    Lbrn2Info info;
    Lbrn2Options o;
    o.machine.origin = ruida::MachineCorner::TopRight;
    o.machine.bed_w_mm = 900;
    o.machine.bed_h_mm = 600;
    std::string err;
    REQUIRE(read_lbrn2_file(out, back, info, o, err));
    const LaserJob& job = f.session->job;
    REQUIRE(back.layers.size() == job.layers.size());
    CHECK_NEAR(back.layers[0].speed, 100.0, 1e-9);
    CHECK(back.layers[0].passes == 2);
    CHECK_NEAR(back.layers[0].min_power, 15.0, 1e-9);
    REQUIRE(back.paths.size() == job.paths.size());
    REQUIRE(back.paths[0].pts.size() == job.paths[0].pts.size());
    for (std::size_t i = 0; i < job.paths[0].pts.size(); ++i) {
        CHECK_NEAR(back.paths[0].pts[i].x, job.paths[0].pts[i].x, 0.001);
        CHECK_NEAR(back.paths[0].pts[i].y, job.paths[0].pts[i].y, 0.001);
    }
}

TEST_CASE("lbrn2in: Save .lbrn2 refuses what write_lbrn2 refuses, with its message") {
    Fixture f;
    std::ofstream(f.dir / "off.lpt") << "layer 1 color=1 mode=auto speed=20 power=30,30 passes=1 air=off order=1\n";
    auto sq = std::make_unique<ncad::Polyline>();
    sq->add({0, 0, 0});
    sq->add({10, 0, 0});
    sq->add({10, 10, 0});
    sq->set_closed(true);
    sq->props().color = 1;
    f.db.add(std::move(sq));
    REQUIRE(f.laser({"Pens", "Load", f.path("off.lpt"), "", "Job", "Extents"}) == EngineStatus::Finished);
    REQUIRE(f.session->has_job);

    // What write_lbrn2 says about this job, directly.
    std::string why, text;
    REQUIRE(!write_lbrn2(f.session->job, Lbrn2Options{}, text, why));
    REQUIRE(!why.empty());

    const std::string out = f.path("off.lbrn2");
    CHECK(f.laser({"Save", out}) == EngineStatus::Finished);
    CHECK(contains(f.transcript, why));
    CHECK(!fs::exists(out));
}

TEST_CASE("lbrn2in: Pens Load takes a .lbrn2 as a pen table") {
    Fixture f;
    auto sq = std::make_unique<ncad::Polyline>();
    sq->add({0, 0, 0});
    sq->add({10, 0, 0});
    sq->add({10, 10, 0});
    sq->set_closed(true);
    sq->props().color = 1;  // red: LightBurn's layer 2
    f.db.add(std::move(sq));

    const std::string lb = Fixture::sample("f05.lbrn2");
    REQUIRE(f.laser({"Pens", "Load", lb, "List"}) == EngineStatus::Finished);
    CHECK(f.db.sysvars().get_string(f.session->pens) == lb);

    // The table is the sample's CutSetting under ACI 1: 100 mm/s, 15% and two passes.
    REQUIRE(f.laser({"Pens", "Load", lb, "", "Job", "Extents"}) == EngineStatus::Finished);
    REQUIRE(f.session->has_job);
    REQUIRE(f.session->job.layers.size() == 1);
    const LaserLayer& l = f.session->job.layers[0];
    CHECK(l.pen == 1);
    CHECK_NEAR(l.speed, 100.0, 1e-9);
    CHECK_NEAR(l.min_power, 15.0, 1e-9);
    CHECK_NEAR(l.max_power, 15.0, 1e-9);
    CHECK(l.has_power2);
    CHECK(l.passes == 2);

    // A project that is not readable is reported where it was typed.
    std::ofstream(f.dir / "bad.lbrn2") << "not xml";
    CHECK(f.laser({"Pens", "Load", f.path("bad.lbrn2")}) == EngineStatus::Finished);
    CHECK(contains(f.transcript, "line 1"));
}
