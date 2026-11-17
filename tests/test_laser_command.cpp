// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// The LASER command, driven through NotoCAD's own CommandEngine with a
// scripted input source, against an in-memory controller. Nothing here opens a
// socket: the connector hands the command a RuidaTransport over the fake.

#include "test.hpp"

#include "fake_controller.hpp"

#include "ncad/command.hpp"
#include "ncad/commands.hpp"
#include "ncad/database.hpp"
#include "ncad/entities.hpp"
#include "ncad/input_text.hpp"
#include "ncad/laser/laser_command.hpp"
#include "ncad/laser/plan.hpp"
#include "ncad/laser/ruida_decode.hpp"
#include "ncad/laser/ruida_transport.hpp"
#include "ncad/lisp/eval.hpp"
#include "ncad/sysvar.hpp"

#include <unistd.h>

#include <cctype>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace ncad::laser;
using ncad::CommandEngine;
using ncad::Database;
using ncad::EngineStatus;
using ncad::Vec3;
using nlaser_test::FakeController;

namespace {

namespace fs = std::filesystem;

// Registrations are process-global; every case ends by forgetting them so the
// next starts from a NotoCAD that has never heard of lasers.
struct Fixture {
    Database db;
    CommandEngine engine{db};
    FakeController fake;
    std::shared_ptr<LaserSession> session;
    fs::path dir;
    int connects{0};

    explicit Fixture(const std::string& conf = "[bench]\nhost = 192.0.2.1\nbed = 600 400\n",
                     const std::string& lpt = "layer 1 color=1 mode=auto speed=20 power=65,65 passes=1 air=on order=1\n") {
        static int serial = 0;
        dir = fs::temp_directory_path() / ("nlaser_cmd_" + std::to_string(::getpid()) + "_" + std::to_string(++serial));
        fs::create_directories(dir);
        write("lasers.conf", conf);
        write("pens.lpt", lpt);

        LaserOptions opts;
        opts.config_path = (dir / "lasers.conf").string();
        opts.connector = [this](const DeviceConfig& cfg, std::unique_ptr<LaserTransport>& out, std::string&) {
            ++connects;
            out = std::make_unique<ruida::RuidaTransport>(fake, cfg);
            return true;
        };
        session = install_laser(opts);
    }

    ~Fixture() {
        ncad::clear_registered_commands();
        ncad::clear_registered_sysvars();
        std::error_code ec;
        fs::remove_all(dir, ec);
    }

    void write(const char* name, const std::string& text) {
        std::ofstream(dir / name, std::ios::binary) << text;
    }

    std::string path(const char* name) const { return (dir / name).string(); }

    std::string read(const std::string& p) const {
        std::ifstream in(p, std::ios::binary);
        return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    }

    // A 10 mm square on ACI 1, plus a far-off line on ACI 2.
    void draw() {
        auto sq = std::make_unique<ncad::Polyline>();
        sq->add({0, 0, 0});
        sq->add({10, 0, 0});
        sq->add({10, 10, 0});
        sq->add({0, 10, 0});
        sq->set_closed(true);
        sq->props().color = 1;
        db.add(std::move(sq));
        auto far = std::make_unique<ncad::Line>(Vec3{100, 100, 0}, Vec3{120, 100, 0});
        far->props().color = 2;
        db.add(std::move(far));
    }

    // Every prompt the command put up, and its final message: what a person
    // would have read. The mode reports through the next prompt, so this is
    // where a keyword's output is.
    std::string transcript;

    // Feeds the tokens and records each prompt before answering it.
    struct Recording final : ncad::InputSource {
        ncad::TextInputSource inner;
        std::string& log;
        Recording(std::vector<std::string> t, std::string& l) : inner(std::move(t)), log(l) {}
        bool next_value(const ncad::Prompt& p, ncad::InputValue& out) override {
            log += p.message + "\n";
            return inner.next_value(p, out);
        }
    };

    // Starts LASER and answers with `tokens` exactly; the mode is left running
    // unless they end it.
    EngineStatus run_raw(std::vector<std::string> tokens) {
        transcript.clear();
        Recording src(std::move(tokens), transcript);
        src.inner.set_database(&db);
        engine.begin(ncad::make_command("LASER"));
        const EngineStatus st = engine.run(src);
        // The prompt it stopped at, if it did.
        if (st == EngineStatus::Waiting) transcript += engine.prompt().message + "\n";
        transcript += engine.message();
        return st;
    }

    // The same, with Enters at the end that leave a sub-mode and then the mode.
    EngineStatus run(std::vector<std::string> tokens) {
        tokens.insert(tokens.end(), 4, std::string());
        return run_raw(std::move(tokens));
    }

    const std::string& out() const { return transcript; }

    void use_pens() { run({"Pens", "Load", path("pens.lpt")}); }
};

bool contains(const std::string& s, const char* part) { return s.find(part) != std::string::npos; }
bool contains(const std::string& s, const std::string& part) { return s.find(part) != std::string::npos; }

}  // namespace

TEST_CASE("laser: install registers the command and its four variables") {
    Fixture f;
    REQUIRE(f.session != nullptr);
    CHECK(ncad::make_command("LASER") != nullptr);
    CHECK(ncad::resolve_command_name("LAS").name == "LASER");

    const ncad::Sysvars& sv = f.db.sysvars();
    CHECK(sv.get_string(f.session->dev).empty());
    CHECK(sv.get_string(f.session->pens).empty());
    CHECK_NEAR(sv.get_real(f.session->tol), 0.01, 1e-12);
    CHECK(sv.get_int(f.session->confirm) == 1);

    // Reachable by name like any built-in, with its range enforced.
    CHECK(f.db.sysvars().set("LASERCONFIRM", ncad::SysvarValue::of_int(0)) == ncad::Sysvars::SetStatus::Ok);
    CHECK(f.db.sysvars().set("LASERCONFIRM", ncad::SysvarValue::of_int(2)) == ncad::Sysvars::SetStatus::OutOfRange);
}

TEST_CASE("laser: Device picks from lasers.conf and sets LASERDEV") {
    Fixture f("[bench]\nhost = 192.0.2.1\n[a2]\nhost = 192.0.2.2\n");
    CHECK(f.run({"Device", "nonesuch"}) == EngineStatus::Finished);
    CHECK(contains(f.out(), "error: no device 'nonesuch'"));
    CHECK(f.db.sysvars().get_string(f.session->dev).empty());

    CHECK(f.run({"Device", "a2"}) == EngineStatus::Finished);
    CHECK(f.db.sysvars().get_string(f.session->dev) == "a2");
    CHECK(contains(f.out(), "192.0.2.2"));

    // Enter at the prompt reports and changes nothing.
    CHECK(f.run({"Device", ""}) == EngineStatus::Finished);
    CHECK(f.db.sysvars().get_string(f.session->dev) == "a2");
}

TEST_CASE("laser: Pens validates the file and sets LASERPENS") {
    Fixture f;
    CHECK(f.run({"Pens", "Load", f.path("missing.lpt")}) == EngineStatus::Finished);
    CHECK(contains(f.out(), "error:"));
    CHECK(f.db.sysvars().get_string(f.session->pens).empty());
    CHECK(f.run({"Pens", "Load", f.path("pens.lpt")}) == EngineStatus::Finished);
    CHECK(f.db.sysvars().get_string(f.session->pens) == f.path("pens.lpt"));
    CHECK(f.run({"Pens", "Load", "."}) == EngineStatus::Finished);
    CHECK(f.db.sysvars().get_string(f.session->pens).empty());
}

TEST_CASE("laser: Job builds, plans and reports; Save .ljob matches the pipeline byte for byte") {
    Fixture f;
    f.draw();
    f.use_pens();

    CHECK(f.run({"Job", "Extents"}) == EngineStatus::Finished);
    const std::string msg = f.out();
    CHECK(contains(msg, "2 paths on 2 pens"));
    CHECK(contains(msg, "layer 1"));
    CHECK(contains(msg, "Bounds 0.000,0.000 .. 120.000,100.000"));
    CHECK(contains(msg, "estimated"));
    // Pen 2 has no pen-table entry: warned, and it would run at power 0.
    CHECK(contains(msg, "pen 2 has no pen-table entry"));
    REQUIRE(f.session->has_job);
    CHECK(f.session->job.planned);

    // The same pipeline by hand.
    std::vector<LaserLayer> pens;
    std::string error;
    REQUIRE(read_pen_table_file(f.path("pens.lpt"), pens, error));
    LaserJob raw, expected;
    DrawingReport report;
    DrawingOptions opts;
    REQUIRE(render_drawing(f.db, opts, raw, report, error));
    apply_pens(raw, pens, report);
    PlanReport plan;
    REQUIRE(plan_job(raw, expected, PlanOptions{}, plan, error));

    const std::string out = f.path("out.ljob");
    CHECK(f.run({"Save", out}) == EngineStatus::Finished);
    CHECK(f.read(out) == write_ljob(expected));
    CHECK(f.read(out) == write_ljob(f.session->job));
}

TEST_CASE("laser: Job Window takes whole entities that touch it, and only those") {
    Fixture f;
    f.draw();
    CHECK(f.run({"Job", "Window", "-5,-5", "15,15"}) == EngineStatus::Finished);
    REQUIRE(f.session->has_job);
    CHECK(f.session->job.paths.size() == 1);

    // A window on nothing is an error, not an empty job.
    CHECK(f.run({"Job", "Window", "500,500", "600,600"}) == EngineStatus::Finished);
    CHECK(contains(f.out(), "error:"));
}

TEST_CASE("laser: Job honours LASERTOL") {
    Fixture f;
    auto c = std::make_unique<ncad::Circle>(Vec3{0, 0, 0}, 20.0);
    c->props().color = 1;
    f.db.add(std::move(c));
    f.run({"Job", "Extents"});
    const std::size_t fine = f.session->job.paths[0].pts.size();
    f.db.sysvars().set_real(f.session->tol, 0.5);
    f.run({"Job", "Extents"});
    CHECK(f.session->job.paths[0].pts.size() < fine);
    f.db.sysvars().set_real(f.session->tol, -1.0);
    CHECK(f.run({"Job", "Extents"}) == EngineStatus::Finished);
    CHECK(contains(f.out(), "LASERTOL"));
}

TEST_CASE("laser: a single straight line is a job, though its extents have no height") {
    Fixture f;
    auto l = std::make_unique<ncad::Line>(Vec3{0, 0, 0}, Vec3{30, 0, 0});
    l->props().color = 1;
    f.db.add(std::move(l));
    CHECK(f.run({"Job", "Extents"}) == EngineStatus::Finished);
    CHECK(f.session->job.paths.size() == 1);
    // Enter at the area prompt is Extents; Display with no view here falls back, said so.
    CHECK(f.run({"Job", ""}) == EngineStatus::Finished);
    CHECK(!contains(f.out(), "No display here"));
    CHECK(f.run({"Job", "Display"}) == EngineStatus::Finished);
    CHECK(contains(f.out(), "No display here"));
}

TEST_CASE("laser: an empty drawing makes no job") {
    Fixture f;
    CHECK(f.run({"Job", "Extents"}) == EngineStatus::Finished);
    CHECK(contains(f.out(), "error: nothing to cut"));
    CHECK(!f.session->has_job);
}

TEST_CASE("laser: Save .rd decodes to the job's geometry") {
    // Two devices and none chosen, so the file is encoded for the defaults.
    Fixture f("[bench]\nhost = 192.0.2.1\n[a2]\nhost = 192.0.2.2\n");
    f.draw();
    f.use_pens();
    f.run({"Job", "Window", "-5,-5", "15,15"});
    const std::string out = f.path("out.rd");
    CHECK(f.run({"Save", out}) == EngineStatus::Finished);
    CHECK(contains(f.out(), "no device chosen"));

    const std::string data = f.read(out);
    REQUIRE(!data.empty());
    const std::string plain = ruida::Swizzle(0x88).decode(data);
    const ruida::RdSummary s = ruida::summarize_rd(ruida::decode_rd(plain));
    CHECK(s.cuts > 0);
    CHECK_NEAR(s.cut_mm, 40.0, 0.01);
}

TEST_CASE("laser: Save .rd for a chosen device uses that device's machine") {
    Fixture f("[bench]\nhost = 192.0.2.1\norigin_corner = tr\nmagic = 0x11\n");
    f.draw();
    f.use_pens();
    f.run({"Job", "Window", "-5,-5", "15,15"});
    const std::string out = f.path("tr.rd");
    CHECK(f.run({"Save", out}) == EngineStatus::Finished);
    CHECK(!contains(f.out(), "no device chosen"));
    const std::string plain = ruida::Swizzle(0x11).decode(f.read(out));
    ruida::RdMachine m;
    m.origin = ruida::MachineCorner::TopRight;
    CHECK(ruida::summarize_rd(ruida::decode_rd(plain), m).cuts > 0);
}

TEST_CASE("laser: Save asks before replacing a file, and No keeps it") {
    Fixture f;
    f.draw();
    f.run({"Job", "Extents"});
    const std::string out = f.path("keep.ljob");
    f.write("keep.ljob", "mine\n");
    f.run({"Save", out, "No"});
    CHECK(f.read(out) == "mine\n");
    f.run({"Save", out, "Yes"});
    CHECK(contains(f.read(out), "ljob 1"));
}

TEST_CASE("laser: Send with LASERCONFIRM 0 uploads to the controller") {
    Fixture f;
    f.draw();
    f.use_pens();
    f.run({"Device", "bench"});
    f.run({"Job", "Window", "-5,-5", "15,15"});
    f.db.sysvars().set_int(f.session->confirm, 0);

    CHECK(f.run({"Send"}) == EngineStatus::Finished);
    CHECK(f.connects == 1);
    CHECK(f.fake.enqs >= 1);
    CHECK(!f.fake.job.empty());
    CHECK(contains(f.out(), "Sent"));

    // What went over the wire is the same job: its cut length is the square's.
    const ruida::RdSummary s = ruida::summarize_rd(ruida::decode_rd(f.fake.job));
    CHECK_NEAR(s.cut_mm, 40.0, 0.01);
}

TEST_CASE("laser: Send asks first, and only Yes sends") {
    Fixture f;
    f.draw();
    f.use_pens();
    f.run({"Device", "bench"});
    f.run({"Job", "Window", "-5,-5", "15,15"});

    // The default answer is No.
    CHECK(f.run({"Send", ""}) == EngineStatus::Finished);
    CHECK(contains(f.out(), "Not sent"));
    CHECK(f.connects == 0);
    CHECK(f.fake.packets.empty());

    CHECK(f.run({"Send", "No"}) == EngineStatus::Finished);
    CHECK(f.fake.packets.empty());

    CHECK(f.run({"Send", "Yes"}) == EngineStatus::Finished);
    CHECK(!f.fake.job.empty());
}

TEST_CASE("laser: the confirmation prompt says what is about to run") {
    Fixture f;
    f.draw();
    f.use_pens();
    f.run({"Device", "bench"});
    f.run({"Job", "Window", "-5,-5", "15,15"});
    f.run_raw({"Send"});
    REQUIRE(f.engine.status() == EngineStatus::Waiting);
    const std::string prompt = f.engine.prompt().message;
    CHECK(contains(prompt, "bench"));
    CHECK(contains(prompt, "power 65-65%"));
    CHECK(contains(prompt, "bounds 0.000,0.000 .. 10.000,10.000"));
    CHECK(contains(prompt, "[Yes/No]"));
}

TEST_CASE("laser: Send refuses without a job, a device, or within the power ceiling") {
    {
        Fixture f;
        f.draw();
        CHECK(f.run({"Send"}) == EngineStatus::Finished);  // no job
        CHECK(contains(f.out(), "error: no job"));
        CHECK(f.fake.packets.empty());
    }
    {
        Fixture f("[bench]\nhost = 192.0.2.1\n[a2]\nhost = 192.0.2.2\n");
        f.draw();
        f.use_pens();
        f.run({"Job", "Extents"});
        CHECK(f.run({"Send"}) == EngineStatus::Finished);  // two devices, none chosen
        CHECK(contains(f.out(), "error:"));
        CHECK(f.connects == 0);
    }
    {
        Fixture f("[bench]\nhost = 192.0.2.1\nmax_power = 50\n");
        f.draw();
        f.use_pens();
        f.run({"Job", "Window", "-5,-5", "15,15"});
        CHECK(f.run({"Send"}) == EngineStatus::Finished);  // layer 1 asks for 65%
        CHECK(contains(f.out(), "error:"));
        CHECK(contains(f.out(), "65.0%"));
        CHECK(f.connects == 0);
        CHECK(f.fake.packets.empty());
    }
}

TEST_CASE("laser: Send refuses a job bigger than the bed") {
    Fixture f("[bench]\nhost = 192.0.2.1\nbed = 50 50\n");
    f.draw();
    f.use_pens();
    f.run({"Job", "Extents"});  // 120 x 100
    CHECK(f.run({"Send"}) == EngineStatus::Finished);
    CHECK(contains(f.out(), "error:"));
    CHECK(contains(f.out(), "bed"));
    CHECK(f.fake.packets.empty());
}

TEST_CASE("laser: a connection failure is reported, not hung on") {
    Fixture f;
    f.draw();
    f.use_pens();
    f.run({"Job", "Window", "-5,-5", "15,15"});
    f.db.sysvars().set_int(f.session->confirm, 0);
    f.fake.silent = true;  // the controller never answers ENQ
    CHECK(f.run({"Send"}) == EngineStatus::Finished);
    CHECK(contains(f.out(), "error:"));
    CHECK(contains(f.out(), "no answer"));
}

TEST_CASE("laser: Frame uploads a laser-off box") {
    Fixture f;
    f.draw();
    f.use_pens();
    f.run({"Job", "Window", "-5,-5", "15,15"});
    CHECK(f.run({"Frame"}) == EngineStatus::Finished);  // no confirmation: the laser is off
    CHECK(!f.fake.job.empty());
    const ruida::RdSummary s = ruida::summarize_rd(ruida::decode_rd(f.fake.job));
    CHECK(s.cuts == 0);
    CHECK(s.moves > 0);
}

TEST_CASE("laser: Dryrun traces the whole planned path with no cut and no power") {
    Fixture f;
    f.draw();
    f.use_pens();
    f.run({"Job", "Window", "-5,-5", "15,15"});
    CHECK(f.run({"Dryrun"}) == EngineStatus::Finished);  // no confirmation: the laser is off
    CHECK(contains(f.out(), "laser off"));
    CHECK(!f.fake.job.empty());
    const auto cmds = ruida::decode_rd(f.fake.job);
    const ruida::RdSummary s = ruida::summarize_rd(cmds);
    CHECK(s.cuts == 0);
    CHECK(s.moves > 4);
    for (const ruida::RdCommand& cmd : cmds) {
        for (const ruida::RdField& fld : cmd.fields) {
            if (fld.kind == 'p') CHECK(fld.i == 0);
        }
    }
}

TEST_CASE("laser: Status, Counters and STop reach the controller") {
    Fixture f;
    CHECK(f.run({"Status"}) == EngineStatus::Finished);
    CHECK(contains(f.out(), "Status of bench"));
    CHECK(f.run({"Counters"}) == EngineStatus::Finished);
    CHECK(contains(f.out(), "Counters of bench"));
    CHECK(f.run({"STop"}) == EngineStatus::Finished);
    CHECK(f.fake.stops == 1);
}

TEST_CASE("laser: Preview prints the planned sequence; Order List is its old spelling") {
    Fixture f;
    f.draw();
    f.use_pens();
    f.run({"Job", "Extents"});
    CHECK(f.run({"Preview"}) == EngineStatus::Finished);
    CHECK(contains(f.out(), "Cutting order: 2 paths"));
    CHECK(contains(f.out(), "pen 1  closed"));

    CHECK(f.run({"Order", "List"}) == EngineStatus::Finished);
    CHECK(contains(f.out(), "Cutting order: 2 paths"));
    // The "List" was swallowed, not taken for an unknown option.
    CHECK(!contains(f.out(), "unknown"));

    // No job, no preview.
    Fixture g;
    CHECK(g.run({"Preview"}) == EngineStatus::Finished);
    CHECK(contains(g.out(), "error: no job"));
}

TEST_CASE("laser: Origin reports the LASER UCS, or world") {
    Fixture f;
    CHECK(f.run({"Origin", ""}) == EngineStatus::Finished);
    CHECK(contains(f.out(), "0.000,0.000 (world"));
    CHECK(contains(f.out(), "head's current position"));
    CHECK(f.session->start == StartMode::Current);

    ncad::Ucs u;
    u.origin = {20, 30, 0};
    f.db.add_ucs("LASER", u);
    CHECK(f.run({"Origin", "Current"}) == EngineStatus::Finished);
    CHECK(contains(f.out(), "20.000,30.000 (UCS LASER)"));

    f.draw();
    f.run({"Job", "Extents"});
    CHECK_NEAR(f.session->job.origin.x, 20.0, 1e-9);
    CHECK_NEAR(f.session->job.origin.y, 30.0, 1e-9);
    CHECK(f.session->job.start == StartMode::Current);

    // Start From is session state: it applies to the job already built and to
    // the next one.
    f.run_raw({"Origin"});
    REQUIRE(f.engine.status() == EngineStatus::Waiting);
    CHECK(contains(f.engine.prompt().message, "Start from [Current/Absolute]"));
    CHECK(f.run({"Origin", "Absolute"}) == EngineStatus::Finished);
    CHECK(contains(f.out(), "absolute machine coordinates"));
    CHECK(f.session->start == StartMode::Absolute);
    CHECK(f.session->job.start == StartMode::Absolute);
    f.run({"Job", "Extents"});
    CHECK(f.session->job.start == StartMode::Absolute);
    CHECK(f.run({"Origin", "Sideways"}) == EngineStatus::Finished);
    CHECK(contains(f.out(), "error: Start from"));
}

TEST_CASE("laser: the job outlives the command that built it") {
    Fixture f;
    f.draw();
    f.use_pens();
    f.run({"Device", "bench"});
    f.run({"Job", "Window", "-5,-5", "15,15"});
    // A different drawing operation in between changes nothing about the job.
    f.db.add(std::make_unique<ncad::Line>(Vec3{0, 0, 0}, Vec3{1, 1, 0}));
    f.db.sysvars().set_int(f.session->confirm, 0);
    CHECK(f.run({"Send"}) == EngineStatus::Finished);
    CHECK_NEAR(ncad::laser::ruida::summarize_rd(ruida::decode_rd(f.fake.job)).cut_mm, 40.0, 0.01);
}

TEST_CASE("laser: (command \"LASER\" ...) drives it from AutoLISP") {
    Fixture f;
    f.draw();
    f.use_pens();

    ncad::lisp::Context ctx;
    ncad::lisp::Interp in{ctx};
    std::ostringstream out;
    in.set_output(&out);
    in.set_database(&f.db);
    in.set_command_engine(&f.engine);

    ncad::lisp::Value v;
    // Abbreviated keywords, as at the terminal.
    REQUIRE(in.eval_string(R"((command "LASER" "J" "E" ""))", v));
    REQUIRE(f.session->has_job);
    CHECK(f.session->job.paths.size() == 2);

    REQUIRE(in.eval_string(R"((getvar "LASERTOL"))", v));
    CHECK(ncad::lisp::prin1(v) == "0.01");
    REQUIRE(in.eval_string(R"((setvar "LASERCONFIRM" 0))", v));
    CHECK(f.db.sysvars().get_int(f.session->confirm) == 0);

    // A device operation from LISP, in one call: Device, then Send. ("DE":
    // since Dryrun, a bare "D" is ambiguous.) The trailing "" leaves the mode.
    REQUIRE(in.eval_string(R"((command "LASER" "DE" "bench" "Send" ""))", v));
    CHECK(!f.fake.job.empty());
}

TEST_CASE("laser: without a connector a device operation says so") {
    Fixture f;
    f.session->options.connector = nullptr;
    f.draw();
    f.use_pens();
    f.run({"Job", "Extents"});
    f.db.sysvars().set_int(f.session->confirm, 0);
    CHECK(f.run({"Send"}) == EngineStatus::Finished);
    CHECK(contains(f.out(), "network"));
}

// --- the modal prompt ---------------------------------------------------------------

namespace {

// Records what Move was asked to do, in order.
struct MoveLog {
    std::vector<std::string> moves;
};

struct FakeMove final : MoveTarget {
    explicit FakeMove(MoveLog* l) : log(l) {}
    MoveLog* log;
    bool jog(double dx, double dy, std::string&) override {
        char b[64];
        std::snprintf(b, sizeof b, "jog %g,%g", dx, dy);
        log->moves.push_back(b);
        return true;
    }
    bool goto_point(double x, double y, std::string&) override {
        char b[64];
        std::snprintf(b, sizeof b, "goto %g,%g", x, y);
        log->moves.push_back(b);
        return true;
    }
};

void use_fake_moves(Fixture& f, MoveLog& log) {
    f.session->options.move_connector = [&log](const DeviceConfig&, std::unique_ptr<MoveTarget>& out, std::string&) {
        out = std::make_unique<FakeMove>(&log);
        return true;
    };
}

// The words in the last [...] of a prompt, split at '/'.
std::vector<std::string> bracket_items(const std::string& message) {
    std::vector<std::string> items;
    const std::size_t open = message.rfind('[');
    const std::size_t close = message.find(']', open);
    if (open == std::string::npos || close == std::string::npos) return items;
    std::string cur;
    for (std::size_t i = open + 1; i < close; ++i) {
        if (message[i] == '/') {
            items.push_back(cur);
            cur.clear();
        } else {
            cur += message[i];
        }
    }
    items.push_back(cur);
    return items;
}

std::string upper(std::string s) {
    for (char& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

// The capitalised abbreviation each word advertises must select exactly that
// word under NotoCAD's own matcher, and the list shown must be the list the
// matcher has.
void check_abbreviations(const ncad::Prompt& p) {
    const std::vector<std::string> items = bracket_items(p.message);
    CHECK(items.size() == p.keywords.size());
    for (const std::string& item : items) {
        std::string prefix;
        for (char c : item) {
            if (std::islower(static_cast<unsigned char>(c))) break;
            prefix += c;
        }
        ncad::InputValue v;
        std::string err;
        CHECK(ncad::parse_input(p, prefix, v, err));
        CHECK(v.kind == ncad::InputKind::Keyword);
        CHECK(v.text == upper(item));
        // And the whole word, in any case.
        CHECK(ncad::parse_input(p, item, v, err));
        CHECK(v.text == upper(item));
    }
    // No keyword is a bare prefix of another, which would make it unreachable.
    for (std::size_t i = 0; i < p.keywords.size(); ++i) {
        for (std::size_t j = 0; j < p.keywords.size(); ++j) {
            if (i == j) continue;
            CHECK(upper(p.keywords[j]).compare(0, p.keywords[i].size(), upper(p.keywords[i])) != 0);
        }
    }
}

}  // namespace

TEST_CASE("laser: the mode ends on Enter and on Quit, and stays otherwise") {
    Fixture f;
    CHECK(f.run_raw({""}) == EngineStatus::Finished);
    CHECK(f.run_raw({"Quit"}) == EngineStatus::Finished);
    CHECK(f.run_raw({"Q"}) == EngineStatus::Finished);

    // A keyword returns to the Laser prompt rather than ending the command.
    CHECK(f.run_raw({"Origin", ""}) == EngineStatus::Waiting);
    CHECK(contains(f.engine.prompt().message, "Laser ["));
    CHECK(contains(f.engine.prompt().message, "<Quit>"));
    CHECK(f.run_raw({"Origin", "", ""}) == EngineStatus::Finished);
    CHECK(f.run_raw({"Origin", "", "Quit"}) == EngineStatus::Finished);
}

TEST_CASE("laser: the prompt abbreviations are unambiguous under NotoCAD's matcher") {
    Fixture f;
    f.draw();
    f.run_raw({});
    check_abbreviations(f.engine.prompt());
    CHECK(f.engine.prompt().keywords.size() == 16);
    f.run_raw({"Move"});
    check_abbreviations(f.engine.prompt());
    f.run_raw({"Pens"});
    check_abbreviations(f.engine.prompt());
    f.run_raw({"Job"});
    check_abbreviations(f.engine.prompt());
    f.run_raw({"Origin"});
    check_abbreviations(f.engine.prompt());
    f.run_raw({"Pens", "Set", "5", "", "", ""});
    check_abbreviations(f.engine.prompt());  // Air [On/Off]

    // The old collision, spelled out: D alone is ambiguous, DE and DR are not.
    f.run_raw({});
    ncad::InputValue v;
    std::string err;
    CHECK(ncad::parse_input(f.engine.prompt(), "D", v, err));
    CHECK(v.kind != ncad::InputKind::Keyword);  // ambiguous: left as plain text
    CHECK(ncad::parse_input(f.engine.prompt(), "DE", v, err));
    CHECK(v.text == "DEVICE");
    CHECK(ncad::parse_input(f.engine.prompt(), "DR", v, err));
    CHECK(v.text == "DRYRUN");
    CHECK(ncad::parse_input(f.engine.prompt(), "STA", v, err));
    CHECK(v.text == "STATUS");
    CHECK(ncad::parse_input(f.engine.prompt(), "STO", v, err));
    CHECK(v.text == "STOP");
}

TEST_CASE("laser: one session shares its state across keywords: Job, Save, Send") {
    Fixture f;
    f.draw();
    f.use_pens();
    f.db.sysvars().set_int(f.session->confirm, 0);
    const std::string out = f.path("one.ljob");

    CHECK(f.run({"Device", "bench", "Job", "Window", "-5,-5", "15,15", "Save", out, "Send"}) ==
          EngineStatus::Finished);
    CHECK(contains(f.out(), "Device bench"));
    CHECK(contains(f.out(), "1 paths on 1 pens"));
    CHECK(contains(f.out(), "Saved"));
    CHECK(contains(f.out(), "Sent"));
    CHECK(contains(f.read(out), "ljob 1"));
    CHECK_NEAR(ruida::summarize_rd(ruida::decode_rd(f.fake.job)).cut_mm, 40.0, 0.01);

    // A keyword that fails reports and goes on; the mode and its job survive it.
    CHECK(f.run({"Job", "Window", "500,500", "600,600", "Preview"}) == EngineStatus::Finished);
    CHECK(contains(f.out(), "error:"));
}

TEST_CASE("laser: Move jogs by the step, and Fine, Coarse and Step change it") {
    Fixture f;
    MoveLog log;
    use_fake_moves(f, log);
    f.run({"Device", "bench", "Move", "Coarse", "-X", "-X", "Fine", "+Y", "Step", "2.5", "+X", "-Y", "Goto", "3,4"});
    const std::vector<std::string> want = {"jog -10,0", "jog -10,0", "jog 0,0.1", "jog 2.5,0", "jog 0,-2.5", "goto 3,4"};
    CHECK(log.moves == want);
    CHECK(contains(f.out(), "(step 10 mm)"));
    CHECK(contains(f.out(), "(step 2.5 mm)"));
    CHECK(contains(f.out(), "moved -10,+0 mm"));

    // Enter leaves Move for the Laser prompt, which Enter then leaves.
    CHECK(f.run_raw({"Move", ""}) == EngineStatus::Waiting);
    CHECK(contains(f.engine.prompt().message, "Laser ["));

    // The step is the mode's: a bad one is refused and keeps the old.
    log.moves.clear();
    f.run({"Move", "Step", "-3", "+X"});
    CHECK(contains(f.out(), "error:"));
    CHECK(log.moves == std::vector<std::string>{"jog 1,0"});

    // Abbreviations, as at the terminal.
    log.moves.clear();
    f.run({"M", "C", "+X", "F", "-Y"});
    CHECK(log.moves == (std::vector<std::string>{"jog 10,0", "jog 0,-0.1"}));
}

TEST_CASE("laser: Move without a machine says so and keeps the mode") {
    Fixture f;
    f.session->options.connector = nullptr;  // nothing to reach a machine with: the stub
    CHECK(f.run({"Move", "+X", "Goto", "1,1"}) == EngineStatus::Finished);
    CHECK(contains(f.out(), "no network support"));
}

TEST_CASE("laser: Escape in a sub-prompt ends the command, keeping what was done") {
    Fixture f;
    f.draw();
    f.use_pens();
    CHECK(f.run_raw({"Job", "Extents", "Job", "\x1b"}) == EngineStatus::Cancelled);
    CHECK(f.session->has_job);  // built before the Escape, not undone
    CHECK(f.run_raw({"Job", "Window", "1,1", "\x1b"}) == EngineStatus::Cancelled);
    CHECK(f.run_raw({"Pens", "Set", "\x1b"}) == EngineStatus::Cancelled);

    MoveLog log;
    use_fake_moves(f, log);
    CHECK(f.run_raw({"Device", "bench", "Move", "+X", "Goto", "\x1b"}) == EngineStatus::Cancelled);
    CHECK(log.moves == std::vector<std::string>{"jog 1,0"});
    // Escape at the Laser prompt itself ends it too.
    CHECK(f.run_raw({"\x1b"}) == EngineStatus::Cancelled);
}

TEST_CASE("laser: Pens is a sub-mode: List, Load, Set, Save") {
    Fixture f;
    CHECK(f.run({"Pens", "Load", f.path("pens.lpt"), "List"}) == EngineStatus::Finished);
    // Load answered, and the Pens prompt came back for List.
    CHECK(contains(f.out(), "Pen table " + f.path("pens.lpt") + " (1 pens)"));
    CHECK(contains(f.out(), "layer 1 color=1"));

    // Set pen 2 from nothing, pen 1 kept as it is with Enter.
    const std::string out = f.path("edited.lpt");
    f.run({"Pens", "Load", f.path("pens.lpt"), "Set", "2", "15", "40", "2", "Off", "Set", "1", "", "", "", "", "List",
           "Save", out});
    CHECK(contains(f.out(), "pen 2: 15 mm/s, power 0-40%, passes 2, air off"));
    CHECK(contains(f.out(), "(edited, not saved)"));
    CHECK(contains(f.out(), "Saved 2 pens"));

    std::vector<LaserLayer> pens;
    std::string error;
    REQUIRE(read_pen_table_file(out, pens, error));
    REQUIRE(pens.size() == 2);
    CHECK(pens[0].pen == 1);
    CHECK_NEAR(pens[0].speed, 20.0, 1e-9);
    CHECK(pens[1].pen == 2);
    CHECK_NEAR(pens[1].max_power, 40.0, 1e-9);
    CHECK(pens[1].passes == 2);
    CHECK(!pens[1].air);

    // Edits reach the next Job in the same mode, without being saved.
    f.draw();
    f.run({"Pens", "Load", f.path("pens.lpt"), "Set", "1", "7", "30", "", "", "", "Job", "Extents"});
    REQUIRE(f.session->has_job);
    CHECK_NEAR(f.session->job.layer(1)->speed, 7.0, 1e-9);
    CHECK_NEAR(f.session->job.layer(1)->max_power, 30.0, 1e-9);
    // Leaving with unsaved edits is said.
    CHECK(f.run({"Pens", "Set", "3", "", "", "", "", ""}) == EngineStatus::Finished);
    CHECK(contains(f.out(), "not saved"));

    // Save asks before replacing, as the job's does.
    f.run({"Pens", "Load", f.path("pens.lpt"), "Save", f.path("pens.lpt"), "No"});
    CHECK(contains(f.out(), "Not saved"));
}

TEST_CASE("laser: Pulse asks, then fires through the transport") {
    Fixture f;
    f.run({"Device", "bench", "Pulse", "10", "100"});
    CHECK(contains(f.out(), "[Yes/No] <No>"));
    CHECK(f.fake.job.empty());  // default answer: nothing fired

    f.run({"Pulse", "10", "100", "Yes"});
    CHECK(contains(f.out(), "Fired 10% for 100 ms"));
    CHECK(!f.fake.job.empty());

    f.fake.job.clear();
    f.run({"Pulse", "10", "5000", "Yes"});  // out of the 10..2000 ms range
    CHECK(contains(f.out(), "error:"));
    CHECK(f.fake.job.empty());
}

TEST_CASE("laser: a whole LISP script: Pens, Job, Save, Device, Send, leaving the mode") {
    Fixture f;
    f.draw();
    f.db.sysvars().set_int(f.session->confirm, 0);

    ncad::lisp::Context ctx;
    ncad::lisp::Interp in{ctx};
    std::ostringstream out;
    in.set_output(&out);
    in.set_database(&f.db);
    in.set_command_engine(&f.engine);

    const std::string ljob = f.path("x.ljob");
    const std::string script = "(command \"LASER\" \"Pens\" \"Load\" \"" + f.path("pens.lpt") +
                               "\" \"\" \"Job\" \"\" \"Save\" \"" + ljob + "\" \"Device\" \"bench\" \"Send\" \"\")";
    ncad::lisp::Value v;
    REQUIRE(in.eval_string(script, v));
    // The trailing "" really left the mode: nothing is waiting.
    CHECK(f.engine.status() == EngineStatus::Finished);
    CHECK(contains(f.read(ljob), "ljob 1"));
    CHECK(!f.fake.job.empty());

    // Without it the mode is still there.
    REQUIRE(in.eval_string(R"((command "LASER" "Origin" ""))", v));
    CHECK(f.engine.status() == EngineStatus::Waiting);
}

// --- Move over the real transport, against the fake controller -------------------------

namespace {

using ncad::laser::ruida::kMemPosX;
using ncad::laser::ruida::kMemPosY;

// A top-right machine with the head at (head_x, head_y) mm in machine counts.
struct MoveBench {
    Fixture f{"[bench]\nhost = 192.0.2.1\nbed = 600 400\norigin_corner = tr\n"};
    MoveBench(double head_x, double head_y) {
        f.fake.set_number(kMemPosX, static_cast<std::uint64_t>(head_x * 1000));
        f.fake.set_number(kMemPosY, static_cast<std::uint64_t>(head_y * 1000));
    }
    static std::string goto_bytes(double mx, double my) {
        std::string plain, error;
        CHECK(ncad::laser::ruida::prepare_goto(mx, my, 0.0, plain, error));
        return plain;
    }
    int count(const std::string& plain) const {
        int n = 0;
        for (const auto& p : f.fake.packets) n += p == plain;
        return n;
    }
    int reads_of_x() const {
        int n = 0;
        for (const auto& p : f.fake.packets) n += p.size() == 4 && p[0] == '\xDA' && p[1] == '\x00' && p.substr(2) == std::string("\x04\x21", 2) ? 1 : 0;
        return n;
    }
};

}  // namespace

TEST_CASE("laser: Move jog sends an Offset move in bed directions") {
    MoveBench b(500, 300);
    b.f.run({"Device", "bench", "Move", "+X"});
    // Bed +X on a right-origin machine is machine -X.
    CHECK(b.count(MoveBench::goto_bytes(499, 300)) == 1);
    CHECK(contains(b.f.out(), "head at machine 499.000,300.000"));
}

TEST_CASE("laser: Move Goto from the current position anchors once and maps through the machine") {
    MoveBench b(500, 300);
    // Origin 0,0 (no UCS LASER): (10,20) is 10 right and 20 up of the anchor,
    // which on a top-right machine is -10 and -20 in machine counts.
    b.f.run({"Device", "bench", "Move", "Goto", "10,20", "Goto", "10,20"});
    CHECK(b.count(MoveBench::goto_bytes(490, 280)) == 2);  // same bytes, no drift
    CHECK(b.reads_of_x() >= 1);
    // The anchor is not re-read by the second Goto: only the move's own status
    // read and the single anchor poll remain, so reads grow by one per move.
    const int reads = b.reads_of_x();
    CHECK(reads <= 3);
}

TEST_CASE("laser: a jog does not move the Goto anchor, a new Move visit does") {
    MoveBench b(500, 300);
    b.f.run({"Device", "bench", "Move", "Goto", "0,0", "+X", "Goto", "0,0"});
    CHECK(b.count(MoveBench::goto_bytes(500, 300)) == 2);  // fake head never moved; anchor held
    b.f.fake.set_number(kMemPosX, 400000);
    b.f.fake.packets.clear();
    b.f.run({"Device", "bench", "Move", "Goto", "0,0"});
    CHECK(b.count(MoveBench::goto_bytes(400, 300)) == 1);
}

TEST_CASE("laser: Move Goto with Start From Absolute uses bed coordinates") {
    MoveBench b(500, 300);
    b.f.run({"Origin", "Absolute", "Move", "Goto", "100,50"});
    CHECK(b.count(MoveBench::goto_bytes(500, 350)) == 1);  // 600-100, 400-50
}

TEST_CASE("laser: Move Goto is refused when the head position cannot be read, jog still works") {
    MoveBench b(500, 300);
    b.f.fake.mute.insert(kMemPosX);
    b.f.run({"Device", "bench", "Move", "Goto", "10,20", "+X"});
    CHECK(contains(b.f.out(), "cannot read the head position"));
}

TEST_CASE("laser: Order is a sub-mode: List, Pick, Start, Clear, re-planning the job") {
    Fixture f;
    f.run({"Pens", "Load", f.path("pens.lpt")});
    // Three short lines the planner cuts left to right, and a square.
    std::vector<ncad::Handle> h;
    for (int i = 0; i < 3; ++i) {
        auto l = std::make_unique<ncad::Line>(Vec3{20.0 * (i + 1), 0, 0}, Vec3{20.0 * (i + 1), 5, 0});
        l->props().color = 1;
        h.push_back(f.db.add(std::move(l)));
    }
    auto sq = std::make_unique<ncad::Polyline>();
    sq->add({100, 100, 0});
    sq->add({110, 100, 0});
    sq->add({110, 110, 0});
    sq->add({100, 110, 0});
    sq->set_closed(true);
    sq->props().color = 1;
    const ncad::Handle hsq = f.db.add(std::move(sq));
    f.run({"Job", "Extents"});
    REQUIRE(f.session->has_job);

    auto position = [&](ncad::Handle handle) {
        for (std::size_t i = 0; i < f.session->job.paths.size(); ++i) {
            if (f.session->job.paths[i].source == handle) return i;
        }
        return f.session->job.paths.size();
    };
    CHECK(f.session->job.paths.size() == 4);
    CHECK(position(h[0]) < position(h[1]));
    CHECK(position(h[1]) < position(h[2]));

    // The prompt and its abbreviations.
    f.run_raw({"Order"});
    CHECK(contains(f.engine.prompt().message, "Order [List/Pick/Start/Clear] <done>"));
    check_abbreviations(f.engine.prompt());

    // List prints the sequence, with ids; Enter leaves the sub-mode.
    CHECK(f.run({"Order", "List"}) == EngineStatus::Finished);
    CHECK(contains(f.out(), "Cutting order: 4 paths"));
    CHECK(contains(f.out(), " id "));

    // Pick: the third line, then the first. The second is optimised around.
    CHECK(f.run({"Order", "Pick", std::to_string(h[2]), std::to_string(h[0]), ""}) == EngineStatus::Finished);
    CHECK(contains(f.out(), "Fixed the order of 2 paths"));
    CHECK(position(h[2]) < position(h[0]));
    CHECK(f.session->overrides.steps.size() == 2);

    // Clear puts it back.
    f.run({"Order", "Clear"});
    CHECK(contains(f.out(), "Cleared 2 overrides"));
    CHECK(position(h[0]) < position(h[2]));

    // Start: a point near the square's (110,110) corner, cut the other way.
    f.run({"Order", "Start", "109,109", "Yes"});
    CHECK(contains(f.out(), "starts at vertex 2, reversed"));
    const std::size_t at = position(hsq);
    REQUIRE(at < f.session->job.paths.size());
    const auto& pts = f.session->job.paths[at].pts;
    CHECK(pts[0].x == 110.0);
    CHECK(pts[0].y == 110.0);
    CHECK(pts[1].x == 110.0);
    CHECK(pts[1].y == 100.0);
    // Only the start is fixed: no place in the order was.
    REQUIRE(f.session->overrides.steps.size() == 1);
    CHECK(f.session->overrides.steps[0].free_order);

    // Picking something that is not in the job says so and stays in Pick.
    f.run_raw({"Order", "Pick", "99999"});
    CHECK(contains(f.engine.prompt().message, "no path in the job"));

    // A new Job renumbers paths, so it drops the overrides.
    f.run({"Job", "Extents"});
    CHECK(f.session->overrides.empty());

    // No job, no Order.
    Fixture g;
    g.run({"Order"});
    CHECK(contains(g.out(), "error: no job"));
}
