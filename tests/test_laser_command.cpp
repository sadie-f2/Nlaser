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

    // Starts LASER and answers with `tokens`, in order.
    EngineStatus run(std::vector<std::string> tokens) {
        ncad::TextInputSource src(std::move(tokens));
        src.set_database(&db);
        engine.begin(ncad::make_command("LASER"));
        return engine.run(src);
    }

    void use_pens() { run({"Pens", path("pens.lpt")}); }
};

bool contains(const std::string& s, const char* part) { return s.find(part) != std::string::npos; }

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
    CHECK(f.run({"Device", "nonesuch"}) == EngineStatus::Failed);
    CHECK(f.db.sysvars().get_string(f.session->dev).empty());

    CHECK(f.run({"Device", "a2"}) == EngineStatus::Finished);
    CHECK(f.db.sysvars().get_string(f.session->dev) == "a2");
    CHECK(contains(f.engine.message(), "192.0.2.2"));

    // Enter at the prompt reports and changes nothing.
    CHECK(f.run({"Device", ""}) == EngineStatus::Finished);
    CHECK(f.db.sysvars().get_string(f.session->dev) == "a2");
}

TEST_CASE("laser: Pens validates the file and sets LASERPENS") {
    Fixture f;
    CHECK(f.run({"Pens", f.path("missing.lpt")}) == EngineStatus::Failed);
    CHECK(f.db.sysvars().get_string(f.session->pens).empty());
    CHECK(f.run({"Pens", f.path("pens.lpt")}) == EngineStatus::Finished);
    CHECK(f.db.sysvars().get_string(f.session->pens) == f.path("pens.lpt"));
    CHECK(f.run({"Pens", "."}) == EngineStatus::Finished);
    CHECK(f.db.sysvars().get_string(f.session->pens).empty());
}

TEST_CASE("laser: Job builds, plans and reports; Save .ljob matches the pipeline byte for byte") {
    Fixture f;
    f.draw();
    f.use_pens();

    // Enter takes the default option, Job; "Extents" the area.
    CHECK(f.run({"", "Extents"}) == EngineStatus::Finished);
    const std::string& msg = f.engine.message();
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
    CHECK(f.run({"Job", "Window", "500,500", "600,600"}) == EngineStatus::Failed);
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
    CHECK(f.run({"Job", "Extents"}) == EngineStatus::Failed);
}

TEST_CASE("laser: a single straight line is a job, though its extents have no height") {
    Fixture f;
    auto l = std::make_unique<ncad::Line>(Vec3{0, 0, 0}, Vec3{30, 0, 0});
    l->props().color = 1;
    f.db.add(std::move(l));
    CHECK(f.run({"Job", "Extents"}) == EngineStatus::Finished);
    CHECK(f.session->job.paths.size() == 1);
    // Enter at the area prompt is Display; with no view here it is Extents, said so.
    CHECK(f.run({"Job", ""}) == EngineStatus::Finished);
    CHECK(contains(f.engine.message(), "No display here"));
}

TEST_CASE("laser: an empty drawing makes no job") {
    Fixture f;
    CHECK(f.run({"Job", "Extents"}) == EngineStatus::Failed);
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
    CHECK(contains(f.engine.message(), "no device chosen"));

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
    CHECK(!contains(f.engine.message(), "no device chosen"));
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
    CHECK(contains(f.engine.message(), "Sent"));

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
    CHECK(contains(f.engine.message(), "Not sent"));
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
    f.run({"Send"});
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
        CHECK(f.run({"Send"}) == EngineStatus::Failed);  // no job
        CHECK(f.fake.packets.empty());
    }
    {
        Fixture f("[bench]\nhost = 192.0.2.1\n[a2]\nhost = 192.0.2.2\n");
        f.draw();
        f.use_pens();
        f.run({"Job", "Extents"});
        CHECK(f.run({"Send"}) == EngineStatus::Failed);  // two devices, none chosen
        CHECK(f.connects == 0);
    }
    {
        Fixture f("[bench]\nhost = 192.0.2.1\nmax_power = 50\n");
        f.draw();
        f.use_pens();
        f.run({"Job", "Window", "-5,-5", "15,15"});
        CHECK(f.run({"Send"}) == EngineStatus::Failed);  // layer 1 asks for 65%
        CHECK(contains(f.engine.message(), "65.0%"));
        CHECK(f.connects == 0);
        CHECK(f.fake.packets.empty());
    }
}

TEST_CASE("laser: Send refuses a job bigger than the bed") {
    Fixture f("[bench]\nhost = 192.0.2.1\nbed = 50 50\n");
    f.draw();
    f.use_pens();
    f.run({"Job", "Extents"});  // 120 x 100
    CHECK(f.run({"Send"}) == EngineStatus::Failed);
    CHECK(contains(f.engine.message(), "bed"));
    CHECK(f.fake.packets.empty());
}

TEST_CASE("laser: a connection failure is reported, not hung on") {
    Fixture f;
    f.draw();
    f.use_pens();
    f.run({"Job", "Window", "-5,-5", "15,15"});
    f.db.sysvars().set_int(f.session->confirm, 0);
    f.fake.silent = true;  // the controller never answers ENQ
    CHECK(f.run({"Send"}) == EngineStatus::Failed);
    CHECK(contains(f.engine.message(), "no answer"));
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
    CHECK(contains(f.engine.message(), "laser off"));
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
    CHECK(contains(f.engine.message(), "Status of bench"));
    CHECK(f.run({"Counters"}) == EngineStatus::Finished);
    CHECK(contains(f.engine.message(), "Counters of bench"));
    CHECK(f.run({"STop"}) == EngineStatus::Finished);
    CHECK(f.fake.stops == 1);
}

TEST_CASE("laser: Order List prints the planned sequence; the rest is not built") {
    Fixture f;
    f.draw();
    f.use_pens();
    f.run({"Job", "Extents"});
    CHECK(f.run({"Order", "List"}) == EngineStatus::Finished);
    CHECK(contains(f.engine.message(), "Cutting order: 2 paths"));
    CHECK(contains(f.engine.message(), "pen 1  closed"));
    CHECK(f.run({"Order", ""}) == EngineStatus::Finished);

    CHECK(f.run({"Order", "Show"}) == EngineStatus::Failed);
    CHECK(contains(f.engine.message(), "come later"));

    f.run({"Order"});
    REQUIRE(f.engine.status() == EngineStatus::Waiting);
    CHECK(contains(f.engine.prompt().message, "come later"));
}

TEST_CASE("laser: Origin reports the LASER UCS, or world") {
    Fixture f;
    CHECK(f.run({"Origin"}) == EngineStatus::Finished);
    CHECK(contains(f.engine.message(), "0.000,0.000 (world"));

    ncad::Ucs u;
    u.origin = {20, 30, 0};
    f.db.add_ucs("LASER", u);
    CHECK(f.run({"Origin"}) == EngineStatus::Finished);
    CHECK(contains(f.engine.message(), "20.000,30.000 (UCS LASER)"));

    f.draw();
    f.run({"Job", "Extents"});
    CHECK_NEAR(f.session->job.origin.x, 20.0, 1e-9);
    CHECK_NEAR(f.session->job.origin.y, 30.0, 1e-9);
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
    REQUIRE(in.eval_string(R"((command "LASER" "J" "E"))", v));
    REQUIRE(f.session->has_job);
    CHECK(f.session->job.paths.size() == 2);

    REQUIRE(in.eval_string(R"((getvar "LASERTOL"))", v));
    CHECK(ncad::lisp::prin1(v) == "0.01");
    REQUIRE(in.eval_string(R"((setvar "LASERCONFIRM" 0))", v));
    CHECK(f.db.sysvars().get_int(f.session->confirm) == 0);

    // A device operation from LISP, in one call: Device, then Send. ("DE":
    // since Dryrun, a bare "D" is ambiguous.)
    REQUIRE(in.eval_string(R"((command "LASER" "DE" "bench" "LASER" "Send"))", v));
    CHECK(!f.fake.job.empty());
}

TEST_CASE("laser: without a connector a device operation says so") {
    Fixture f;
    f.session->options.connector = nullptr;
    f.draw();
    f.use_pens();
    f.run({"Job", "Extents"});
    f.db.sysvars().set_int(f.session->confirm, 0);
    CHECK(f.run({"Send"}) == EngineStatus::Failed);
    CHECK(contains(f.engine.message(), "network"));
}
