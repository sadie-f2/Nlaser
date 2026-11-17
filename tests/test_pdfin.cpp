// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// PDFIN through NotoCAD's CommandEngine. Without qpdf the file cannot be read,
// so what is checked is registration and the clean refusal; the shapes it
// would add are the SVGIN path (test_svgin.cpp) fed by the tested interpreter.

#include "test.hpp"

#include "ncad/command.hpp"
#include "ncad/commands.hpp"
#include "ncad/database.hpp"
#include "ncad/input_text.hpp"
#include "ncad/laser/laser_command.hpp"
#include "ncad/laser/pdf_import.hpp"
#include "ncad/sysvar.hpp"

#include <string>

using namespace ncad::laser;
using ncad::CommandEngine;
using ncad::Database;
using ncad::EngineStatus;

TEST_CASE("pdfin: registered, and refuses cleanly without qpdf") {
    Database db;
    CommandEngine engine{db};
    auto session = install_laser(LaserOptions{});
    REQUIRE(session != nullptr);
    CHECK(ncad::make_command("PDFIN") != nullptr);
    if (!pdf_container_available()) {
        ncad::TextInputSource src({});
        src.set_database(&db);
        engine.begin(ncad::make_command("PDFIN"));
        CHECK(engine.run(src) == EngineStatus::Failed);
        CHECK(engine.message().find("built without qpdf") != std::string::npos);
        CHECK(db.order().empty());
    }
    ncad::clear_registered_commands();
    ncad::clear_registered_sysvars();
}
