// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// PDFIN: import a page of a PDF or an Illustrator (CS or later) file as real
// entities, exactly as SVGIN does for an SVG -- same shapes, same layers, same
// one undo step -- from the shapes pdf_import.hpp interprets.
//
//   PDF or AI file to import: art.pdf
//   Page <1>:
//
// Optional-content layers (Illustrator's) become layers of the same name.
// Without qpdf in the build the command fails with "built without qpdf".
#pragma once

#include "ncad/command.hpp"
#include "ncad/laser/laser_command.hpp"
#include "ncad/laser/pdf_import.hpp"

#include <memory>
#include <string>

namespace ncad::laser {

class PdfInCommand final : public Command {
public:
    explicit PdfInCommand(std::shared_ptr<LaserSession> session) : session_(std::move(session)) {}

    const char* name() const override { return "PDFIN"; }
    Step start(CommandContext& ctx) override;
    Step next(CommandContext& ctx, const InputValue& value) override;

private:
    std::shared_ptr<LaserSession> session_;
    std::string path_;
};

}  // namespace ncad::laser
