// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

#include "ncad/laser/pdfin_command.hpp"

#include "ncad/laser/svgin_command.hpp"

#include <cstdio>

namespace ncad::laser {

Step PdfInCommand::start(CommandContext&) {
    if (!session_) return Step::failed("the laser backend is not installed");
    // Refuse before asking anything, so a build without qpdf does not make the
    // user pick a file it cannot read.
    if (!pdf_container_available()) {
        return Step::failed("PDFIN: built without qpdf; PDF and AI import need NLASER_WITH_QPDF");
    }
    path_.clear();
    Prompt p;
    p.kind = PromptKind::String;
    p.message = "PDF or AI file to import";
    p.file = FileIntent::Open;
    p.file_extension = "pdf";
    return Step::ask(std::move(p));
}

Step PdfInCommand::next(CommandContext& ctx, const InputValue& value) {
    PdfOptions opts;
    if (path_.empty()) {
        if (value.kind != InputKind::String || value.text.empty()) return Step::cancelled();
        path_ = value.text;
        Prompt p;
        p.kind = PromptKind::Integer;
        p.message = "Page <1>";
        return Step::ask(std::move(p));
    }
    if (value.kind == InputKind::Cancel) return Step::cancelled();
    if (value.kind == InputKind::Integer) opts.page = value.integer;

    SvgImport shapes;
    std::string error;
    if (!import_pdf_file(path_, opts, shapes, error)) return Step::failed(error);

    const SvgInReport rep = add_svg_to_drawing(ctx.db, shapes);
    char head[160];
    std::snprintf(head, sizeof head, "PDFIN: %zu polylines, %zu splines, %zu new layers (%.1f x %.1f mm)",
                  rep.polylines, rep.splines, rep.layers_created, shapes.width_mm, shapes.height_mm);
    std::string msg = head;
    for (const std::string& w : shapes.warnings) msg += "\nwarning: " + w;
    return Step::done(msg);
}

}  // namespace ncad::laser
