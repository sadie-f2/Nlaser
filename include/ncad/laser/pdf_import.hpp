// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// PDF and Illustrator (CS and later) import, neutral of NotoCAD. The result is
// the SVG importer's own type (SvgImport / SvgShape, svg_import.hpp), so
// svg_to_job, svg2ljob's flattening and add_svg_to_drawing serve both.
//
// Two halves with one seam between them:
//   * import_pdf_content: an in-tree interpreter for a page's already
//     decompressed content stream. Core, no dependency, fully tested.
//   * import_pdf_file: opens the file, finds the page and hands its content
//     stream to the interpreter. Object parsing and FlateDecode are qpdf's
//     (optional, NLASER_WITH_QPDF); without it the call refuses.
//
// Units: PDF user space is 1/72 in, converted to mm. PDF is already Y up, so
// "the flip" is only moving the MediaBox's lower-left corner to the origin.
// Page /Rotate and CropBox are not applied.
//
// What is imported: path construction (m l c v y h re) and painting. A stroke
// is a path; a fill is outlined with the same warning SVG gives. Stroke width
// is `w` times the CTM's scale. Layers come from optional-content marked
// content (`/OC /Name BDC`, how Illustrator writes layers); with none, one
// layer "0". Text, shadings, images and form XObjects are skipped, with a
// warning each.
#pragma once

#include "ncad/laser/svg_import.hpp"

#include <map>
#include <string>

namespace ncad::laser {

struct PdfPageData {
    // MediaBox, PDF user units: x0, y0, x1, y1 (any corner order).
    double box[4]{0.0, 0.0, 612.0, 792.0};
    std::string content;  // decompressed, all of the page's streams joined
    // The page's /Resources /Properties: name ("MC0") to the optional content
    // group's /Name, the layer name Illustrator wrote.
    std::map<std::string, std::string> properties;
};

bool import_pdf_content(const PdfPageData& page, SvgImport& out, std::string& error);

struct PdfOptions {
    int page{1};  // 1-based
};

// True when this build can open PDF files (NLASER_WITH_QPDF).
bool pdf_container_available();

// An Illustrator file from before CS is PostScript (`%!PS-Adobe`), not PDF.
bool looks_like_postscript(const std::string& head);

bool import_pdf_file(const std::string& path, const PdfOptions& opts, SvgImport& out, std::string& error);

}  // namespace ncad::laser
