// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// The seam to qpdf. Declared always, defined in pdf_qpdf.cpp only when qpdf was
// found; pdf_import.cpp calls it only under NLASER_HAVE_QPDF.
#pragma once

#include "ncad/laser/pdf_import.hpp"

namespace ncad::laser {

// Page `page` (1-based) of the file: MediaBox, decompressed content, and the
// /Properties names.
bool qpdf_load_page(const std::string& path, int page, PdfPageData& out, std::string& error);

}  // namespace ncad::laser
