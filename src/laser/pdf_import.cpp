// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

#include "ncad/laser/pdf_import.hpp"

#ifdef NLASER_HAVE_QPDF
#include "pdf_qpdf.hpp"
#endif

#include <cctype>
#include <fstream>

namespace ncad::laser {

bool pdf_container_available() {
#ifdef NLASER_HAVE_QPDF
    return true;
#else
    return false;
#endif
}

bool looks_like_postscript(const std::string& head) {
    std::size_t i = 0;
    while (i < head.size() && std::isspace(static_cast<unsigned char>(head[i]))) ++i;
    return head.compare(i, 10, "%!PS-Adobe") == 0;
}

bool import_pdf_file(const std::string& path, const PdfOptions& opts, SvgImport& out, std::string& error) {
    // Read the head first: the PostScript refusal needs no qpdf, and the user
    // should hear it rather than "built without qpdf" for a file qpdf could
    // never have opened.
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        error = "cannot read '" + path + "'";
        return false;
    }
    std::string head(1024, '\0');
    in.read(head.data(), static_cast<std::streamsize>(head.size()));
    head.resize(static_cast<std::size_t>(in.gcount()));
    if (looks_like_postscript(head) && head.find("%PDF-") == std::string::npos) {
        error = path + ": Illustrator files before CS are PostScript and cannot be imported; "
                       "open it in Illustrator and save as SVG or PDF";
        return false;
    }
    if (opts.page < 1) {
        error = "the page number starts at 1";
        return false;
    }
#ifdef NLASER_HAVE_QPDF
    PdfPageData page;
    std::string detail;
    if (!qpdf_load_page(path, opts.page, page, detail)) {
        error = path + ": " + detail;
        return false;
    }
    if (!import_pdf_content(page, out, error)) {
        error = path + ": " + error;
        return false;
    }
    return true;
#else
    (void)out;
    error = "built without qpdf: PDF and AI import need NLASER_WITH_QPDF "
            "(pdfstream2ljob reads an already decompressed content stream without it)";
    return false;
#endif
}

}  // namespace ncad::laser
