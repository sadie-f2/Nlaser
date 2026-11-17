// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// NOT COMPILED OR TESTED YET: written against qpdf's documented C++ API on a
// machine without qpdf. It is built only when CMake finds qpdf
// (NLASER_WITH_QPDF), is kept to the container work -- open, page, content,
// resources -- and leaves everything else to pdf_content.cpp.

#include "pdf_qpdf.hpp"

#include <qpdf/Pl_String.hh>
#include <qpdf/QPDF.hh>
#include <qpdf/QPDFObjectHandle.hh>
#include <qpdf/QPDFPageDocumentHelper.hh>
#include <qpdf/QPDFPageObjectHelper.hh>

#include <exception>
#include <vector>

namespace ncad::laser {

bool qpdf_load_page(const std::string& path, int page, PdfPageData& out, std::string& error) {
    // qpdf reports damage and unsupported features by exception.
    try {
        QPDF pdf;
        pdf.processFile(path.c_str());
        QPDFPageDocumentHelper doc(pdf);
        const std::vector<QPDFPageObjectHelper> pages = doc.getAllPages();
        if (page < 1 || static_cast<std::size_t>(page) > pages.size()) {
            error = "page " + std::to_string(page) + " does not exist; the file has " +
                    std::to_string(pages.size()) + (pages.size() == 1 ? " page" : " pages");
            return false;
        }
        QPDFPageObjectHelper pg = pages[static_cast<std::size_t>(page) - 1];

        // getMediaBox follows /Parent inheritance, which a page tree uses freely.
        const QPDFObjectHandle mb = pg.getMediaBox();
        if (!mb.isArray() || mb.getArrayNItems() != 4) {
            error = "the page has no usable MediaBox";
            return false;
        }
        for (int i = 0; i < 4; ++i) out.box[i] = mb.getArrayItem(i).getNumericValue();

        // The page's content streams, each decoded and joined with the
        // separator qpdf inserts so a token never runs across two of them.
        Pl_String sink("content", nullptr, out.content);
        pg.pipeContents(&sink);
        sink.finish();

        // Illustrator names a layer by an optional content group, which the
        // content stream refers to through /Resources /Properties.
        out.properties.clear();
        const QPDFObjectHandle res = pg.getAttribute("/Resources", false);
        if (res.isDictionary() && res.getKey("/Properties").isDictionary()) {
            const QPDFObjectHandle props = res.getKey("/Properties");
            for (const std::string& key : props.getKeys()) {
                const QPDFObjectHandle ocg = props.getKey(key);
                if (ocg.isDictionary() && ocg.getKey("/Name").isString()) {
                    out.properties[key.substr(1)] = ocg.getKey("/Name").getUTF8Value();
                }
            }
        }
        return true;
    } catch (const std::exception& e) {
        error = e.what();
        return false;
    }
}

}  // namespace ncad::laser
