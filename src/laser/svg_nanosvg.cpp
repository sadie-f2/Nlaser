// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// The only file that includes nanosvg. Its warnings are switched off in
// CMake for this file alone; it holds nothing but the copy out of nanosvg's
// structures, so there is no Nlaser logic here for them to hide a problem in.
#define NANOSVG_IMPLEMENTATION
#include "../../third_party/nanosvg/nanosvg.h"

#include "svg_raw.hpp"

namespace ncad::laser {

namespace {

RawPaint paint_kind(int type) {
    if (type == NSVG_PAINT_COLOR) return RawPaint::Color;
    if (type == NSVG_PAINT_LINEAR_GRADIENT || type == NSVG_PAINT_RADIAL_GRADIENT) return RawPaint::Gradient;
    return RawPaint::None;
}

unsigned rgb_of(unsigned abgr) {
    return ((abgr & 0xFFu) << 16) | (abgr & 0xFF00u) | ((abgr >> 16) & 0xFFu);
}

}  // namespace

bool parse_svg_raw(std::string text, float dpi, RawImage& out, std::string& error) {
    NSVGimage* img = nsvgParse(text.data(), "mm", dpi);  // text is NUL-terminated and nanosvg edits it
    if (!img) {
        error = "not a readable SVG";
        return false;
    }
    RawImage r;
    // nanosvg scales the shapes to `units` but leaves the page size in pixels.
    const float to_mm = 25.4f / dpi;
    r.width = img->width * to_mm;
    r.height = img->height * to_mm;
    for (NSVGshape* s = img->shapes; s; s = s->next) {
        RawShape rs;
        rs.group = s->groupId;
        rs.stroke = paint_kind(s->stroke.type);
        rs.fill = paint_kind(s->fill.type);
        if (rs.stroke == RawPaint::Color) rs.stroke_rgb = rgb_of(s->stroke.color);
        if (rs.fill == RawPaint::Color) rs.fill_rgb = rgb_of(s->fill.color);
        rs.stroke_width = s->strokeWidth;
        rs.dashed = s->strokeDashCount > 0;
        rs.visible = (s->flags & NSVG_FLAGS_VISIBLE) != 0;
        for (NSVGpath* p = s->paths; p; p = p->next) {
            RawPath rp;
            rp.closed = p->closed != 0;
            rp.pts.assign(p->pts, p->pts + 2 * p->npts);
            rs.paths.push_back(std::move(rp));
        }
        r.shapes.push_back(std::move(rs));
    }
    nsvgDelete(img);
    out = std::move(r);
    return true;
}

}  // namespace ncad::laser
