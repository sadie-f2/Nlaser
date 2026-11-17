// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

#include "ncad/laser/svg_import.hpp"

#include "svg_raw.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>

namespace ncad::laser {

namespace {

// How far a Bezier's control points may sit off its chord and still be a
// straight line. nanosvg builds lines with controls at the thirds, so a real
// line is within float rounding of this; a curve flatter than 0.1 micron is a
// line to any laser.
constexpr double kLineEps = 1e-4;
constexpr double kZeroLen = 1e-6;

double dist(const Pt& a, const Pt& b) { return std::hypot(a.x - b.x, a.y - b.y); }

// Distance from p to the line through a and b (to a alone if they coincide).
double off_line(const Pt& p, const Pt& a, const Pt& b) {
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double len = std::hypot(dx, dy);
    if (len < kZeroLen) return dist(p, a);
    return std::fabs((p.x - a.x) * dy - (p.y - a.y) * dx) / len;
}

Pt mid(const Pt& a, const Pt& b) { return {(a.x + b.x) / 2.0, (a.y + b.y) / 2.0}; }

void flatten_rec(const Pt& p0, const Pt& c1, const Pt& c2, const Pt& p1, double tol, int depth, std::vector<Pt>& out) {
    // A cubic strays from its chord by at most 3/4 of its control points'
    // distance from it.
    const double d = std::max(off_line(c1, p0, p1), off_line(c2, p0, p1));
    if (depth >= 18 || 0.75 * d <= tol) {
        out.push_back(p1);
        return;
    }
    const Pt a = mid(p0, c1), b = mid(c1, c2), c = mid(c2, p1);
    const Pt ab = mid(a, b), bc = mid(b, c);
    const Pt m = mid(ab, bc);
    flatten_rec(p0, a, ab, m, tol, depth + 1, out);
    flatten_rec(m, bc, c, p1, tol, depth + 1, out);
}

struct Seg {
    Pt c1, c2, end;
    bool line{false};
};

std::string count_of(std::size_t n, const char* one, const char* many) {
    return std::to_string(n) + " " + (n == 1 ? one : many);
}

// The shapes one nanosvg path makes: runs of lines and runs of curves.
void split_path(const RawPath& rp, double height, const std::string& layer, int aci, double width,
                std::vector<SvgShape>& out, std::size_t& wide_curves) {
    const std::size_t n = rp.pts.size() / 2;
    if (n < 4 || n % 3 != 1) return;
    // nanosvg's Y is down; flip about the page height so the page's lower-left
    // corner is the origin.
    auto pt = [&](std::size_t i) { return Pt{rp.pts[2 * i], height - rp.pts[2 * i + 1]}; };

    std::vector<Seg> segs;
    const Pt start = pt(0);
    Pt cur = start;
    for (std::size_t i = 0; i + 3 < n; i += 3) {
        Seg g;
        g.c1 = pt(i + 1);
        g.c2 = pt(i + 2);
        g.end = pt(i + 3);
        g.line = std::max(off_line(g.c1, cur, g.end), off_line(g.c2, cur, g.end)) <= kLineEps;
        // A zero-length line is nanosvg closing a path that already ends where
        // it began; it draws nothing.
        if (g.line && dist(cur, g.end) < kZeroLen) continue;
        segs.push_back(g);
        cur = g.end;
    }
    if (segs.empty()) return;

    std::vector<SvgShape> runs;
    Pt run_start = start;
    for (std::size_t i = 0; i < segs.size();) {
        const bool line = segs[i].line;
        std::size_t j = i;
        while (j < segs.size() && segs[j].line == line) ++j;
        SvgShape sh;
        sh.layer = layer;
        sh.aci = aci;
        sh.curve = !line;
        sh.pts.push_back(run_start);
        for (std::size_t k = i; k < j; ++k) {
            if (!line) {
                sh.pts.push_back(segs[k].c1);
                sh.pts.push_back(segs[k].c2);
            }
            sh.pts.push_back(segs[k].end);
        }
        run_start = segs[j - 1].end;
        runs.push_back(std::move(sh));
        i = j;
    }

    // Closed only when one run is the whole path and it comes back to its
    // start; a polyline then drops the repeated point.
    if (rp.closed && runs.size() == 1 && dist(runs[0].pts.front(), runs[0].pts.back()) < kLineEps) {
        SvgShape& r = runs[0];
        if (r.curve) {
            r.closed = true;
        } else if (r.pts.size() > 3) {
            r.pts.pop_back();
            r.closed = true;
        }
    }
    for (SvgShape& r : runs) {
        if (r.curve) {
            if (width > 0.0) ++wide_curves;
        } else {
            r.width = width;
        }
        out.push_back(std::move(r));
    }
}

}  // namespace

void flatten_cubic(const Pt& p0, const Pt& c1, const Pt& c2, const Pt& p1, double tol, std::vector<Pt>& out) {
    out.push_back(p0);
    flatten_rec(p0, c1, c2, p1, tol, 0, out);
}

bool import_svg(const std::string& text, const SvgOptions& opts, SvgImport& out, std::string& error) {
    if (!(opts.px_per_in > 0.0)) {
        error = "px per inch must be positive";
        return false;
    }
    RawImage raw;
    if (!parse_svg_raw(text, static_cast<float>(opts.px_per_in), raw, error)) return false;

    SvgImport res;
    res.width_mm = raw.width;
    res.height_mm = raw.height;

    std::size_t gradients = 0, dashed = 0, wide_curves = 0, inexact = 0;
    for (const RawShape& s : raw.shapes) {
        if (!s.visible) continue;

        // The colour a shape is cut in: its stroke, or for a fill-only shape
        // the fill's outline. Neither, or a gradient: no colour to map, so 7.
        const bool stroked = s.stroke != RawPaint::None;
        const bool filled = s.fill != RawPaint::None;
        if (!stroked && !filled) continue;
        if (filled) ++res.filled;
        int aci = 7;
        if (stroked ? s.stroke == RawPaint::Color : s.fill == RawPaint::Color) {
            bool exact = false;
            aci = rgb_to_aci(stroked ? s.stroke_rgb : s.fill_rgb, &exact);
            if (!exact) ++inexact;
        } else {
            ++gradients;
        }
        if (s.dashed) ++dashed;
        // nanosvg works in float, so a stroke of exactly 0.5 can arrive a hair
        // under it; the slack keeps the threshold inclusive as documented.
        const double width =stroked && s.stroke_width >= kSvgWideStroke - 1e-4 ? s.stroke_width : 0.0;

        const std::string layer = s.group.empty() ? "0" : s.group;
        if (std::none_of(res.layers.begin(), res.layers.end(), [&](const SvgLayer& l) { return l.name == layer; })) {
            res.layers.push_back({layer, aci});
        }
        for (const RawPath& rp : s.paths) split_path(rp, res.height_mm, layer, aci, width, res.shapes, wide_curves);
    }

    if (res.filled) {
        res.warnings.push_back(count_of(res.filled, "shape has a fill", "shapes have fills") +
                               "; filled regions are raster, which is not supported yet, so " +
                               (res.filled == 1 ? "its outline is" : "their outlines are") + " imported");
    }
    if (gradients) {
        res.warnings.push_back(count_of(gradients, "shape has a gradient", "shapes have gradients") +
                               " and no plain colour: imported in colour 7");
    }
    if (inexact) {
        res.warnings.push_back(count_of(inexact, "colour is", "colours are") +
                               " not in the AutoCAD palette and went to the nearest entry");
    }
    if (dashed) {
        res.warnings.push_back(count_of(dashed, "dashed stroke", "dashed strokes") + " imported as solid");
    }
    if (wide_curves) {
        res.warnings.push_back(count_of(wide_curves, "curve has", "curves have") +
                               " a stroke of 0.5 mm or more; splines carry no width, so " +
                               (wide_curves == 1 ? "it is" : "they are") + " cut on the centre line");
    }
    out = std::move(res);
    return true;
}

bool import_svg_file(const std::string& path, const SvgOptions& opts, SvgImport& out, std::string& error) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        error = "cannot read '" + path + "'";
        return false;
    }
    const std::string text{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    if (!import_svg(text, opts, out, error)) {
        error = path + ": " + error;
        return false;
    }
    return true;
}

bool svg_to_job(const SvgImport& svg, double tol, const std::vector<LaserLayer>& pens, const std::string& source,
                LaserJob& job, std::vector<std::string>& warnings, std::string& error) {
    if (!(tol > 0.0)) {
        error = "the tolerance must be positive";
        return false;
    }
    LaserJob out;
    const std::size_t slash = source.find_last_of("/\\");
    out.source = slash == std::string::npos ? source : source.substr(slash + 1);
    const std::size_t dot = out.source.find_last_of('.');
    out.name = dot == std::string::npos || dot == 0 ? out.source : out.source.substr(0, dot);

    for (const SvgShape& s : svg.shapes) {
        LaserPath p;
        p.pen = s.aci;
        p.closed = s.closed;
        if (!s.curve) {
            p.pts = s.pts;
        } else {
            for (std::size_t i = 0; i + 3 < s.pts.size(); i += 3) {
                std::vector<Pt> seg;
                flatten_cubic(s.pts[i], s.pts[i + 1], s.pts[i + 2], s.pts[i + 3], tol, seg);
                // Each segment starts where the last ended; keep one copy.
                p.pts.insert(p.pts.end(), seg.begin() + (p.pts.empty() ? 0 : 1), seg.end());
            }
            // A closed spline ends on its start point; a closed path is
            // closed by its flag, not by repeating it.
            if (s.closed && p.pts.size() > 1) p.pts.pop_back();
        }
        if (p.pts.size() >= 2) out.paths.push_back(std::move(p));
    }

    std::vector<int> used;
    for (const LaserPath& p : out.paths) used.push_back(p.pen);
    std::sort(used.begin(), used.end());
    used.erase(std::unique(used.begin(), used.end()), used.end());
    std::string unset;
    std::size_t n_unset = 0;
    for (const int pen : used) {
        const auto it = std::find_if(pens.begin(), pens.end(), [pen](const LaserLayer& l) { return l.pen == pen; });
        if (it != pens.end()) {
            out.layers.push_back(*it);
            continue;
        }
        LaserLayer l;
        l.pen = pen;
        l.color = pen;
        l.mode = LayerMode::Auto;
        l.min_power = 0.0;
        l.max_power = 0.0;
        out.layers.push_back(l);
        unset += (unset.empty() ? "" : ", ") + std::to_string(pen);
        ++n_unset;
    }
    warnings = svg.warnings;
    if (n_unset) {
        warnings.push_back(std::string(n_unset == 1 ? "pen " : "pens ") + unset + (n_unset == 1 ? " has" : " have") +
                           " no pen-table entry and will run at power 0");
    }
    job = std::move(out);
    return true;
}

}  // namespace ncad::laser
