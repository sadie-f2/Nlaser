// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// SVG import, neutral of NotoCAD: an SVG becomes shapes in millimetres, Y up,
// that the `svg2ljob` tool flattens into a job and the SVGIN command turns
// into entities. Parsing is nanosvg's, behind src/laser/svg_raw.hpp; none of
// its types appear here.
//
// Every nanosvg path is a chain of cubic Beziers (a line is a degenerate one).
// Lines are recognised again and split from curves, so one path may give
// several shapes: a run of straight segments is a Polyline, a run of curved
// ones is a chain of cubics. A rectangle with one rounded corner is two shapes
// that share an end point, which the planner joins like any other drawing.
//
// Units: the SVG's width/height (mm, in, ...) and viewBox give the scale; a
// bare-px or unitless SVG is `px_per_in` pixels to the inch (96; Illustrator
// writes 72). Y is flipped about the page height, so the SVG's lower-left
// corner is the origin and nothing lands at negative Y.
#pragma once

#include "ncad/laser/job.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace ncad::laser {

struct SvgOptions {
    double px_per_in{96.0};
};

// Stroke width at or above this is carried on the polyline as its width, below
// it the stroke is a hairline. 0.5 mm is kRasterWidth, where the drawing
// pipeline starts treating a width as a filled stroke.
inline constexpr double kSvgWideStroke = 0.5;

struct SvgShape {
    std::string layer;       // "0" when the shape sits in no named group
    int aci{7};
    double width{0.0};       // mm; 0 for a hairline
    bool closed{false};
    bool curve{false};
    // Polyline: the points (a closed one does not repeat the first).
    // Curve: the start point, then (control1, control2, end) per segment.
    std::vector<Pt> pts;
};

struct SvgLayer {
    std::string name;
    int aci{7};  // the colour of the first shape on it
};

struct SvgImport {
    double width_mm{0.0};
    double height_mm{0.0};
    std::vector<SvgLayer> layers;  // first-seen order
    std::vector<SvgShape> shapes;
    std::size_t filled{0};         // shapes with a fill: outline only, see warnings
    std::vector<std::string> warnings;
};

bool import_svg(const std::string& text, const SvgOptions& opts, SvgImport& out, std::string& error);
bool import_svg_file(const std::string& path, const SvgOptions& opts, SvgImport& out, std::string& error);

// Points along a cubic Bezier, chord error at most `tol` mm, start included.
void flatten_cubic(const Pt& p0, const Pt& c1, const Pt& c2, const Pt& p1, double tol, std::vector<Pt>& out);

// A raw job: one path per shape (curves flattened at `tol`), pen = ACI, and a
// layer per pen from `pens`; a pen not listed runs at power 0 and is warned
// about in `warnings`. Origin 0,0; plan it next.
bool svg_to_job(const SvgImport& svg, double tol, const std::vector<LaserLayer>& pens, const std::string& source,
                LaserJob& job, std::vector<std::string>& warnings, std::string& error);

}  // namespace ncad::laser
