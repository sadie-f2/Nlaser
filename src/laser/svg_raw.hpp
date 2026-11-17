// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// What nanosvg gives us, copied into plain structs. Private to src/laser: the
// one translation unit that includes nanosvg.h (svg_nanosvg.cpp) is compiled
// with warnings off, so nothing of its types may leak past this boundary and
// every line of Nlaser's own import logic stays under the normal flags.
#pragma once

#include <string>
#include <vector>

namespace ncad::laser {

struct RawPath {
    std::vector<float> pts;  // x,y pairs: a start point, then 3 per cubic Bezier
    bool closed{false};
};

enum class RawPaint { None, Color, Gradient };

struct RawShape {
    std::string group;  // innermost named group, "" if none
    RawPaint stroke{RawPaint::None};
    RawPaint fill{RawPaint::None};
    unsigned stroke_rgb{0};  // 0xRRGGBB
    unsigned fill_rgb{0};
    float stroke_width{0.0f};
    bool dashed{false};
    bool visible{true};
    std::vector<RawPath> paths;
};

struct RawImage {
    float width{0.0f};   // in `units`, as asked of the parser
    float height{0.0f};
    std::vector<RawShape> shapes;
};

// Parses SVG `text` and returns everything in millimetres (nanosvg's own
// units="mm" scaling, with `dpi` pixels per inch for bare px). Y is still down.
bool parse_svg_raw(std::string text, float dpi, RawImage& out, std::string& error);

}  // namespace ncad::laser
