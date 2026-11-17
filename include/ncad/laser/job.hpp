// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// The controller-independent laser job, and its text form `.ljob`.
//
// Every driver consumes a LaserJob; the planner reads and writes one. The text
// form is the inspectable interchange -- what DXF text is to drawings -- so a
// job can be built in NotoCAD, read or edited by hand, and sent later. Tests
// compare its bytes.
//
//   ljob 1 name="tag-plate" source="tag.dxf" planned start=current
//   bed 900 600
//   origin 10 10
//   layer 5 color=5 mode=auto speed=150 power=18,18 passes=1 air=on order=1
//   path 5 closed
//     10.000 10.000
//     90.000 10.000
//   end
//
// Millimetres, X right and Y up. A layer is identified by its pen number,
// which defaults to the ACI colour. `#` starts a comment; blank lines are
// ignored. `origin` is the job origin: the point the controller puts at the
// head when the job starts. Absent, it is 0,0 -- the world origin, which is
// where a drawing with no LASER UCS puts it.
//
// `raster` records are part of the format but not of this version: the
// reader refuses them by line number rather than dropping them.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace ncad::laser {

struct Pt {
    double x{0.0};
    double y{0.0};
};

// Where the controller puts the job: at the head's current position (the
// default), or at absolute machine coordinates. `ljob` header field
// start=current|absolute|user (user: the user origin set on the panel; placed
// like current, but the file opens with Ref Point 1).
enum class StartMode : std::uint8_t { Current, Absolute, User };

enum class LayerMode : std::uint8_t { Auto, Vector, Raster, Skip };

const char* layer_mode_name(LayerMode m);

struct LaserLayer {
    int pen{7};
    int color{7};  // ACI
    LayerMode mode{LayerMode::Auto};
    double speed{100.0};      // mm/s
    double min_power{0.0};    // percent
    double max_power{0.0};
    bool has_power2{false};   // tube 2; absent means "as tube 1"
    double min_power2{0.0};
    double max_power2{0.0};
    int passes{1};
    bool air{true};
    int order{0};             // lower cuts first; ties by pen
};

struct LaserPath {
    int pen{7};
    bool closed{false};
    std::vector<Pt> pts;
};

struct LaserJob {
    int version{1};
    std::string name;
    std::string source;
    bool planned{false};
    StartMode start{StartMode::Current};
    double bed_w{0.0};  // 0 = not stated
    double bed_h{0.0};
    Pt origin{};
    std::vector<LaserLayer> layers;
    std::vector<LaserPath> paths;

    const LaserLayer* layer(int pen) const;
};

std::string write_ljob(const LaserJob& job);

// False with `error` naming the line when the text is not a valid job; `job`
// is then untouched.
bool read_ljob(const std::string& text, LaserJob& job, std::string& error);

bool read_ljob_file(const std::string& path, LaserJob& job, std::string& error);
bool write_ljob_file(const std::string& path, const LaserJob& job, std::string& error);

// ACI index to 0xRRGGBB, for the colour a controller shows. NotoCAD's display
// palette (src/gui/qpainter_renderer.cpp), with 7 as black rather than white:
// on a light panel that is how 7 is drawn.
std::uint32_t aci_rgb(int index);

// Where LightBurn's DXF import puts an ACI colour (f12): its layer number (the
// C-number of its palette) and the colour that layer has on the panel, as
// written in the .rd. Only the colours f12 measured are known; false for the
// rest. `rgb` is 0 for layer 5 (it was disabled in f12, so its colour was not seen).
bool lightburn_layer_of_aci(int aci, int& layer, std::uint32_t& rgb);

}  // namespace ncad::laser
