// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// The Ruida .rd encoder: an ordered vector job in, plain (unswizzled) bytes out.
//
// **The structure is LightBurn's, command for command**, as read from its own
// files with `rd2txt` (tests/fixtures/lightburn_*.rd). MeerK40t's writer was
// the starting point for the command bytes, but its sequence differs from
// LightBurn's and several of its header values are constants copied from some
// other job (see test_ruida_decode.cpp). The test that matters re-encodes
// LightBurn's jobs from their own geometry and settings and gets LightBurn's
// bytes back.
//
// **Coordinates.** Millimetres in, X right and Y up, in any frame the caller
// likes (then mapped to the machine's axes by RdJob::machine); they are rounded once to integer micrometres and every relative move
// is the difference of two rounded positions, so rounding never accumulates.
// The header describes the job's box RELATIVE TO `origin`, which is the point
// the controller puts at the head's position when the job starts (Start From
// "current position"). LightBurn always puts the origin on a corner of the
// box; here it may be anywhere, which is what lets two partial jobs register
// with each other. Whether the controller honours an origin off the box's
// corners is a hardware test still to run (frame, laser off).
//
// **Passes** (Current layout): per path, `Move`, then once per pass a short
// block, the settings block and the path's cuts (f02-p2/p3, f07). An OPEN path
// has no sample: the encoder travels back to its start before each further
// pass, which is a guess (a closed path needs nothing: it ends where it began).
//
// **Not yet supported, refused rather than guessed:** start modes other than
// current position, and passes > 1 in the Legacy layout. Each waits for a
// LightBurn sample to copy.
#pragma once

#include "ncad/laser/ruida_machine.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace ncad::laser::ruida {

struct RdPoint {
    double x{0.0};
    double y{0.0};
};

// One layer: a "part" in Ruida terms, numbered by order of first use.
struct RdLayer {
    std::uint32_t rgb{0x000000};  // 0xRRGGBB, shown on the panel
    double speed_mm_s{100.0};
    double min_power{0.0};        // percent, tube 1
    double max_power{0.0};
    // Tube 2. Negative means "same as tube 1". LightBurn writes its own
    // second-laser settings here even on a single-tube machine.
    double min_power2{-1.0};
    double max_power2{-1.0};
    bool air{true};
    int passes{1};
};

struct RdPath {
    std::size_t layer{0};  // index into RdJob::layers
    bool closed{false};    // closed: a final cut back to the first point
    std::vector<RdPoint> pts;
};

// The two layer layouts LightBurn has written.
enum class RdLayout {
    // lightburn_power_test*.rd, lightburn_box_joint.rd: each layer opens with
    // "Layer End" and its settings; no passes.
    Legacy,
    // tests/fixtures/lightburn/f*.rd (LightBurn, 2026-11): a Speed Axis before
    // every Layer End, and each pass of each path is its own short block
    // (Work Mode 6 ... Tube Start 3) plus the full settings block plus the cuts.
    Current,
};

struct RdJob {
    std::vector<RdLayer> layers;
    // In cutting order. A layer's paths must be contiguous: the planner orders
    // by layer, and the format has one block per layer.
    std::vector<RdPath> paths;
    RdPoint origin;
    // How the machine counts from its own origin corner. Points and `origin`
    // are DESIGN coordinates; every coordinate written (moves, header boxes,
    // part bounds, the origin) is first mapped to machine coordinates, so a
    // planned job never depends on the machine. The default is the identity.
    RdMachine machine;
    // New jobs use the layout of the newest LightBurn that produced samples,
    // which is what the controllers in the field are being fed today; Legacy
    // exists to keep reproducing the older fixtures byte for byte.
    RdLayout layout{RdLayout::Current};
    // Current layout only. The "Speed Axis" LightBurn writes before every
    // Layer End: 15 mm/s in every fixture, whatever the layer's own speed. It
    // reads as a device setting (the axis/jog speed), so it is a field here
    // rather than a constant.
    double axis_speed_mm_s{15.0};
    // Every motion is written as a MOVE (laser off) and every power as 0:
    // the file contains no cut command at all. This is how Frame traces the
    // job's box -- as a job, so it lands exactly where the job will.
    bool travel_only{false};
};

// False with `error` set when the job cannot be encoded as asked; `plain` is
// then untouched.
bool encode_rd(const RdJob& job, std::string& plain, std::string& error);

}  // namespace ncad::laser::ruida
