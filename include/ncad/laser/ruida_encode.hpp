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
// likes; they are rounded once to integer micrometres and every relative move
// is the difference of two rounded positions, so rounding never accumulates.
// The header describes the job's box RELATIVE TO `origin`, which is the point
// the controller puts at the head's position when the job starts (Start From
// "current position"). LightBurn always puts the origin on a corner of the
// box; here it may be anywhere, which is what lets two partial jobs register
// with each other. Whether the controller honours an origin off the box's
// corners is a hardware test still to run (frame, laser off).
//
// **Not yet supported, refused rather than guessed:** start modes other than
// current position, and passes > 1. Each waits for a LightBurn sample to copy.
#pragma once

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

struct RdJob {
    std::vector<RdLayer> layers;
    // In cutting order. A layer's paths must be contiguous: the planner orders
    // by layer, and the format has one block per layer.
    std::vector<RdPath> paths;
    RdPoint origin;
    // Every motion is written as a MOVE (laser off) and every power as 0:
    // the file contains no cut command at all. This is how Frame traces the
    // job's box -- as a job, so it lands exactly where the job will.
    bool travel_only{false};
};

// False with `error` set when the job cannot be encoded as asked; `plain` is
// then untouched.
bool encode_rd(const RdJob& job, std::string& plain, std::string& error);

}  // namespace ncad::laser::ruida
