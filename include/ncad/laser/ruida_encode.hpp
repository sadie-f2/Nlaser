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
// **Start modes** (RdJob::start). Current position (Ref Point 0) is the
// default. Absolute (Ref Point 2 "machine zero" + Set Absolute, LightBurn's
// Start From = Absolute Coords; fixture f03) differs from f01 in the header
// opening and in where the moves lie.
//
// **The pointer offset K** (RdJob::pointer_offset) is the (5.2, 31.2) mm that
// appears in every LightBurn fixture: the machine's red-dot pointer offset,
// configured in LightBurn's device settings (confirmed by the user). The red
// dot is where the panel and the user position things; the beam is K away
// from it. LightBurn adds K to every laser move, in the file's (machine)
// frame, in both modes -- proven by the fixtures:
//   absolute (f03): moves = workspace position + K, where the workspace
//     position is the header box (221.670..261.670 / 211.020..251.020);
//   current (f01 ...): moves = K + (machine(design) - machine(origin)).
// The header's boxes, part bounds and array points are the grid-rounded box in
// the moves' frame MINUS K, in both modes; Document Min Point is the rounded
// box minimum in the moves' frame, so summarize_rd's "origin" is K.
//
// Which operation applies K (the caller sets pointer_offset or leaves it 0):
//   send, dryrun   the beam follows the path: K applied, as LightBurn. Proven
//                  by the fixtures (for send).
//   frame          traces the job box with the RED DOT, so it is the pointer
//                  that must land on the box: K NOT applied. The user's
//                  description; not captured from LightBurn.
//   goto, jog      rapid moves of the pointer to a point: K not applied.
// The older fixtures have other values (the power-test and box-joint files
// have (-0.5, 32.3)), which is why K is a parameter.
//
// **Fills** (RdLayer::kind; f11, f12). A layer's blocks are normal in a Cut
// layer. In a Scan layer (Work Mode Part 2) each settings block opens with
// `Work Mode 1` where a cut layer's has `Layer End`, carries the laser delays
// last and no Through Power, and the layer ends with `Block End` (E7 00), last
// layer or not. In both a Scan and an Offset layer (an offset fill, Work Mode
// Part 0) the whole layer is ONE block pair, the settings block after the
// short block is itself preceded by `Work Mode 6`, and its layer number is 0.
// The rows or rings are one path with its own travel (RdPath::travel). See
// docs/lightburn-fixtures.md.
//
// **Not yet supported, refused rather than guessed:** passes > 1 in the Legacy
// layout, which waits for a LightBurn sample to copy.
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

// LightBurn's layer types as they show in the file. Cut is an ordinary vector
// layer. Scan is a vector fill ("Fill" or "Scan" without an image: f11, f12),
// Offset an offset fill (f12): both make one block pair for the whole layer
// and carry their own travel between rows or rings; see "Fills" below.
enum class RdLayerKind { Cut, Scan, Offset };

// One layer: a "part" in Ruida terms, numbered by order of first use.
struct RdLayer {
    int pen{-1};                  // the job's pen number, for messages only; -1 if not from a pen
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
    // "Through Power" (tubes 1 and 2), percent: the layer's through-power
    // setting (f12: 95 on two layers). Negative is LightBurn's usual raw 1.
    double through_power{-1.0};
    // The LightBurn layer type behind it; see "Fills" in the header comment.
    RdLayerKind kind{RdLayerKind::Cut};
};

struct RdPath {
    std::size_t layer{0};  // index into RdJob::layers
    bool closed{false};    // closed: a final cut back to the first point
    // Fills and offset fills (RdLayerKind::Scan, Offset): the layer's one path
    // carries its own travel. travel[k] says the segment that ends at pts[k]
    // is a Move (laser off) rather than a cut; travel[0] is unused. Empty: every
    // segment cuts. A scan layer's rows alternate: pts[0] the head's first
    // position, pts[1] the end of the first row (cut), pts[2] the start of the
    // next row (Move), pts[3] its end (cut), and so on. `closed` is not used with it.
    std::vector<bool> travel;
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

// The hard power ceiling every encoded job gets unless told otherwise, in
// percent. It protects the tube, so it is built in: it applies with no
// lasers.conf and to `ncad-laser encode`, which has no device at all.
inline constexpr double kDefaultPowerLimit = 80.0;

struct RdJob {
    std::vector<RdLayer> layers;
    // In cutting order. A layer's paths must be contiguous: the planner orders
    // by layer, and the format has one block per layer.
    std::vector<RdPath> paths;
    RdPoint origin;
    // Start mode; see the header comment.
    RdStart start{RdStart::Current};
    // K, the red-dot pointer offset, in mm in the FILE's frame exactly as
    // LightBurn writes it (added after the design is mapped to machine axes,
    // so it is not itself mapped). Both start modes; see the header comment.
    // 0,0 for the operations that move the pointer (frame, goto, jog).
    RdPoint pointer_offset;
    // How the machine counts from its own origin corner. Points and `origin`
    // are DESIGN coordinates; every coordinate written (moves, header boxes,
    // part bounds, the origin) is first mapped to machine coordinates, so a
    // planned job never depends on the machine. The default is the identity.
    RdMachine machine;
    // New jobs use the layout of the newest LightBurn that produced samples,
    // which is what the controllers in the field are being fed today; Legacy
    // exists to keep reproducing the older fixtures byte for byte.
    RdLayout layout{RdLayout::Current};
    // Current layout only: LightBurn 1.4.02's version of it (f04-user,
    // f08..f13). Every path's Move comes BEFORE its short block and settings
    // block (1.4.03, f01..f07: after the first pass's pair), and the first path
    // of the file has one more short block plus settings block before its own
    // pair. Once per file, not per layer or path.
    bool lead_block{false};
    // Current layout only. The "Speed Axis" LightBurn writes before every
    // Layer End: 15 mm/s in every fixture, whatever the layer's own speed. It
    // reads as a device setting (the axis/jog speed), so it is a field here
    // rather than a constant.
    double axis_speed_mm_s{15.0};
    // Every motion is written as a MOVE (laser off) and every power as 0:
    // the file contains no cut command at all. This is how Frame traces the
    // job's box -- as a job, so it lands exactly where the job will.
    bool travel_only{false};
    // The cut-length counter near the end of the file, in whole mm, written
    // instead of the one computed from the cuts when >= 0. LightBurn's value
    // for f12 is 1 mm below the sum of the cuts it wrote (an offset fill is in
    // that file; why is not understood), so a test can pin the file's own value.
    double cut_length_mm{-1.0};
    // Hard ceiling in percent (DeviceConfig::power_limit). encode_rd refuses
    // any layer power above it, tube 1 and tube 2, so no caller can bypass it.
    // Not checked for travel_only jobs: every power in those is written as 0.
    double power_limit{kDefaultPowerLimit};
};

// False with `error` set when the job cannot be encoded as asked; `plain` is
// then untouched.
bool encode_rd(const RdJob& job, std::string& plain, std::string& error);

}  // namespace ncad::laser::ruida
