// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// LightBurn's project format (.lbrn2, XML) as a LaserJob, in and out. What the
// samples show, and what is still unknown, is in docs/lbrn2-format.md; this
// header records only the decisions.
//
// - Layers: each <CutSetting type="Cut"> is a LaserLayer whose pen is
//   LightBurn's layer index (<index>). The ACI colour is derived from the
//   index by the one mapping the samples show (index 2 is red, ACI 1); any
//   other index gets ACI 7 until more samples say otherwise.
// - Shapes: Path (line segments; cubic Béziers flattened) and Ellipse become
//   LaserPaths. Anything else (Rect, Group, Text, Bitmap, ...) is refused by
//   name and line, never dropped.
// - Frame: LightBurn's workspace coordinates are MACHINE coordinates (see
//   docs/lightburn-fixtures.md, "Bed coordinates"). The reader maps them to the
//   design frame (X right, Y up) with the machine origin corner the caller
//   supplies; the writer maps back. With the bed size known the result is bed
//   coordinates; without it, a machine that counts from the right or top edge
//   gets plain negation (positions then are only meaningful relative to each
//   other, which is all a start-from-current job needs).
// - The project stores neither the start mode nor the job origin (they are
//   LightBurn UI state, not in the samples), so both come from the options.
#pragma once

#include "ncad/laser/job.hpp"
#include "ncad/laser/ruida_machine.hpp"

#include <string>
#include <utility>
#include <vector>

namespace ncad::laser {

// The nine-dot job origin of LightBurn's UI; only these five are offered.
enum class JobOrigin { BottomLeft, BottomRight, TopLeft, TopRight, Centre };

// "bl" "br" "tl" "tr" "c"; false for anything else.
bool parse_job_origin(const std::string& s, JobOrigin& out);

struct Lbrn2Options {
    // Mapping between LightBurn's workspace and the design frame; also gives
    // the bed (bed_w_mm/bed_h_mm) when absolute positions matter.
    ruida::RdMachine machine;
    // Chord tolerance for curves, mm.
    double tolerance{0.01};
    // Reader only: not in the file, see above.
    StartMode start{StartMode::Current};
    JobOrigin job_origin{JobOrigin::BottomLeft};
};

struct Lbrn2Info {
    std::string app_version;  // AppVersion, "" if absent
    int format_version{0};
    bool mirror_x{false};     // MirrorX / MirrorY on the project
    bool mirror_y{false};
    // LightBurn's name for each layer index, in file order.
    std::vector<std::pair<int, std::string>> layer_names;
    // What the job got from the options rather than from the file.
    StartMode start{StartMode::Current};
    JobOrigin job_origin{JobOrigin::BottomLeft};
    bool placement_from_file{false};  // always false: the samples carry neither
    // Bounding box of all geometry in LightBurn's workspace, mm; zero if no shapes.
    Pt workspace_min{};
    Pt workspace_max{};
    std::size_t shapes{0};
    // Things that were understood but ignored, one line each.
    std::vector<std::string> warnings;
};

// The LightBurn layer index to ACI colour mapping the samples support.
int lbrn2_index_to_aci(int index);

// False with `error` ("line N: ...") when the text is not a project this
// reader understands; `job` and `info` are then untouched.
bool read_lbrn2(const std::string& text, LaserJob& job, Lbrn2Info& info, const Lbrn2Options& opts,
                std::string& error);

bool read_lbrn2_file(const std::string& path, LaserJob& job, Lbrn2Info& info, const Lbrn2Options& opts,
                     std::string& error);

// A project LightBurn can (we believe) open: validated only by round trip and
// by comparison with the samples' structure, never by LightBurn itself.
// Refuses what the samples do not let us spell: a skip or raster layer, air
// off. `names`, when given, supplies layer names (from read_lbrn2).
bool write_lbrn2(const LaserJob& job, const Lbrn2Options& opts, std::string& out, std::string& error,
                 const Lbrn2Info* names = nullptr);

bool write_lbrn2_file(const std::string& path, const LaserJob& job, const Lbrn2Options& opts,
                      std::string& error, const Lbrn2Info* names = nullptr);

}  // namespace ncad::laser
