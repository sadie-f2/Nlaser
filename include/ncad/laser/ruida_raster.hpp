// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// Ruida raster (image engraving) jobs, the way LightBurn writes them. The
// stream is described in docs/lightburn-fixtures.md, "Raster"; the evidence
// is tests/fixtures/lightburn/r01, r02, r03 and r03-pt (.rd and .lbrn2).
//
// In brief: the image becomes a grid of pixels one line interval (25.4/dpi mm)
// square; every scan line is a run of Cut / Move Rel X commands along one
// grid line, alternate lines run in opposite directions, and each line ends
// where the next one starts, so the step between lines is a Move Rel Y.
// Binary rasters (threshold, dither) trim each line to its first and last
// fired pixel and Move over the gaps; greyscale cuts every pixel and sets the
// power of each run with an "Imd Power" pair.
//
// Everything here is in the FILE's frame (machine axes, micrometres, K not
// yet added), exactly as the stream carries it; `RdRasterGeometry` and
// `make_rd_raster` turn a placed image into that frame.
#pragma once

#include "ncad/laser/raster.hpp"
#include "ncad/laser/ruida_decode.hpp"
#include "ncad/laser/ruida_encode.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ncad::laser::ruida {

// A raster in scan order and file orientation.
struct RdRaster {
    std::size_t cols{0};
    std::size_t rows{0};
    // Grid lines, um, ascending. Pixel i spans x_lines[i]..x_lines[i+1].
    std::vector<std::int64_t> x_lines;  // cols + 1
    // Optional: the grid as written in each line, when the lines disagree by a
    // micrometre or so (r01's greyscale lines do). Empty: x_lines for all.
    std::vector<std::vector<std::int64_t>> row_x_lines;  // rows x (cols + 1)
    // The y of scan line k (the beam passes along it), um. LightBurn scans
    // from the highest file y down in every sample (y_lines descending).
    std::vector<std::int64_t> y_lines;  // rows
    // rows x cols, row k = scan line k, column i ascending in file x.
    // Binary: non-zero fires. Greyscale: 0..255, burn strength.
    std::vector<std::uint8_t> level;
    bool grayscale{false};
    // Line 0 runs towards +x (true) or -x (false); later lines alternate.
    // Threshold and Jarvis samples run +x, greyscale and pass-through -x:
    // not understood yet, so it is an input.
    bool first_forward{true};
};

struct RdRasterJob {
    RdLayer layer;          // speed, powers, air, colour; passes must be 1
    RdRaster raster;
    // 0 current position (D8 12), 1 "anchor" (D8 11; the raster samples),
    // 2 machine zero + absolute (D8 10, E6 01).
    int ref_point{0};
    RdPoint pointer_offset;  // K, mm, file frame: the boxes are relative to it
    double axis_speed_mm_s{15.0};
    // The greyscale and pass-through samples write an extra "Work Mode 6"
    // before the Speed Axis of each settings block; threshold and Jarvis do not.
    bool lead_wm6{false};
    double power_limit{kDefaultPowerLimit};
};

// The 14-bit raw power LightBurn writes for greyscale level g (0..255):
// float32 g * (1/254) * 16383, rounded half up, clamped to 16383.
std::uint16_t raster_level_to_raw(std::uint8_t g);

bool encode_rd_raster(const RdRasterJob& job, std::string& plain, std::string& error);

// The inverse of encode_rd_raster for a raster file: recovers the grid, the
// pixels and the settings. False with `error` when the stream is not shaped
// like the samples (rows that do not alternate, a gap in greyscale, ...).
bool raster_job_from_rd(const std::vector<RdCommand>& cmds, const RdSummary& summary, RdRasterJob& out,
                        std::string& error);

}  // namespace ncad::laser::ruida

namespace ncad::laser {

enum class RdRasterMode : std::uint8_t { Threshold, Dither, Grayscale };

// A placed image, in the file's frame. LightBurn's grid phase (where line 0
// sits relative to the image edge) is not derivable from the placement in
// three samples, so it is an input; 0 puts the grid on the image edge.
struct RdRasterGeometry {
    double width_mm{0.0};
    double height_mm{0.0};
    double pitch_um{25400.0 / 154.0};
    // The file's x grows towards the design's left (machine origin on the
    // right) / y grows towards the design's bottom.
    bool mirror_x{false};
    bool mirror_y{false};
    double file_x_lo_um{0.0};  // the image's lowest file x
    double file_y_hi_um{0.0};  // the image's highest file y: the scan starts here
    double x_phase_um{0.0};
    double y_phase_um{0.0};
    bool first_forward{true};
};

struct RdRasterSettings {
    RdRasterMode mode{RdRasterMode::Threshold};
    bool negative{false};      // true: light pixels burn (LightBurn "Negative Image")
    int threshold{128};
    DitherMethod dither{DitherMethod::Jarvis};  // Dither mode
};

// Samples `src` the way LightBurn does for its threshold samples: nearest
// pixel at each grid cell's centre (area averaging was tried: it changes
// edge pixels), then threshold / dither / greyscale.
bool make_rd_raster(const GreyImage& src, const RdRasterGeometry& geo, const RdRasterSettings& set,
                    ruida::RdRaster& out, std::string& error);

}  // namespace ncad::laser
