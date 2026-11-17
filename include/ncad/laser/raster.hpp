// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// Raster engraving up to, but not including, the controller's bytes: load an
// image, resample it to the layer's line interval at its placed size, apply
// levels, dither it to fired pixels, and plan the scan lines. The result is a
// RasterPlan, which is the whole input of the encoder (encode_raster, below):
// the Ruida raster encoding is not known yet, and nothing here guesses it.
//
// Orientation: pixel row 0 is the TOP of the image. The job is Y-up and a
// raster's `at` is its lower-left corner, so row r lies at
// y = at.y + height - (r + 0.5) * pitch_y.
#pragma once

#include "ncad/laser/job.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ncad::laser {

// --- images ------------------------------------------------------------------

struct GreyImage {
    int w{0};
    int h{0};
    std::vector<std::uint8_t> px;  // w*h, row-major, 0 black .. 255 white
};

// Any format stb_image reads (PNG, JPEG, BMP, GIF; a GIF's first frame).
// Luminance is Rec. 601; alpha is composited over white, because a
// transparent background should not burn.
bool decode_grey_image(const std::uint8_t* data, std::size_t size, GreyImage& out, std::string& error);
bool load_grey_image(const std::string& path, GreyImage& out, std::string& error);

// 8-bit greyscale PNG, written with stb_image_write.
bool write_grey_png(const std::string& path, int w, int h, const std::uint8_t* px, std::string& error);

// --- tone and size -----------------------------------------------------------

struct Levels {
    int black{0};       // this grey and darker become black
    int white{255};     // this grey and lighter become white
    double gamma{1.0};  // > 1 lightens the midtones
    bool invert{false};
};

// Levels first, then gamma, then invert.
void apply_levels(GreyImage& img, const Levels& lv);

// Area-averaged when shrinking an axis, bilinear when enlarging it.
GreyImage resample(const GreyImage& src, int out_w, int out_h);

// Pixel counts for a placed size at `dpi` (line interval 25.4/dpi mm); at
// least 1x1. False on a non-positive input or an absurd (> 64 Mpixel) result.
bool raster_pixel_size(double width_mm, double height_mm, double dpi, int& nx, int& ny, std::string& error);

// --- dithering ---------------------------------------------------------------

// What each pixel asks of the laser: 0 off, 255 full. Binary methods give only
// 0 and 255; Grey gives darkness (255 - grey), later scaled between the
// layer's min and max power.
struct RasterBitmap {
    int w{0};
    int h{0};
    std::vector<std::uint8_t> level;
    bool binary{true};
};

struct DitherOptions {
    bool serpentine{true};  // error diffusion alternates direction per row
    int threshold{128};     // Threshold method: grey below this fires
};

RasterBitmap dither(const GreyImage& img, DitherMethod method, const DitherOptions& opts = {});

// --- scan planning -----------------------------------------------------------

// A run of consecutive fired pixels [x0, x1) in one line. `levels` is empty
// for a binary raster (full power) and x1-x0 long for Grey.
struct RasterSpan {
    int x0{0};
    int x1{0};
    std::vector<std::uint8_t> levels;
};

struct RasterLine {
    int row{0};             // image row, 0 = top
    double y{0.0};          // mm
    double x_start{0.0};    // where the head is at the start of the pass, overscan included
    double x_end{0.0};      // the far end of the pass
    bool left_to_right{true};
    std::vector<RasterSpan> spans;  // ascending in x, never empty
};

struct RasterPlanOptions {
    double accel_mm_s2{3000.0};  // the controller's engraving acceleration; to be read from it later
    double margin_mm{1.0};       // overscan beyond the v^2/(2a) ramp
    double travel_mm_s{400.0};   // unidirectional return moves
    bool bidir{true};
};

struct RasterPlan {
    Pt at{};
    double width{0.0};
    double height{0.0};
    int w{0};                 // pixels
    int h{0};
    double pitch_x{0.0};      // mm per pixel
    double pitch_y{0.0};
    int pen{7};
    double speed_mm_s{0.0};
    double min_power{0.0};    // percent; Grey maps level 1..255 onto min..max
    double max_power{0.0};
    bool binary{true};        // false: spans carry per-pixel levels
    bool bidir{true};
    double overscan_mm{0.0};  // v^2/(2a) + margin
    std::vector<RasterLine> lines;  // top row first; blank rows omitted
    std::size_t blank_rows{0};
    std::size_t fired_pixels{0};
    double estimated_s{0.0};
};

// Overscan distance: the head must be at speed before the first fired pixel
// and may only slow after the last, so it travels v^2/(2a) beyond each end,
// plus a margin. Non-positive acceleration leaves just the margin.
double raster_overscan_mm(double speed_mm_s, double accel_mm_s2, double margin_mm);

// Skips blank rows, trims blank margins, orders passes (alternating direction
// when bidir), adds overscan, estimates time. `place` supplies position, size
// and pen; `layer` supplies speed and power; direction comes from opts.bidir.
bool plan_raster(const RasterBitmap& bm, const LaserRaster& place, const LaserLayer& layer,
                 const RasterPlanOptions& opts, RasterPlan& out, std::string& error);

struct RasterPrepOptions {
    Levels levels;
    DitherOptions dither;
    RasterPlanOptions plan;
};

// The whole front half: load place.image, resample to the line interval at the
// placed size, levels, dither, plan (with place.bidir). `bitmap`, when given,
// receives the dithered pixels. A relative image path is taken as written (the
// caller resolves it).
bool prepare_raster(const LaserRaster& place, const LaserLayer& layer, const RasterPrepOptions& opts,
                    RasterPlan& out, RasterBitmap* bitmap, std::string& error);

// --- the encoder seam --------------------------------------------------------

// Where the Ruida raster encoder plugs in: a RasterPlan in, controller bytes
// out. Until the LightBurn samples are decoded this refuses with "raster
// encoding not implemented yet"; nothing else emits raster bytes.
bool encode_raster(const RasterPlan& plan, std::vector<std::uint8_t>& bytes, std::string& error);

}  // namespace ncad::laser
