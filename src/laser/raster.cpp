// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

#include "ncad/laser/raster.hpp"

#include "ncad/laser/plan.hpp"  // segment_time_s

#include <algorithm>
#include <cmath>

namespace ncad::laser {

namespace {

std::uint8_t clamp8(double v) {
    return static_cast<std::uint8_t>(std::lround(std::clamp(v, 0.0, 255.0)));
}

std::size_t idx(int y, int x, int w) {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
}

// For each output index, the source pixels it draws on and their weights.
struct Taps {
    int first{0};
    std::vector<float> w;
};

// Shrinking: the output pixel covers [i*s, (i+1)*s) of the source, each source
// pixel weighted by its overlap, so every input pixel counts and a checkerboard
// halves to its mean. Enlarging: bilinear between the two nearest pixel
// centres, which does not stair-step the way nearest-neighbour would.
std::vector<Taps> make_taps(int in_n, int out_n) {
    std::vector<Taps> taps(static_cast<std::size_t>(out_n));
    const double s = static_cast<double>(in_n) / out_n;
    for (int i = 0; i < out_n; ++i) {
        Taps& t = taps[static_cast<std::size_t>(i)];
        if (s > 1.0) {
            const double lo = i * s, hi = (i + 1) * s;
            const int a = static_cast<int>(std::floor(lo));
            const int b = std::min(in_n - 1, static_cast<int>(std::ceil(hi)) - 1);
            t.first = a;
            double sum = 0.0;
            for (int k = a; k <= b; ++k) {
                const double ov = std::min<double>(k + 1, hi) - std::max<double>(k, lo);
                t.w.push_back(static_cast<float>(std::max(ov, 0.0)));
                sum += static_cast<double>(t.w.back());
            }
            for (float& v : t.w) v = static_cast<float>(static_cast<double>(v) / sum);
        } else {
            const double c = std::clamp((i + 0.5) * s - 0.5, 0.0, static_cast<double>(in_n - 1));
            const int a = std::min(static_cast<int>(std::floor(c)), in_n - 1);
            const float f = static_cast<float>(c - a);
            t.first = a;
            if (a + 1 < in_n && f > 0.0f) t.w = {1.0f - f, f};
            else t.w = {1.0f};
        }
    }
    return taps;
}

struct Kernel {
    int dx, dy;
    int weight;
};

}  // namespace

void apply_levels(GreyImage& img, const Levels& lv) {
    std::uint8_t lut[256];
    const double black = lv.black;
    const double white = std::max<double>(lv.white, black + 1.0);
    const double inv_gamma = lv.gamma > 0.0 ? 1.0 / lv.gamma : 1.0;
    for (int g = 0; g < 256; ++g) {
        double v = std::clamp((g - black) / (white - black), 0.0, 1.0);
        v = std::pow(v, inv_gamma);
        if (lv.invert) v = 1.0 - v;
        lut[g] = clamp8(v * 255.0);
    }
    for (std::uint8_t& p : img.px) p = lut[p];
}

GreyImage resample(const GreyImage& src, int out_w, int out_h) {
    GreyImage out;
    if (src.w <= 0 || src.h <= 0 || out_w <= 0 || out_h <= 0) return out;
    const std::vector<Taps> tx = make_taps(src.w, out_w);
    const std::vector<Taps> ty = make_taps(src.h, out_h);
    // The horizontal pass keeps float precision so the vertical pass rounds once.
    std::vector<float> mid(static_cast<std::size_t>(out_w) * static_cast<std::size_t>(src.h));
    for (int y = 0; y < src.h; ++y) {
        const std::uint8_t* row = &src.px[idx(y, 0, src.w)];
        for (int x = 0; x < out_w; ++x) {
            const Taps& t = tx[static_cast<std::size_t>(x)];
            float acc = 0.0f;
            for (std::size_t k = 0; k < t.w.size(); ++k) acc += t.w[k] * row[t.first + static_cast<int>(k)];
            mid[idx(y, x, out_w)] = acc;
        }
    }
    out.w = out_w;
    out.h = out_h;
    out.px.resize(static_cast<std::size_t>(out_w) * static_cast<std::size_t>(out_h));
    for (int y = 0; y < out_h; ++y) {
        const Taps& t = ty[static_cast<std::size_t>(y)];
        for (int x = 0; x < out_w; ++x) {
            float acc = 0.0f;
            for (std::size_t k = 0; k < t.w.size(); ++k) acc += t.w[k] * mid[idx(t.first + static_cast<int>(k), x, out_w)];
            out.px[idx(y, x, out_w)] = clamp8(static_cast<double>(acc));
        }
    }
    return out;
}

bool raster_pixel_size(double width_mm, double height_mm, double dpi, int& nx, int& ny, std::string& error) {
    if (!(width_mm > 0.0) || !(height_mm > 0.0) || !(dpi > 0.0)) {
        error = "raster size and dpi must be positive";
        return false;
    }
    const double pitch = 25.4 / dpi;
    const double fx = std::max(1.0, std::round(width_mm / pitch));
    const double fy = std::max(1.0, std::round(height_mm / pitch));
    if (fx * fy > 64e6 || fx > 1e6 || fy > 1e6) {
        error = "raster would be more than 64 million pixels at this size and dpi";
        return false;
    }
    nx = static_cast<int>(fx);
    ny = static_cast<int>(fy);
    return true;
}

RasterBitmap dither(const GreyImage& img, DitherMethod method, const DitherOptions& opts) {
    RasterBitmap bm;
    bm.w = img.w;
    bm.h = img.h;
    bm.binary = method != DitherMethod::Grey;
    const std::size_t n = img.px.size();
    bm.level.assign(n, 0);

    if (method == DitherMethod::Grey) {
        for (std::size_t i = 0; i < n; ++i) bm.level[i] = static_cast<std::uint8_t>(255 - img.px[i]);
        return bm;
    }
    if (method == DitherMethod::Threshold) {
        for (std::size_t i = 0; i < n; ++i) bm.level[i] = img.px[i] < opts.threshold ? 255 : 0;
        return bm;
    }
    if (method == DitherMethod::Bayer) {
        // Classic 4x4 index matrix. A pixel fires when darker than
        // (index + 0.5)/16 of full scale, so grey g fires on about
        // (255-g)/255 of each 4x4 tile.
        static const int kBayer[4][4] = {{0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};
        for (int y = 0; y < img.h; ++y) {
            for (int x = 0; x < img.w; ++x) {
                const int t = kBayer[y & 3][x & 3] * 16 + 8;
                bm.level[idx(y, x, img.w)] = img.px[idx(y, x, img.w)] < t ? 255 : 0;
            }
        }
        return bm;
    }

    // Error diffusion. Kernels are listed for a left-to-right scan; a reversed
    // (serpentine) row mirrors dx. Serpentine breaks up the diagonal worms a
    // fixed direction leaves.
    static const Kernel kFloyd[] = {{1, 0, 7}, {-1, 1, 3}, {0, 1, 5}, {1, 1, 1}};
    static const Kernel kJarvis[] = {{1, 0, 7}, {2, 0, 5}, {-2, 1, 3}, {-1, 1, 5}, {0, 1, 7}, {1, 1, 5},
                                     {2, 1, 3}, {-2, 2, 1}, {-1, 2, 3}, {0, 2, 5}, {1, 2, 3}, {2, 2, 1}};
    static const Kernel kStucki[] = {{1, 0, 8}, {2, 0, 4}, {-2, 1, 2}, {-1, 1, 4}, {0, 1, 8}, {1, 1, 4},
                                     {2, 1, 2}, {-2, 2, 1}, {-1, 2, 2}, {0, 2, 4}, {1, 2, 2}, {2, 2, 1}};
    const Kernel* k = kFloyd;
    std::size_t kn = 4;
    float div = 16.0f;
    if (method == DitherMethod::Jarvis) { k = kJarvis; kn = 12; div = 48.0f; }
    if (method == DitherMethod::Stucki) { k = kStucki; kn = 12; div = 42.0f; }

    std::vector<float> buf(n);
    for (std::size_t i = 0; i < n; ++i) buf[i] = img.px[i];
    for (int y = 0; y < img.h; ++y) {
        const bool rev = opts.serpentine && (y & 1);
        for (int s = 0; s < img.w; ++s) {
            const int x = rev ? img.w - 1 - s : s;
            const std::size_t i = idx(y, x, img.w);
            const float old = buf[i];
            const bool fire = old < 128.0f;
            bm.level[i] = fire ? 255 : 0;
            const float err = old - (fire ? 0.0f : 255.0f);
            for (std::size_t j = 0; j < kn; ++j) {
                const int nx = x + (rev ? -k[j].dx : k[j].dx);
                const int ny = y + k[j].dy;
                if (nx < 0 || nx >= img.w || ny >= img.h) continue;
                buf[idx(ny, nx, img.w)] += err * static_cast<float>(k[j].weight) / div;
            }
        }
    }
    return bm;
}

double raster_overscan_mm(double speed_mm_s, double accel_mm_s2, double margin_mm) {
    const double ramp = accel_mm_s2 > 0.0 ? speed_mm_s * speed_mm_s / (2.0 * accel_mm_s2) : 0.0;
    return ramp + std::max(margin_mm, 0.0);
}

bool plan_raster(const RasterBitmap& bm, const LaserRaster& place, const LaserLayer& layer,
                 const RasterPlanOptions& opts, RasterPlan& out, std::string& error) {
    if (bm.w <= 0 || bm.h <= 0 ||
        bm.level.size() != static_cast<std::size_t>(bm.w) * static_cast<std::size_t>(bm.h)) {
        error = "empty raster bitmap";
        return false;
    }
    if (!(layer.speed > 0.0)) {
        error = "raster layer " + std::to_string(layer.pen) + " has no positive speed";
        return false;
    }
    RasterPlan p;
    p.at = place.at;
    p.width = place.width;
    p.height = place.height;
    p.w = bm.w;
    p.h = bm.h;
    p.pitch_x = place.width / bm.w;
    p.pitch_y = place.height / bm.h;
    p.pen = place.pen;
    p.speed_mm_s = layer.speed;
    p.min_power = layer.min_power;
    p.max_power = layer.max_power;
    p.binary = bm.binary;
    p.bidir = opts.bidir;
    p.overscan_mm = raster_overscan_mm(layer.speed, opts.accel_mm_s2, opts.margin_mm);

    bool ltr = true;
    for (int row = 0; row < bm.h; ++row) {
        const std::uint8_t* r = &bm.level[idx(row, 0, bm.w)];
        RasterLine line;
        for (int x = 0; x < bm.w;) {
            if (r[x] == 0) {
                ++x;
                continue;
            }
            RasterSpan s;
            s.x0 = x;
            while (x < bm.w && r[x] != 0) ++x;
            s.x1 = x;
            if (!bm.binary) s.levels.assign(r + s.x0, r + s.x1);
            line.spans.push_back(std::move(s));
        }
        if (line.spans.empty()) {
            ++p.blank_rows;
            continue;
        }
        // The pass covers only first..last fired pixel: blank margins are not
        // swept, which is most of the time saved on a small logo on a big plate.
        const int first = line.spans.front().x0;
        const int last = line.spans.back().x1;
        for (const RasterSpan& s : line.spans) p.fired_pixels += static_cast<std::size_t>(s.x1 - s.x0);
        const double lo = p.at.x + first * p.pitch_x - p.overscan_mm;
        const double hi = p.at.x + last * p.pitch_x + p.overscan_mm;
        line.row = row;
        line.y = p.at.y + p.height - (row + 0.5) * p.pitch_y;
        line.left_to_right = ltr;
        line.x_start = ltr ? lo : hi;
        line.x_end = ltr ? hi : lo;
        if (opts.bidir) ltr = !ltr;
        p.estimated_s += segment_time_s(hi - lo, layer.speed, opts.accel_mm_s2);
        if (!opts.bidir) p.estimated_s += segment_time_s(hi - lo, opts.travel_mm_s, opts.accel_mm_s2);
        p.lines.push_back(std::move(line));
    }
    out = std::move(p);
    return true;
}

bool prepare_raster(const LaserRaster& place, const LaserLayer& layer, const RasterPrepOptions& opts,
                    RasterPlan& out, RasterBitmap* bitmap, std::string& error) {
    GreyImage img;
    if (!load_grey_image(place.image, img, error)) return false;
    int nx = 0, ny = 0;
    if (!raster_pixel_size(place.width, place.height, place.dpi, nx, ny, error)) return false;
    GreyImage r = resample(img, nx, ny);
    apply_levels(r, opts.levels);
    const RasterBitmap bm = dither(r, place.dither, opts.dither);
    RasterPlanOptions po = opts.plan;
    po.bidir = place.bidir;
    if (!plan_raster(bm, place, layer, po, out, error)) return false;
    if (bitmap) *bitmap = bm;
    return true;
}

bool encode_raster(const RasterPlan& plan, std::vector<std::uint8_t>& bytes, std::string& error) {
    (void)plan;
    bytes.clear();
    error = "raster encoding not implemented yet";
    return false;
}

}  // namespace ncad::laser
