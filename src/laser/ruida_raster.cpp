// SPDX-License-Identifier: BSD-3-Clause AND MIT
// Copyright (c) 2026, Sadie Forbes
// Copyright (c) 2021 meerk40t
//
// Command bytes from meerk40t/ruida/rdjob.py; the stream is LightBurn's, read
// from its .rd files. See third_party/meerk40t/NOTICE.

#include "ncad/laser/ruida_raster.hpp"

#include "ncad/laser/ruida_codec.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <map>

namespace ncad::laser::ruida {

namespace {

struct Um {
    std::int64_t x{0};
    std::int64_t y{0};
};

std::int64_t grid10(std::int64_t um) {
    const std::int64_t v = um + 5;
    const std::int64_t q = v / 10 - ((v % 10 != 0 && v < 0) ? 1 : 0);
    return q * 10;
}

const unsigned char kElementName[10] = {0x05, 0x2A, 0x39, 0x1C, 0x41, 0x04, 0x6A, 0x15, 0x08, 0x20};

bool fail(std::string& error, const std::string& what) {
    error = what;
    return false;
}

bool to_um(double mm, std::int64_t& um) {
    if (!std::isfinite(mm)) return false;
    const double v = std::round(mm * 1000.0);
    if (v < -2147483648.0 || v > 2147483647.0) return false;
    um = static_cast<std::int64_t>(v);
    return true;
}

// One motion of the scan: where it ends, whether it fires, and the raw power
// to set first (-1: none).
struct Step {
    Um to;
    bool cut{false};
    int raw{-1};
};

}  // namespace

std::uint16_t raster_level_to_raw(std::uint8_t g) {
    const float f = static_cast<float>(g) * (1.0f / 254.0f);
    const float p = f * 16383.0f;
    const int raw = static_cast<int>(std::floor(p + 0.5f));
    return static_cast<std::uint16_t>(std::min(raw, 16383));
}

bool encode_rd_raster(const RdRasterJob& job, std::string& plain, std::string& error) {
    const RdRaster& R = job.raster;
    const RdLayer& L = job.layer;
    if (R.cols == 0 || R.rows == 0) return fail(error, "the raster is empty");
    if (R.x_lines.size() != R.cols + 1 || R.y_lines.size() != R.rows || R.level.size() != R.cols * R.rows) {
        return fail(error, "the raster's grid and pixels do not agree");
    }
    if (!R.row_x_lines.empty()) {
        if (R.row_x_lines.size() != R.rows) return fail(error, "the raster's per-line grids do not agree");
        for (const auto& v : R.row_x_lines) {
            if (v.size() != R.cols + 1) return fail(error, "the raster's per-line grids do not agree");
        }
    }
    if (L.passes != 1) return fail(error, "raster layers support one pass only (no sample)");
    if (job.ref_point < 0 || job.ref_point > 2) return fail(error, "ref_point must be 0, 1 or 2");
    const double min2 = L.min_power2 < 0.0 ? L.min_power : L.min_power2;
    const double max2 = L.max_power2 < 0.0 ? L.max_power : L.max_power2;
    for (double p : {L.min_power, L.max_power, min2, max2}) {
        if (!(p >= 0.0 && p <= 100.0)) return fail(error, "power outside 0..100%");
        if (p > job.power_limit) {
            char buf[160];
            std::snprintf(buf, sizeof buf, "power above the hard limit of %g%% (power_limit in lasers.conf)",
                          job.power_limit);
            return fail(error, buf);
        }
    }
    if (L.min_power > L.max_power || min2 > max2) return fail(error, "min power above max power");
    if (!(L.speed_mm_s > 0.0)) return fail(error, "speed must be positive");
    // Greyscale sets each run's power itself, 0..100% of the tube, ignoring
    // the layer's max (as LightBurn's own file does): the hard limit has to
    // apply to those values too.
    if (R.grayscale) {
        const double top = 100.0 * raster_level_to_raw(255) / 16383.0;
        bool any = false;
        std::uint8_t mx = 0;
        for (std::uint8_t g : R.level) mx = std::max(mx, g);
        any = !R.level.empty();
        if (any && 100.0 * raster_level_to_raw(mx) / 16383.0 > job.power_limit) {
            char buf[200];
            std::snprintf(buf, sizeof buf,
                          "greyscale pixel power reaches %.1f%%, above the hard limit of %g%% (greyscale writes "
                          "absolute powers; lower the image's white level)",
                          100.0 * raster_level_to_raw(mx) / 16383.0, job.power_limit);
            return fail(error, buf);
        }
        (void)top;
    }

    // --- the steps ------------------------------------------------------------

    std::vector<Step> steps;
    Um first;
    bool have_first = false;
    for (std::size_t k = 0; k < R.rows; ++k) {
        const bool forward = R.first_forward != ((k & 1u) != 0);
        const std::uint8_t* px = &R.level[k * R.cols];
        const std::int64_t y = R.y_lines[k];
        std::size_t lo = 0, hi = R.cols;  // [lo, hi) pixels written
        if (!R.grayscale) {
            lo = R.cols;
            hi = 0;
            for (std::size_t i = 0; i < R.cols; ++i) {
                if (px[i]) {
                    lo = std::min(lo, i);
                    hi = i + 1;
                }
            }
            if (lo >= hi) continue;  // a blank line is skipped (no sample)
        }
        // Runs, in the line's direction.
        struct Run {
            std::size_t a, b;  // pixels [a, b)
            bool cut;
            int raw;
        };
        std::vector<Run> runs;
        for (std::size_t i = lo; i < hi;) {
            std::size_t j = i + 1;
            if (R.grayscale) {
                while (j < hi && px[j] == px[i]) ++j;
                runs.push_back({i, j, true, raster_level_to_raw(px[i])});
            } else {
                const bool on = px[i] != 0;
                while (j < hi && (px[j] != 0) == on) ++j;
                runs.push_back({i, j, on, -1});
            }
            i = j;
        }
        const std::vector<std::int64_t>& XL = R.row_x_lines.empty() ? R.x_lines : R.row_x_lines[k];
        const std::int64_t start_x = forward ? XL[lo] : XL[hi];
        if (!have_first) {
            first = {start_x, y};
            have_first = true;
        } else {
            steps.push_back({{start_x, y}, false, -1});
        }
        if (forward) {
            for (const Run& r : runs) steps.push_back({{XL[r.b], y}, r.cut, r.raw});
        } else {
            for (std::size_t n = runs.size(); n-- > 0;) {
                steps.push_back({{XL[runs[n].a], y}, runs[n].cut, runs[n].raw});
            }
        }
    }
    if (!have_first) return fail(error, "the raster fires no pixel");

    // --- boxes ----------------------------------------------------------------

    Um K;
    if (!to_um(job.pointer_offset.x, K.x) || !to_um(job.pointer_offset.y, K.y)) {
        return fail(error, "the pointer offset is out of range");
    }
    std::int64_t min_x = first.x, max_x = first.x, min_y = first.y, max_y = first.y;
    for (const Step& s : steps) {
        min_x = std::min(min_x, s.to.x);
        max_x = std::max(max_x, s.to.x);
        min_y = std::min(min_y, s.to.y);
        max_y = std::max(max_y, s.to.y);
    }
    const std::int64_t bx0 = grid10(min_x) - K.x, by0 = grid10(min_y) - K.y;
    const std::int64_t bx1 = grid10(max_x) - K.x, by1 = grid10(max_y) - K.y;

    std::string out;
    bool ok = true;
    auto cmd = [&](std::initializer_list<unsigned> key) {
        for (unsigned b : key) out.push_back(static_cast<char>(b));
    };
    auto byte = [&](unsigned b) { out.push_back(static_cast<char>(b & 0x7Fu)); };
    auto abs_c = [&](std::int64_t um) { ok = put_abs_coord(out, um) && ok; };
    auto point = [&](std::int64_t x, std::int64_t y) {
        abs_c(x);
        abs_c(y);
    };
    auto power = [&](double pct) { ok = put_power(out, pct) && ok; };
    auto speed = [&](double v) { ok = put_speed(out, v) && ok; };

    // --- header (the vector layout's, with Work Mode Part 2) -------------------

    if (job.ref_point == 2) {
        cmd({0xD8, 0x10});
        cmd({0xE6, 0x01});
    } else {
        cmd({0xD8, job.ref_point == 1 ? 0x11u : 0x12u});
    }
    cmd({0xF0});
    cmd({0xF1, 0x02}); byte(0);
    cmd({0xD8, 0x00});
    cmd({0xE7, 0x06}); put_u35(out, 0); put_u35(out, 0);
    cmd({0xE7, 0x38}); byte(0);
    cmd({0xE7, 0x03}); point(bx0, by0);
    cmd({0xE7, 0x07}); point(bx1, by1);
    cmd({0xE7, 0x50}); point(grid10(min_x), grid10(min_y));
    cmd({0xE7, 0x51}); point(bx1, by1);
    cmd({0xE7, 0x04}); put_u14(out, 1); put_u14(out, 1); point(0, 0);
    cmd({0xE7, 0x05}); byte(0);

    cmd({0xC9, 0x04}); byte(0); speed(L.speed_mm_s);
    cmd({0xC6, 0x31}); byte(0); power(L.min_power);
    cmd({0xC6, 0x32}); byte(0); power(L.max_power);
    cmd({0xC6, 0x41}); byte(0); power(min2);
    cmd({0xC6, 0x42}); byte(0); power(max2);
    const std::uint32_t colour = ((L.rgb >> 16) & 0xFFu) | (L.rgb & 0xFF00u) | ((L.rgb & 0xFFu) << 16);
    cmd({0xCA, 0x06}); byte(0); put_u35(out, colour);
    cmd({0xCA, 0x41}); byte(0); byte(2);  // Work Mode Part 2: raster
    cmd({0xE7, 0x52}); byte(0); point(bx0, by0);
    cmd({0xE7, 0x53}); byte(0); point(bx1, by1);
    cmd({0xE7, 0x61}); byte(0); point(bx0, by0);
    cmd({0xE7, 0x62}); byte(0); point(bx1, by1);
    cmd({0xCA, 0x22}); byte(0);
    cmd({0xE7, 0x54}); byte(0); abs_c(0);
    cmd({0xE7, 0x54}); byte(1); abs_c(0);
    cmd({0xE7, 0x55}); byte(0); abs_c(0);
    cmd({0xE7, 0x55}); byte(1); abs_c(0);
    cmd({0xF1, 0x03}); point(0, 0);
    cmd({0xF1, 0x00}); byte(0);
    cmd({0xF1, 0x01}); byte(0);
    cmd({0xF2, 0x00}); byte(0);
    cmd({0xF2, 0x01}); byte(0);
    cmd({0xF2, 0x02});
    for (unsigned char ch : kElementName) byte(ch);
    cmd({0xF2, 0x03}); point(bx0, by0);
    cmd({0xF2, 0x04}); point(bx1, by1);
    cmd({0xF2, 0x05}); put_u14(out, 1); put_u14(out, 1); point(bx1 - bx0, by1 - by0);
    cmd({0xF2, 0x06}); point(0, 0);
    cmd({0xF2, 0x07}); byte(0);
    cmd({0xEA}); byte(0);
    cmd({0xE7, 0x60}); byte(0);
    cmd({0xE7, 0x13}); point(bx0, by0);
    cmd({0xE7, 0x17}); point(bx1, by1);
    cmd({0xE7, 0x23}); point(bx0, by0);
    cmd({0xE7, 0x24}); byte(0);
    cmd({0xE7, 0x37}); point(bx1, by1);
    cmd({0xE7, 0x08}); put_u14(out, 1); put_u14(out, 1); point(bx1 - bx0, by1 - by0);

    // --- layer blocks -----------------------------------------------------------

    // The raster settings block: no Layer End, no Through Power, and the
    // delays AFTER the powers (the vector block has them the other way round).
    auto settings_block = [&](bool lead) {
        if (lead) cmd({0xCA, 0x01, 0x05});
        cmd({0xC9, 0x03}); speed(job.axis_speed_mm_s);
        cmd({0xCA, 0x01, 0x01});  // Work Mode 1
        cmd({0xCA, 0x02}); byte(0);
        cmd({0xCA, 0x01, 0x30});
        cmd({0xCA, 0x01, 0x10});
        cmd({0xCA, 0x01, L.air ? 0x13u : 0x12u});
        cmd({0xC9, 0x02}); speed(L.speed_mm_s);
        cmd({0xC6, 0x01}); power(L.min_power);
        cmd({0xC6, 0x02}); power(L.max_power);
        cmd({0xC6, 0x21}); power(min2);
        cmd({0xC6, 0x22}); power(max2);
        cmd({0xC6, 0x12}); put_u35(out, 0);
        cmd({0xC6, 0x13}); put_u35(out, 0);
        cmd({0xCA, 0x03}); byte(1);
    };
    auto pass_block = [&]() {
        cmd({0xCA, 0x01, 0x05});
        cmd({0xC9, 0x03}); speed(job.axis_speed_mm_s);
        cmd({0xCA, 0x01, 0x00});  // Layer End
        cmd({0xCA, 0x02}); byte(0);
        cmd({0xCA, 0x01, 0x30});
        cmd({0xCA, 0x01, 0x10});
        cmd({0xCA, 0x03}); byte(3);
    };

    settings_block(false);
    cmd({0x88}); point(first.x, first.y);  // Move Abs to the first line's start
    Um at = first;
    // Two (pass block, settings block) pairs follow in every sample.
    for (int n = 0; n < 2; ++n) {
        pass_block();
        settings_block(job.lead_wm6);
    }

    double cut_um = 0.0;
    for (const Step& s : steps) {
        if (s.raw >= 0) {
            const auto raw = static_cast<std::uint32_t>(s.raw);
            cmd({0xC7}); put_u14(out, raw);
            cmd({0xC2}); put_u14(out, raw);
        }
        const std::int64_t dx = s.to.x - at.x, dy = s.to.y - at.y;
        if (dx == 0 && dy == 0) continue;
        if (!fits_rel(dx) || !fits_rel(dy)) {
            cmd({s.cut ? 0xA8u : 0x88u});
            point(s.to.x, s.to.y);
        } else if (dy == 0) {
            cmd({s.cut ? 0xAAu : 0x8Au});
            ok = put_rel_coord(out, static_cast<std::int32_t>(dx)) && ok;
        } else if (dx == 0) {
            cmd({s.cut ? 0xABu : 0x8Bu});
            ok = put_rel_coord(out, static_cast<std::int32_t>(dy)) && ok;
        } else {
            cmd({s.cut ? 0xA9u : 0x89u});
            ok = put_rel_coord(out, static_cast<std::int32_t>(dx)) && ok;
            ok = put_rel_coord(out, static_cast<std::int32_t>(dy)) && ok;
        }
        if (s.cut) cut_um += static_cast<double>(std::llabs(dx) + std::llabs(dy));
        at = s.to;
    }

    cmd({0xE7, 0x00});  // Block End: raster files close the scan block, then the array
    cmd({0xEB});
    cmd({0xE7, 0x00});
    const auto cut_mm = static_cast<std::uint64_t>(std::floor(cut_um / 1000.0 + 1e-6));
    cmd({0xDA, 0x01, 0x06, 0x20}); put_u35(out, cut_mm); put_u35(out, cut_mm);
    const std::uint64_t sum = byte_sum(out) + 0xD7;
    cmd({0xE5, 0x05}); put_u35(out, sum);
    cmd({0xD7});

    if (!ok) return fail(error, "a value could not be encoded");
    plain = std::move(out);
    return true;
}

// --- reading a raster file back ---------------------------------------------------

namespace {

std::uint32_t arg14(const RdCommand& c, std::size_t at) {
    return get_u14(reinterpret_cast<const std::uint8_t*>(c.raw.data()) + at);
}

struct Seg {
    std::int64_t x0, x1;
    bool cut;
    int raw;
};
struct RowRead {
    std::int64_t y{0};
    std::vector<Seg> segs;
};

}  // namespace

bool raster_job_from_rd(const std::vector<RdCommand>& cmds, const RdSummary& summary, RdRasterJob& out,
                        std::string& error) {
    RdRasterJob job;
    job.pointer_offset = {static_cast<double>(summary.origin_x) / 1000.0, static_cast<double>(summary.origin_y) / 1000.0};
    int wm6 = 0;
    bool seen_speed = false, seen_min1 = false, seen_max1 = false, seen_min2 = false, seen_max2 = false;
    bool gray = false;
    bool have_pos = false;
    std::int64_t x = 0, y = 0;
    int power_raw = -1;
    std::vector<RowRead> rows;
    for (const RdCommand& c : cmds) {
        if (!c.spec || !c.length_ok) return fail(error, "the stream has a command that does not decode");
        const auto* b = reinterpret_cast<const std::uint8_t*>(c.raw.data());
        const std::size_t n = c.raw.size();
        if (b[0] == 0xD8 && n == 2) {
            if (b[1] == 0x10) job.ref_point = 2;
            else if (b[1] == 0x11) job.ref_point = 1;
            else if (b[1] == 0x12) job.ref_point = 0;
        } else if (b[0] == 0xCA && n == 3 && b[1] == 0x01) {
            if (b[2] == 0x05) ++wm6;
            if (b[2] == 0x13) job.layer.air = true;
            if (b[2] == 0x12) job.layer.air = false;
        } else if (b[0] == 0xCA && b[1] == 0x06 && n >= 7) {
            const auto v = static_cast<std::uint32_t>(c.fields.back().i);
            job.layer.rgb = ((v >> 16) & 0xFFu) | (v & 0xFF00u) | ((v & 0xFFu) << 16);
        } else if (b[0] == 0xC9 && b[1] == 0x03 && !c.fields.empty()) {
            job.axis_speed_mm_s = c.fields[0].d;
        } else if (b[0] == 0xC9 && b[1] == 0x02 && !seen_speed) {
            job.layer.speed_mm_s = c.fields[0].d;
            seen_speed = true;
        } else if (b[0] == 0xC6 && n == 4 && b[1] == 0x01 && !seen_min1) {
            job.layer.min_power = c.fields[0].d;
            seen_min1 = true;
        } else if (b[0] == 0xC6 && n == 4 && b[1] == 0x02 && !seen_max1) {
            job.layer.max_power = c.fields[0].d;
            seen_max1 = true;
        } else if (b[0] == 0xC6 && n == 4 && b[1] == 0x21 && !seen_min2) {
            job.layer.min_power2 = c.fields[0].d;
            seen_min2 = true;
        } else if (b[0] == 0xC6 && n == 4 && b[1] == 0x22 && !seen_max2) {
            job.layer.max_power2 = c.fields[0].d;
            seen_max2 = true;
        } else if (b[0] == 0xC7 && n == 3) {
            power_raw = static_cast<int>(arg14(c, 1));
            gray = true;
        } else if (b[0] == 0xC2 && n == 3) {
            // the second tube's copy of the C7 before it
        } else {
            char axis = 'b';
            const RdMotion m = motion_of(c, &axis);
            if (m == RdMotion::None) continue;
            std::int64_t nx = x, ny = y;
            if (m == RdMotion::MoveAbs || m == RdMotion::CutAbs) {
                nx = c.fields[0].i;
                ny = c.fields[1].i;
            } else if (c.fields.size() == 2) {
                nx += c.fields[0].i;
                ny += c.fields[1].i;
            } else if (axis == 'x') {
                nx += c.fields[0].i;
            } else {
                ny += c.fields[0].i;
            }
            const bool cut = m == RdMotion::CutAbs || m == RdMotion::CutRel;
            if (!have_pos) {
                if (m != RdMotion::MoveAbs) return fail(error, "the first motion is not an absolute move");
                have_pos = true;
                rows.push_back({ny, {}});
            } else if (ny != y) {
                if (cut) return fail(error, "a cut changes the line");
                rows.push_back({ny, {}});
                if (nx != x) rows.back().segs.push_back({nx, nx, false, -1});  // the start-point shift
            } else {
                rows.back().segs.push_back({x, nx, cut, cut ? power_raw : -1});
            }
            x = nx;
            y = ny;
        }
    }
    if (rows.empty() || !have_pos) return fail(error, "no raster lines");
    job.lead_wm6 = wm6 > 2;
    job.layer.passes = 1;
    RdRaster& R = job.raster;
    R.grayscale = gray;

    // Drop the zero-length start markers (kept only to carry the start x).
    std::vector<std::int64_t> starts(rows.size());
    {
        std::int64_t cx = 0;
        bool first = true;
        (void)cx;
        (void)first;
    }
    // Row start x: the first segment's x0 (or the marker).
    for (std::size_t k = 0; k < rows.size(); ++k) {
        RowRead& r = rows[k];
        if (r.segs.empty()) return fail(error, "a line carries no pixels");
        if (r.segs.front().x0 == r.segs.front().x1) {
            starts[k] = r.segs.front().x0;
            r.segs.erase(r.segs.begin());
            if (r.segs.empty()) return fail(error, "a line carries no pixels");
        } else {
            starts[k] = r.segs.front().x0;
        }
    }

    // The grid: every position seen, with the unseen lines inside a long run
    // interpolated at the pixel pitch.
    std::vector<std::int64_t> pos;
    for (const RowRead& r : rows) {
        for (const Seg& sg : r.segs) {
            pos.push_back(sg.x0);
            pos.push_back(sg.x1);
        }
    }
    std::sort(pos.begin(), pos.end());
    pos.erase(std::unique(pos.begin(), pos.end()), pos.end());
    if (pos.size() < 2) return fail(error, "the raster has no extent");
    // LightBurn's greyscale lines are not the same in every row: a line can
    // sit 1 um off between rows. Positions within 3 um are one grid line; the
    // per-row values are kept in row_x_lines when they differ.
    std::vector<std::int64_t> reps;
    std::map<std::int64_t, std::size_t> cluster;
    for (std::int64_t p : pos) {
        if (reps.empty() || p - reps.back() > 3) reps.push_back(p);
        cluster[p] = reps.size() - 1;
    }
    std::int64_t min_d = std::numeric_limits<std::int64_t>::max();
    for (std::size_t i = 1; i < reps.size(); ++i) min_d = std::min(min_d, reps[i] - reps[i - 1]);
    const double pitch = (min_d >= 160 && min_d <= 170) ? 25400.0 / 154.0 : static_cast<double>(min_d);
    std::vector<std::int64_t> lines{reps[0]};
    std::vector<std::size_t> line_of_rep{0};  // cluster -> line index
    for (std::size_t i = 1; i < reps.size(); ++i) {
        const std::int64_t span = reps[i] - reps[i - 1];
        const auto cnt = static_cast<std::int64_t>(std::llround(static_cast<double>(span) / pitch));
        if (cnt < 1) return fail(error, "two grid lines closer than a pixel");
        for (std::int64_t q = 1; q < cnt; ++q) lines.push_back(reps[i - 1] + span * q / cnt);
        lines.push_back(reps[i]);
        line_of_rep.push_back(lines.size() - 1);
    }
    auto line_index = [&](std::int64_t p) { return line_of_rep[cluster[p]]; };
    std::vector<std::vector<std::int64_t>> per_row(rows.size(), lines);
    bool jitter = false;
    for (std::size_t k = 0; k < rows.size(); ++k) {
        for (const Seg& sg : rows[k].segs) {
            for (std::int64_t p : {sg.x0, sg.x1}) {
                const std::size_t li = line_index(p);
                if (per_row[k][li] != p) {
                    per_row[k][li] = p;
                    jitter = true;
                }
            }
        }
    }
    R.x_lines = lines;
    if (jitter) R.row_x_lines = per_row;
    R.cols = lines.size() - 1;
    R.rows = rows.size();
    R.level.assign(R.cols * R.rows, 0);
    for (std::size_t k = 0; k < rows.size(); ++k) R.y_lines.push_back(rows[k].y);

    R.first_forward = rows[0].segs.front().x1 > rows[0].segs.front().x0;
    for (std::size_t k = 0; k < rows.size(); ++k) {
        const bool fwd = rows[k].segs.front().x1 > rows[k].segs.front().x0;
        if (fwd != (R.first_forward != ((k & 1u) != 0))) return fail(error, "the lines do not alternate direction");
        for (const Seg& s : rows[k].segs) {
            if (!s.cut) continue;
            const std::size_t a = line_index(std::min(s.x0, s.x1)), bb = line_index(std::max(s.x0, s.x1));
            std::uint8_t g = 255;
            if (gray) {
                int found = -1;
                for (int v = 0; v < 256; ++v) {
                    if (raster_level_to_raw(static_cast<std::uint8_t>(v)) == s.raw) {
                        found = v;
                        break;
                    }
                }
                if (found < 0) return fail(error, "a pixel power is not one LightBurn's greyscale formula writes");
                g = static_cast<std::uint8_t>(found);
            }
            for (std::size_t i = a; i < bb; ++i) R.level[k * R.cols + i] = g;
        }
    }
    out = std::move(job);
    return true;
}

}  // namespace ncad::laser::ruida

namespace ncad::laser {

bool make_rd_raster(const GreyImage& src, const RdRasterGeometry& geo, const RdRasterSettings& set,
                    ruida::RdRaster& out, std::string& error) {
    if (src.w < 1 || src.h < 1 || src.px.size() != static_cast<std::size_t>(src.w) * static_cast<std::size_t>(src.h)) {
        error = "empty image";
        return false;
    }
    if (!(geo.width_mm > 0.0) || !(geo.height_mm > 0.0) || !(geo.pitch_um > 0.0)) {
        error = "raster size and pitch must be positive";
        return false;
    }
    const auto cols = static_cast<std::size_t>(std::floor(geo.width_mm * 1000.0 / geo.pitch_um + 1e-9));
    const auto rows = static_cast<std::size_t>(std::floor(geo.height_mm * 1000.0 / geo.pitch_um + 1e-9));
    if (cols < 1 || rows < 1 || cols * rows > 64000000u) {
        error = "the raster would have no pixels, or more than 64 million";
        return false;
    }
    const double wum = geo.width_mm * 1000.0, hum = geo.height_mm * 1000.0;

    // Nearest pixel at each cell's centre, in file orientation.
    GreyImage g;
    g.w = static_cast<int>(cols);
    g.h = static_cast<int>(rows);
    g.px.resize(cols * rows);
    for (std::size_t k = 0; k < rows; ++k) {
        const double v = (static_cast<double>(k) + 0.5) * geo.pitch_um;  // from the file-high-y edge
        const double from_top = geo.mirror_y ? hum - v : v;               // mirror_y: that edge is the design's bottom
        int sr = static_cast<int>(std::floor(from_top / hum * src.h));
        sr = std::min(std::max(sr, 0), src.h - 1);
        for (std::size_t i = 0; i < cols; ++i) {
            const double u = (static_cast<double>(i) + 0.5) * geo.pitch_um;  // from the file-low-x edge
            const double from_left = geo.mirror_x ? wum - u : u;
            int sc = static_cast<int>(std::floor(from_left / wum * src.w));
            sc = std::min(std::max(sc, 0), src.w - 1);
            g.px[k * cols + i] = src.px[static_cast<std::size_t>(sr) * static_cast<std::size_t>(src.w) +
                                        static_cast<std::size_t>(sc)];
        }
    }
    // Burn strength: dark burns, or light with a negative image.
    for (std::uint8_t& p : g.px) p = set.negative ? p : static_cast<std::uint8_t>(255 - p);

    ruida::RdRaster R;
    R.cols = cols;
    R.rows = rows;
    R.grayscale = set.mode == RdRasterMode::Grayscale;
    R.first_forward = geo.first_forward;
    R.level.resize(cols * rows);
    if (set.mode == RdRasterMode::Grayscale) {
        R.level = g.px;
    } else if (set.mode == RdRasterMode::Threshold) {
        for (std::size_t i = 0; i < R.level.size(); ++i) R.level[i] = g.px[i] >= set.threshold ? 255 : 0;
    } else {
        // dither() fires dark pixels, so hand it the inverse of the burn strength.
        GreyImage inv = g;
        for (std::uint8_t& p : inv.px) p = static_cast<std::uint8_t>(255 - p);
        const RasterBitmap bm = dither(inv, set.dither);
        for (std::size_t i = 0; i < R.level.size(); ++i) R.level[i] = bm.level[i] ? 255 : 0;
    }
    for (std::size_t i = 0; i <= cols; ++i) {
        R.x_lines.push_back(static_cast<std::int64_t>(
            std::floor(geo.file_x_lo_um + geo.x_phase_um + static_cast<double>(i) * geo.pitch_um)));
    }
    for (std::size_t k = 0; k < rows; ++k) {
        R.y_lines.push_back(static_cast<std::int64_t>(
            std::floor(geo.file_y_hi_um + geo.y_phase_um - static_cast<double>(k) * geo.pitch_um)));
    }
    out = std::move(R);
    return true;
}

}  // namespace ncad::laser
