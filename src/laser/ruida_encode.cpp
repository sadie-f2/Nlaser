// SPDX-License-Identifier: BSD-3-Clause AND MIT
// Copyright (c) 2026, Sadie Forbes
// Copyright (c) 2021 meerk40t
//
// Command bytes from meerk40t/ruida/rdjob.py; the sequence and the header
// values are LightBurn's, read from its .rd files. See
// third_party/meerk40t/NOTICE.

#include "ncad/laser/ruida_encode.hpp"

#include "ncad/laser/ruida_codec.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace ncad::laser::ruida {

namespace {

struct Um {
    std::int64_t x{0};
    std::int64_t y{0};
};

struct Box {
    bool any{false};
    std::int64_t min_x{0}, min_y{0}, max_x{0}, max_y{0};
    void add(const Um& p) {
        if (!any) {
            min_x = max_x = p.x;
            min_y = max_y = p.y;
            any = true;
            return;
        }
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
    }
};

// LightBurn's header bounds are on a 10 um grid while its moves are in um:
// 21.975 mm becomes 21.98, -231.521 becomes -231.52 (lightburn_box_joint.rd,
// lightburn_power_test_bengal.rd). Nearest, halves upward.
std::int64_t grid10(std::int64_t um) {
    const std::int64_t v = um + 5;
    const std::int64_t q = v / 10 - ((v % 10 != 0 && v < 0) ? 1 : 0);  // floor
    return q * 10;
}

// The constant LightBurn writes for the element name, in every file seen.
const unsigned char kElementName[10] = {0x05, 0x2A, 0x39, 0x1C, 0x41, 0x04, 0x6A, 0x15, 0x08, 0x20};

class Writer {
public:
    std::string out;
    bool ok{true};

    void cmd(std::initializer_list<unsigned> key) {
        for (unsigned b : key) out.push_back(static_cast<char>(b));
    }
    void byte(unsigned b) { out.push_back(static_cast<char>(b & 0x7Fu)); }
    void abs(std::int64_t um) { ok = put_abs_coord(out, um) && ok; }
    void rel(std::int64_t um) { ok = put_rel_coord(out, static_cast<std::int32_t>(um)) && ok; }
    void point(std::int64_t x, std::int64_t y) {
        abs(x);
        abs(y);
    }
    void power(double pct) { ok = put_power(out, pct) && ok; }
    void speed(double mm_s) { ok = put_speed(out, mm_s) && ok; }
    void u14(std::uint32_t v) { put_u14(out, v); }
    void u35(std::uint64_t v) { put_u35(out, v); }
};

bool fail(std::string& error, const std::string& what) {
    error = what;
    return false;
}

bool to_um(double mm, std::int64_t& um) {
    if (!std::isfinite(mm)) return false;
    const double v = std::round(mm * 1000.0);
    if (v < static_cast<double>(std::numeric_limits<std::int32_t>::min()) ||
        v > static_cast<double>(std::numeric_limits<std::int32_t>::max())) {
        return false;
    }
    um = static_cast<std::int64_t>(v);
    return true;
}

bool power_ok(double p) { return p >= 0.0 && p <= 100.0; }

}  // namespace

bool encode_rd(const RdJob& job, std::string& plain, std::string& error) {
    // --- validate, and fix the order of layers ------------------------------

    std::vector<std::size_t> part_of(job.layers.size(), SIZE_MAX);
    std::vector<std::size_t> layer_of_part;
    for (std::size_t i = 0; i < job.paths.size(); ++i) {
        const RdPath& p = job.paths[i];
        if (p.layer >= job.layers.size()) return fail(error, "path refers to a layer that does not exist");
        if (part_of[p.layer] == SIZE_MAX) {
            part_of[p.layer] = layer_of_part.size();
            layer_of_part.push_back(p.layer);
        } else if (job.paths[i - 1].layer != p.layer) {
            return fail(error, "a layer's paths are not contiguous; plan the job first");
        }
    }
    if (layer_of_part.empty()) return fail(error, "the job has nothing to cut");
    // Part numbers are single argument bytes, which must stay below 0x80.
    if (layer_of_part.size() > 128) return fail(error, "more than 128 layers");

    for (std::size_t li : layer_of_part) {
        const RdLayer& L = job.layers[li];
        char buf[96];
        if (!(L.speed_mm_s > 0.0)) {
            std::snprintf(buf, sizeof buf, "layer %zu: speed must be positive", li);
            return fail(error, buf);
        }
        const double min2 = L.min_power2 < 0.0 ? L.min_power : L.min_power2;
        const double max2 = L.max_power2 < 0.0 ? L.max_power : L.max_power2;
        if (!power_ok(L.min_power) || !power_ok(L.max_power) || !power_ok(min2) || !power_ok(max2)) {
            std::snprintf(buf, sizeof buf, "layer %zu: power outside 0..100%%", li);
            return fail(error, buf);
        }
        if (L.min_power > L.max_power || min2 > max2) {
            std::snprintf(buf, sizeof buf, "layer %zu: min power above max power", li);
            return fail(error, buf);
        }
        if (L.passes != 1) {
            std::snprintf(buf, sizeof buf,
                          "layer %zu: passes other than 1 are not supported yet (no LightBurn sample)", li);
            return fail(error, buf);
        }
    }

    // --- round once, and measure --------------------------------------------

    std::vector<std::vector<Um>> pts(job.paths.size());
    Box job_box;
    std::vector<Box> part_box(layer_of_part.size());
    for (std::size_t i = 0; i < job.paths.size(); ++i) {
        const RdPath& p = job.paths[i];
        if (p.pts.size() < 2) return fail(error, "a path has fewer than two points");
        for (const RdPoint& q : p.pts) {
            Um u;
            if (!to_um(q.x, u.x) || !to_um(q.y, u.y)) return fail(error, "a coordinate is out of range");
            pts[i].push_back(u);
            job_box.add(u);
            part_box[part_of[p.layer]].add(u);
        }
        if (p.closed) pts[i].push_back(pts[i].front());
    }
    Um origin;
    if (!to_um(job.origin.x, origin.x) || !to_um(job.origin.y, origin.y)) {
        return fail(error, "the origin is out of range");
    }

    // The header's box: grid-rounded, relative to the origin.
    const std::int64_t bx0 = grid10(job_box.min_x) - origin.x;
    const std::int64_t by0 = grid10(job_box.min_y) - origin.y;
    const std::int64_t bx1 = grid10(job_box.max_x) - origin.x;
    const std::int64_t by1 = grid10(job_box.max_y) - origin.y;

    Writer w;

    // --- header ---------------------------------------------------------------

    w.cmd({0xD8, 0x12});  // Ref Point 0: start from the current position
    w.cmd({0xF0});        // Ref Point Set
    w.cmd({0xF1, 0x02}); w.byte(0);  // Enable Block Cutting 0
    w.cmd({0xD8, 0x00});  // Start Process
    w.cmd({0xE7, 0x06}); w.u35(0); w.u35(0);  // Feed Repeat
    w.cmd({0xE7, 0x38}); w.byte(0);           // Set Feed Auto Pause
    w.cmd({0xE7, 0x03}); w.point(bx0, by0);   // Process Top Left
    w.cmd({0xE7, 0x07}); w.point(bx1, by1);   // Process Bottom Right
    w.cmd({0xE7, 0x50}); w.point(grid10(job_box.min_x), grid10(job_box.min_y));  // Document Min Point
    w.cmd({0xE7, 0x51}); w.point(bx1, by1);   // Document Max Point (relative, as LightBurn)
    w.cmd({0xE7, 0x04}); w.u14(1); w.u14(1); w.point(0, 0);  // Process Repeat
    w.cmd({0xE7, 0x05}); w.byte(0);           // Array Direction

    for (std::size_t part = 0; part < layer_of_part.size(); ++part) {
        const RdLayer& L = job.layers[layer_of_part[part]];
        const double min2 = L.min_power2 < 0.0 ? L.min_power : L.min_power2;
        const double max2 = L.max_power2 < 0.0 ? L.max_power : L.max_power2;
        const auto pb = static_cast<unsigned>(part);
        w.cmd({0xC9, 0x04}); w.byte(pb); w.speed(L.speed_mm_s);
        w.cmd({0xC6, 0x31}); w.byte(pb); w.power(L.min_power);
        w.cmd({0xC6, 0x32}); w.byte(pb); w.power(L.max_power);
        w.cmd({0xC6, 0x41}); w.byte(pb); w.power(min2);
        w.cmd({0xC6, 0x42}); w.byte(pb); w.power(max2);
        // Colour as 0xBBGGRR.
        const std::uint32_t c = ((L.rgb >> 16) & 0xFFu) | (L.rgb & 0xFF00u) | ((L.rgb & 0xFFu) << 16);
        w.cmd({0xCA, 0x06}); w.byte(pb); w.u35(c);
        w.cmd({0xCA, 0x41}); w.byte(pb); w.byte(0);  // Work Mode Part
        const Box& b = part_box[part];
        const std::int64_t px0 = grid10(b.min_x) - origin.x, py0 = grid10(b.min_y) - origin.y;
        const std::int64_t px1 = grid10(b.max_x) - origin.x, py1 = grid10(b.max_y) - origin.y;
        w.cmd({0xE7, 0x52}); w.byte(pb); w.point(px0, py0);
        w.cmd({0xE7, 0x53}); w.byte(pb); w.point(px1, py1);
        w.cmd({0xE7, 0x61}); w.byte(pb); w.point(px0, py0);
        w.cmd({0xE7, 0x62}); w.byte(pb); w.point(px1, py1);
    }
    w.cmd({0xCA, 0x22}); w.byte(static_cast<unsigned>(layer_of_part.size() - 1));  // Max Layer Part
    w.cmd({0xE7, 0x54}); w.byte(0); w.abs(0);  // Pen Offset
    w.cmd({0xE7, 0x54}); w.byte(1); w.abs(0);
    w.cmd({0xE7, 0x55}); w.byte(0); w.abs(0);  // Layer Offset
    w.cmd({0xE7, 0x55}); w.byte(1); w.abs(0);
    w.cmd({0xF1, 0x03}); w.point(0, 0);        // Display Offset

    w.cmd({0xF1, 0x00}); w.byte(0);  // Element Max Index
    w.cmd({0xF1, 0x01}); w.byte(0);  // Element Name Max Index
    w.cmd({0xF2, 0x00}); w.byte(0);  // Element Index
    w.cmd({0xF2, 0x01}); w.byte(0);  // Element Name Index
    w.cmd({0xF2, 0x02});
    for (unsigned char ch : kElementName) w.byte(ch);
    w.cmd({0xF2, 0x03}); w.point(bx0, by0);  // Element Array Min Point
    w.cmd({0xF2, 0x04}); w.point(bx1, by1);  // Element Array Max Point
    w.cmd({0xF2, 0x05}); w.u14(1); w.u14(1); w.point(bx1 - bx0, by1 - by0);  // Element Array
    w.cmd({0xF2, 0x06}); w.point(0, 0);      // Element Array Add
    w.cmd({0xF2, 0x07}); w.byte(0);          // Element Array Mirror

    w.cmd({0xEA}); w.byte(0);                // Array Start
    w.cmd({0xE7, 0x60}); w.byte(0);          // Set Current Element Index
    w.cmd({0xE7, 0x13}); w.point(bx0, by0);  // Array Min Point
    w.cmd({0xE7, 0x17}); w.point(bx1, by1);  // Array Max Point
    w.cmd({0xE7, 0x23}); w.point(bx0, by0);  // Array Add
    w.cmd({0xE7, 0x24}); w.byte(0);          // Array Mirror
    w.cmd({0xE7, 0x37}); w.point(bx1, by1);  // Array Even Distance
    w.cmd({0xE7, 0x08}); w.u14(1); w.u14(1); w.point(bx1 - bx0, by1 - by0);  // Array Repeat

    // --- layers -------------------------------------------------------------

    double cut_um = 0.0;
    Um at;
    bool first_motion = true;

    auto motion = [&](const Um& to, bool cut) {
        const std::int64_t dx = to.x - at.x;
        const std::int64_t dy = to.y - at.y;
        if (!first_motion && dx == 0 && dy == 0) return;
        if (first_motion || !fits_rel(dx) || !fits_rel(dy)) {
            w.cmd({cut ? 0xA8u : 0x88u});
            w.point(to.x, to.y);
        } else if (dy == 0) {
            w.cmd({cut ? 0xAAu : 0x8Au});
            w.rel(dx);
        } else if (dx == 0) {
            w.cmd({cut ? 0xABu : 0x8Bu});
            w.rel(dy);
        } else {
            w.cmd({cut ? 0xA9u : 0x89u});
            w.rel(dx);
            w.rel(dy);
        }
        if (cut) cut_um += std::hypot(static_cast<double>(dx), static_cast<double>(dy));
        at = to;
        first_motion = false;
    };

    std::size_t i = 0;
    for (std::size_t part = 0; part < layer_of_part.size(); ++part) {
        const RdLayer& L = job.layers[layer_of_part[part]];
        const double min2 = L.min_power2 < 0.0 ? L.min_power : L.min_power2;
        const double max2 = L.max_power2 < 0.0 ? L.max_power : L.max_power2;
        w.cmd({0xCA, 0x01, 0x00});  // Layer End (LightBurn opens every layer with it)
        w.cmd({0xCA, 0x02}); w.byte(static_cast<unsigned>(part));  // Layer Number Part
        w.cmd({0xCA, 0x01, 0x30});  // En Laser 2 Offset 0
        w.cmd({0xCA, 0x01, 0x10});  // Laser Device 0
        w.cmd({0xCA, 0x01, L.air ? 0x13u : 0x12u});
        w.cmd({0xC9, 0x02}); w.speed(L.speed_mm_s);
        w.cmd({0xC6, 0x12}); w.u35(0);  // Laser On Delay
        w.cmd({0xC6, 0x13}); w.u35(0);  // Laser Off Delay
        w.cmd({0xC6, 0x50}); w.u14(1);  // Through Power 1: raw 1, as LightBurn
        w.cmd({0xC6, 0x51}); w.u14(1);  // Through Power 2
        w.cmd({0xC6, 0x01}); w.power(L.min_power);
        w.cmd({0xC6, 0x02}); w.power(L.max_power);
        w.cmd({0xC6, 0x21}); w.power(min2);
        w.cmd({0xC6, 0x22}); w.power(max2);
        w.cmd({0xCA, 0x03}); w.byte(1);  // En Laser Tube Start

        // MeerK40t forces the first motion of a FILE to be absolute; LightBurn
        // makes every layer's first motion absolute in every sample, which is
        // also what keeps one layer's position from depending on another's.
        first_motion = true;
        for (; i < job.paths.size() && part_of[job.paths[i].layer] == part; ++i) {
            const std::vector<Um>& p = pts[i];
            motion(p.front(), false);
            for (std::size_t k = 1; k < p.size(); ++k) motion(p[k], true);
        }
    }

    // --- tail ---------------------------------------------------------------

    w.cmd({0xEB});        // Array End
    w.cmd({0xE7, 0x00});  // Block End
    const auto cut_mm = static_cast<std::uint64_t>(std::floor(cut_um / 1000.0 + 1e-6));
    w.cmd({0xDA, 0x01, 0x06, 0x20}); w.u35(cut_mm); w.u35(cut_mm);  // total cut, whole mm
    const std::uint64_t sum = byte_sum(w.out) + 0xD7;
    w.cmd({0xE5, 0x05}); w.u35(sum);  // Set File Sum
    w.cmd({0xD7});                    // End Of File

    if (!w.ok) return fail(error, "a value could not be encoded");
    plain = std::move(w.out);
    return true;
}

}  // namespace ncad::laser::ruida
