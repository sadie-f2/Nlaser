// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

#include "ncad/laser/job.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <sstream>

namespace ncad::laser {

bool parse_job_anchor(const std::string& text, JobAnchor& out) {
    std::string t;
    for (char c : text) t += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    static const struct {
        const char* name;
        JobAnchor a;
    } kTable[] = {{"none", JobAnchor::None}, {"tl", JobAnchor::TL},       {"t", JobAnchor::T},
                  {"tr", JobAnchor::TR},     {"l", JobAnchor::L},         {"c", JobAnchor::C},
                  {"centre", JobAnchor::C},  {"center", JobAnchor::C},    {"r", JobAnchor::R},
                  {"bl", JobAnchor::BL},     {"b", JobAnchor::B},         {"br", JobAnchor::BR}};
    for (const auto& e : kTable) {
        if (t == e.name) {
            out = e.a;
            return true;
        }
    }
    return false;
}

const char* job_anchor_name(JobAnchor a) {
    static const char* const kNames[] = {"none", "tl", "t", "tr", "l", "c", "r", "bl", "b", "br"};
    return kNames[static_cast<std::size_t>(a)];
}

const char* job_anchor_label(JobAnchor a) {
    static const char* const kLabels[] = {"none", "top left",    "top",    "top right",    "left",
                                          "centre", "right",     "bottom left", "bottom", "bottom right"};
    return kLabels[static_cast<std::size_t>(a)];
}

bool anchor_job(LaserJob& job, JobAnchor anchor) {
    if (anchor == JobAnchor::None) return false;
    bool any = false;
    Pt lo, hi;
    for (const LaserPath& p : job.paths) {
        const LaserLayer* l = job.layer(p.pen);
        if (l != nullptr && l->mode == LayerMode::Skip) continue;
        for (const Pt& q : p.pts) {
            if (!any) {
                lo = hi = q;
                any = true;
            }
            lo.x = std::min(lo.x, q.x);
            lo.y = std::min(lo.y, q.y);
            hi.x = std::max(hi.x, q.x);
            hi.y = std::max(hi.y, q.y);
        }
    }
    if (!any) return false;
    job.origin = job_anchor_point(anchor, lo, hi);
    return true;
}

Pt job_anchor_point(JobAnchor a, const Pt& lo, const Pt& hi) {
    const double mx = (lo.x + hi.x) / 2.0, my = (lo.y + hi.y) / 2.0;
    switch (a) {
        case JobAnchor::TL: return {lo.x, hi.y};
        case JobAnchor::T: return {mx, hi.y};
        case JobAnchor::TR: return {hi.x, hi.y};
        case JobAnchor::L: return {lo.x, my};
        case JobAnchor::C: return {mx, my};
        case JobAnchor::R: return {hi.x, my};
        case JobAnchor::BL: return {lo.x, lo.y};
        case JobAnchor::B: return {mx, lo.y};
        case JobAnchor::BR: return {hi.x, lo.y};
        case JobAnchor::None: break;
    }
    return lo;
}

const char* layer_mode_name(LayerMode m) {
    switch (m) {
        case LayerMode::Auto: return "auto";
        case LayerMode::Vector: return "vector";
        case LayerMode::Raster: return "raster";
        case LayerMode::Skip: return "skip";
    }
    return "auto";
}

const char* dither_name(DitherMethod m) {
    switch (m) {
        case DitherMethod::Threshold: return "threshold";
        case DitherMethod::Floyd: return "floyd";
        case DitherMethod::Jarvis: return "jarvis";
        case DitherMethod::Stucki: return "stucki";
        case DitherMethod::Bayer: return "bayer";
        case DitherMethod::Grey: return "grey";
    }
    return "floyd";
}

bool parse_dither(const std::string& name, DitherMethod& m) {
    for (DitherMethod d : {DitherMethod::Threshold, DitherMethod::Floyd, DitherMethod::Jarvis,
                           DitherMethod::Stucki, DitherMethod::Bayer, DitherMethod::Grey}) {
        if (name == dither_name(d)) {
            m = d;
            return true;
        }
    }
    return false;
}

const LaserLayer* LaserJob::layer(int pen) const {
    for (const LaserLayer& l : layers) {
        if (l.pen == pen) return &l;
    }
    return nullptr;
}

// --- writing -------------------------------------------------------------------

namespace {

// Shortest exact-enough decimal: settings print without trailing zeros,
// coordinates always with three places (micrometres).
std::string num(double v) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.6f", v);
    std::string s = buf;
    while (!s.empty() && s.back() == '0') s.pop_back();
    if (!s.empty() && s.back() == '.') s.pop_back();
    if (s == "-0") s = "0";
    return s;
}

std::string coord(double v) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.3f", v);
    std::string s = buf;
    if (s == "-0.000") s = "0.000";
    return s;
}

std::string quoted(const std::string& s) {
    std::string out = "\"";
    for (char c : s) {
        if (c == '"' || c == '\\') out.push_back('\\');
        out.push_back(c);
    }
    return out + "\"";
}

}  // namespace

std::string write_ljob(const LaserJob& job) {
    std::string out = "ljob " + std::to_string(job.version);
    if (!job.name.empty()) out += " name=" + quoted(job.name);
    if (!job.source.empty()) out += " source=" + quoted(job.source);
    if (job.planned) out += " planned";
    if (job.start == StartMode::Absolute) out += " start=absolute";  // current is the default and stays unwritten
    if (job.start == StartMode::User) out += " start=user";
    out += "\n";
    if (job.bed_w > 0.0 && job.bed_h > 0.0) out += "bed " + num(job.bed_w) + " " + num(job.bed_h) + "\n";
    out += "origin " + coord(job.origin.x) + " " + coord(job.origin.y) + "\n";
    for (const LaserLayer& l : job.layers) {
        out += "layer " + std::to_string(l.pen) + " color=" + std::to_string(l.color) +
               " mode=" + layer_mode_name(l.mode) + " speed=" + num(l.speed) + " power=" + num(l.min_power) +
               "," + num(l.max_power);
        if (l.has_power2) out += " power2=" + num(l.min_power2) + "," + num(l.max_power2);
        out += " passes=" + std::to_string(l.passes) + " air=" + (l.air ? "on" : "off") +
               " order=" + std::to_string(l.order) + "\n";
    }
    for (const LaserPath& p : job.paths) {
        out += "path " + std::to_string(p.pen) + (p.closed ? " closed" : " open");
        if (p.id >= 0) out += " id=" + std::to_string(p.id);
        out += "\n";
        for (const Pt& q : p.pts) out += "  " + coord(q.x) + " " + coord(q.y) + "\n";
        out += "end\n";
    }
    for (const LaserRaster& r : job.rasters) {
        out += "raster " + std::to_string(r.pen) + " at=" + coord(r.at.x) + "," + coord(r.at.y) +
               " size=" + coord(r.width) + "," + coord(r.height) + " dpi=" + num(r.dpi) +
               " image=" + quoted(r.image) + " dither=" + dither_name(r.dither) +
               " bidir=" + (r.bidir ? "on" : "off") + "\n";
    }
    return out;
}

// --- reading -------------------------------------------------------------------

namespace {

struct Tokens {
    std::vector<std::string> words;
    bool ok{true};
    std::string why;
};

// Splits on whitespace, keeping a quoted value (with \" and \\ escapes) as
// part of the word it belongs to.
Tokens tokenize(const std::string& line) {
    Tokens t;
    std::string cur;
    bool in_word = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if (c == '#' ) break;
        if (c == '"') {
            in_word = true;
            ++i;
            bool closed = false;
            for (; i < line.size(); ++i) {
                if (line[i] == '\\' && i + 1 < line.size()) {
                    cur.push_back(line[++i]);
                } else if (line[i] == '"') {
                    closed = true;
                    break;
                } else {
                    cur.push_back(line[i]);
                }
            }
            if (!closed) {
                t.ok = false;
                t.why = "unterminated quote";
                return t;
            }
            continue;
        }
        if (c == ' ' || c == '\t' || c == '\r') {
            if (in_word) t.words.push_back(cur);
            cur.clear();
            in_word = false;
            continue;
        }
        cur.push_back(c);
        in_word = true;
    }
    if (in_word) t.words.push_back(cur);
    return t;
}

bool parse_double(const std::string& s, double& v) {
    if (s.empty()) return false;
    char* end = nullptr;
    v = std::strtod(s.c_str(), &end);
    return end && *end == '\0' && std::isfinite(v);
}

bool parse_int(const std::string& s, int& v) {
    if (s.empty()) return false;
    char* end = nullptr;
    const long l = std::strtol(s.c_str(), &end, 10);
    if (!end || *end != '\0' || l < -1000000 || l > 1000000) return false;
    v = static_cast<int>(l);
    return true;
}

bool parse_pair(const std::string& s, double& a, double& b) {
    const std::size_t comma = s.find(',');
    if (comma == std::string::npos) return false;
    return parse_double(s.substr(0, comma), a) && parse_double(s.substr(comma + 1), b);
}

bool split_kv(const std::string& w, std::string& k, std::string& v) {
    const std::size_t eq = w.find('=');
    if (eq == std::string::npos || eq == 0) return false;
    k = w.substr(0, eq);
    v = w.substr(eq + 1);
    return true;
}

}  // namespace

bool read_ljob(const std::string& text, LaserJob& out, std::string& error) {
    LaserJob job;
    bool have_header = false;
    bool in_path = false;
    LaserPath path;
    std::size_t path_line = 0;

    std::istringstream in(text);
    std::string line;
    std::size_t n = 0;
    auto fail = [&](const std::string& why) {
        error = "line " + std::to_string(n) + ": " + why;
        return false;
    };

    while (std::getline(in, line)) {
        ++n;
        const Tokens t = tokenize(line);
        if (!t.ok) return fail(t.why);
        if (t.words.empty()) continue;
        const std::vector<std::string>& w = t.words;

        if (in_path) {
            if (w[0] == "end") {
                if (w.size() != 1) return fail("'end' takes nothing after it");
                if (path.pts.size() < 2) return fail("a path needs at least two points");
                job.paths.push_back(std::move(path));
                path = LaserPath{};
                in_path = false;
                continue;
            }
            Pt p;
            if (w.size() != 2 || !parse_double(w[0], p.x) || !parse_double(w[1], p.y)) {
                return fail("expected 'x y' or 'end' inside a path");
            }
            path.pts.push_back(p);
            continue;
        }

        if (!have_header) {
            if (w[0] != "ljob") return fail("a job starts with 'ljob <version>'");
            if (w.size() < 2 || !parse_int(w[1], job.version)) return fail("'ljob' needs a version number");
            if (job.version != 1) return fail("this reader knows version 1 only");
            for (std::size_t i = 2; i < w.size(); ++i) {
                std::string k, v;
                if (w[i] == "planned") {
                    job.planned = true;
                } else if (split_kv(w[i], k, v) && k == "name") {
                    job.name = v;
                } else if (split_kv(w[i], k, v) && k == "start") {
                    if (v == "current") job.start = StartMode::Current;
                    else if (v == "absolute") job.start = StartMode::Absolute;
                    else if (v == "user") job.start = StartMode::User;
                    else return fail("bad value for 'start' (current, absolute or user)");
                } else if (split_kv(w[i], k, v) && k == "source") {
                    job.source = v;
                } else {
                    return fail("unknown 'ljob' field '" + w[i] + "'");
                }
            }
            have_header = true;
            continue;
        }

        if (w[0] == "bed") {
            if (w.size() != 3 || !parse_double(w[1], job.bed_w) || !parse_double(w[2], job.bed_h) ||
                job.bed_w <= 0.0 || job.bed_h <= 0.0) {
                return fail("expected 'bed <width> <height>' in mm");
            }
        } else if (w[0] == "origin") {
            if (w.size() != 3 || !parse_double(w[1], job.origin.x) || !parse_double(w[2], job.origin.y)) {
                return fail("expected 'origin <x> <y>'");
            }
        } else if (w[0] == "layer") {
            LaserLayer l;
            if (w.size() < 2 || !parse_int(w[1], l.pen)) return fail("expected 'layer <pen> ...'");
            l.color = l.pen;
            if (job.layer(l.pen)) return fail("layer " + w[1] + " is defined twice");
            for (std::size_t i = 2; i < w.size(); ++i) {
                std::string k, v;
                if (!split_kv(w[i], k, v)) return fail("expected key=value, got '" + w[i] + "'");
                bool ok = true;
                if (k == "color") ok = parse_int(v, l.color);
                else if (k == "mode") {
                    if (v == "auto") l.mode = LayerMode::Auto;
                    else if (v == "vector") l.mode = LayerMode::Vector;
                    else if (v == "raster") l.mode = LayerMode::Raster;
                    else if (v == "skip") l.mode = LayerMode::Skip;
                    else ok = false;
                } else if (k == "speed") ok = parse_double(v, l.speed);
                else if (k == "power") ok = parse_pair(v, l.min_power, l.max_power);
                else if (k == "power2") ok = l.has_power2 = parse_pair(v, l.min_power2, l.max_power2);
                else if (k == "passes") ok = parse_int(v, l.passes) && l.passes >= 1;
                else if (k == "air") {
                    if (v == "on") l.air = true;
                    else if (v == "off") l.air = false;
                    else ok = false;
                } else if (k == "order") ok = parse_int(v, l.order);
                else return fail("unknown layer field '" + k + "'");
                if (!ok) return fail("bad value for '" + k + "': '" + v + "'");
            }
            job.layers.push_back(l);
        } else if (w[0] == "path") {
            if ((w.size() != 3 && w.size() != 4) || !parse_int(w[1], path.pen) ||
                (w[2] != "open" && w[2] != "closed")) {
                return fail("expected 'path <pen> open|closed [id=N]'");
            }
            path.closed = (w[2] == "closed");
            path.id = -1;
            if (w.size() == 4) {
                if (w[3].compare(0, 3, "id=") != 0 || !parse_int(w[3].substr(3), path.id) || path.id < 0) {
                    return fail("bad path id '" + w[3] + "'");
                }
            }
            in_path = true;
            path_line = n;
        } else if (w[0] == "raster") {
            LaserRaster r;
            if (w.size() < 2 || !parse_int(w[1], r.pen)) return fail("expected 'raster <pen> ...'");
            bool have_at = false, have_size = false, have_image = false;
            for (std::size_t i = 2; i < w.size(); ++i) {
                std::string k, v;
                if (!split_kv(w[i], k, v)) return fail("raster: expected key=value, got '" + w[i] + "'");
                bool ok = true;
                if (k == "at") ok = have_at = parse_pair(v, r.at.x, r.at.y);
                else if (k == "size") ok = have_size = parse_pair(v, r.width, r.height) && r.width > 0.0 && r.height > 0.0;
                else if (k == "dpi") ok = parse_double(v, r.dpi) && r.dpi > 0.0 && r.dpi <= 2540.0;
                else if (k == "image") ok = have_image = !v.empty(), r.image = v;
                else if (k == "dither") ok = parse_dither(v, r.dither);
                else if (k == "bidir") {
                    if (v == "on") r.bidir = true;
                    else if (v == "off") r.bidir = false;
                    else ok = false;
                } else return fail("raster: unknown field '" + k + "'");
                if (!ok) return fail("raster: bad value for '" + k + "': '" + v + "'");
            }
            if (!have_at || !have_size || !have_image) return fail("raster record needs at=, size= and image=");
            job.rasters.push_back(std::move(r));
        } else {
            return fail("unknown record '" + w[0] + "'");
        }
    }

    if (!have_header) {
        error = "empty job: no 'ljob' line";
        return false;
    }
    if (in_path) {
        error = "line " + std::to_string(path_line) + ": path has no 'end'";
        return false;
    }
    for (const LaserPath& p : job.paths) {
        if (!job.layer(p.pen)) {
            error = "a path uses pen " + std::to_string(p.pen) + ", which has no layer record";
            return false;
        }
    }
    for (const LaserRaster& r : job.rasters) {
        if (!job.layer(r.pen)) {
            error = "a raster uses pen " + std::to_string(r.pen) + ", which has no layer record";
            return false;
        }
    }
    out = std::move(job);
    return true;
}

bool read_ljob_file(const std::string& path, LaserJob& job, std::string& error) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        error = "cannot read '" + path + "'";
        return false;
    }
    const std::string text{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    if (!read_ljob(text, job, error)) {
        error = path + ": " + error;
        return false;
    }
    return true;
}

bool write_ljob_file(const std::string& path, const LaserJob& job, std::string& error) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) {
        error = "cannot write '" + path + "'";
        return false;
    }
    out << write_ljob(job);
    if (!out) {
        error = "error writing '" + path + "'";
        return false;
    }
    return true;
}

// --- colour ----------------------------------------------------------------------

namespace {

std::uint32_t rgb(int r, int g, int b) {
    return (static_cast<std::uint32_t>(r) << 16) | (static_cast<std::uint32_t>(g) << 8) |
           static_cast<std::uint32_t>(b);
}

}  // namespace

bool lightburn_layer_of_aci(int aci, int& layer, std::uint32_t& rgb) {
    struct Row { int aci; int layer; std::uint32_t rgb; };
    // tests/fixtures/lightburn/f12 (LightBurn 1.4.02, imported unchanged).
    static const Row kRows[] = {
        {1, 2, 0xFF0000},   {2, 4, 0xD0D000},   {3, 3, 0x00E000},   {4, 6, 0x00E0E0},   {5, 1, 0x0000FF},
        {6, 7, 0xFF00FF},   {7, 0, 0x000000},   {8, 27, 0x004754},  {9, 16, 0x808080},  {10, 2, 0xFF0000},
        {30, 5, 0x000000},  {50, 4, 0xD0D000},  {130, 6, 0x00E0E0}, {250, 25, 0x500A78},
    };
    for (const Row& r : kRows) {
        if (r.aci == aci) {
            layer = r.layer;
            rgb = r.rgb;
            return true;
        }
    }
    return false;
}

std::uint32_t aci_rgb(int index) {
    static const std::uint32_t kLow[10] = {
        0x000000,  // 0 BYBLOCK: should be resolved before this
        0xFF0000, 0xFFFF00, 0x00FF00, 0x00FFFF, 0x0000FF, 0xFF00FF,
        0x000000,  // 7: black on a light panel
        0x808080, 0xC0C0C0,
    };
    if (index < 0) index = -index;
    if (index < 10) return kLow[index];
    if (index >= 250 && index <= 255) {
        const int v = 33 + (index - 250) * 36;
        return rgb(v, v, v);
    }
    if (index > 255) return 0x000000;

    // 10..249: 24 hues by five brightness levels at full and half saturation,
    // as NotoCAD's display palette builds it.
    const int slot = index - 10;
    const double hue = static_cast<double>(slot / 10) * 15.0;  // 24 whole hues
    const int variant = slot % 10;
    static const double kValue[5] = {1.0, 0.8, 0.63, 0.45, 0.3};
    const double v = kValue[variant / 2];
    const double s = (variant % 2 == 0) ? 1.0 : 0.5;
    const double c = v * s;
    const double hp = hue / 60.0;
    const double x = c * (1.0 - std::fabs(std::fmod(hp, 2.0) - 1.0));
    double r = 0, g = 0, b = 0;
    if (hp < 1) { r = c; g = x; }
    else if (hp < 2) { r = x; g = c; }
    else if (hp < 3) { g = c; b = x; }
    else if (hp < 4) { g = x; b = c; }
    else if (hp < 5) { r = x; b = c; }
    else { r = c; b = x; }
    const double m = v - c;
    auto to8 = [](double f) { return static_cast<int>(std::lround(f * 255.0)); };
    return rgb(to8(r + m), to8(g + m), to8(b + m));
}

int rgb_to_aci(std::uint32_t rgb, bool* exact) {
    rgb &= 0xFFFFFFu;
    if (rgb == 0xFFFFFFu) {
        if (exact) *exact = true;
        return 7;
    }
    int best = 7;
    long best_d = -1;
    for (int i = 1; i <= 255; ++i) {
        const std::uint32_t c = aci_rgb(i);
        const long dr = static_cast<long>((c >> 16) & 0xFFu) - static_cast<long>((rgb >> 16) & 0xFFu);
        const long dg = static_cast<long>((c >> 8) & 0xFFu) - static_cast<long>((rgb >> 8) & 0xFFu);
        const long db = static_cast<long>(c & 0xFFu) - static_cast<long>(rgb & 0xFFu);
        const long d = dr * dr + dg * dg + db * db;
        if (best_d < 0 || d < best_d) {
            best_d = d;
            best = i;
            if (d == 0) break;
        }
    }
    if (exact) *exact = best_d == 0;
    return best;
}

}  // namespace ncad::laser
