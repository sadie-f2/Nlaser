// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

#include "ncad/laser/laser_renderer.hpp"

#include "ncad/blocks.hpp"
#include "ncad/database.hpp"
#include "ncad/dxf_read.hpp"
#include "ncad/entities.hpp"
#include "ncad/scene.hpp"
#include "ncad/tables.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>

namespace ncad::laser {

namespace {

// Parallel to Z, either way up: an entity drawn in the XY plane, or mirrored
// through it, cuts as drawn once Z is dropped. Anything else is a projection.
bool flat(const Vec3& n) {
    const double len = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
    if (!(len > 0.0)) return true;  // no direction to be wrong about
    return std::hypot(n.x, n.y) <= 1e-9 * len;
}

std::string count_of(std::size_t n, const char* one, const char* many) {
    return std::to_string(n) + " " + (n == 1 ? one : many);
}

// $INSUNITS from a DXF header, or -1. NotoCAD main does not read it, so a
// KiCad export in inches arrives as numbers in inches and would cut 25.4
// times too small; reading the one header variable here is enough to say so.
int dxf_insunits(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    std::string code, value;
    auto trim = [](std::string& s) {
        while (!s.empty() && (s.back() == '\r' || s.back() == ' ')) s.pop_back();
        const std::size_t b = s.find_first_not_of(' ');
        s.erase(0, b == std::string::npos ? s.size() : b);
    };
    bool want = false;
    while (std::getline(in, code) && std::getline(in, value)) {
        trim(code);
        trim(value);
        if (code == "0" && value == "ENDSEC") return -1;  // the header is first
        if (want) return code == "70" ? std::atoi(value.c_str()) : -1;
        want = code == "9" && value == "$INSUNITS";
    }
    return -1;
}

const char* insunits_name(int u) {
    switch (u) {
        case 1: return "inches";
        case 2: return "feet";
        case 5: return "centimetres";
        case 6: return "metres";
        case 8: return "microinches";
        case 9: return "mils";
        case 13: return "micrometres";
        default: return "not millimetres";
    }
}

constexpr double kPi = 3.14159265358979323846264338327950288;

// LightBurn's smallest power-of-two polygon for a full circle of radius r whose
// sagitta r(1 - cos(pi/n)) is within tol (f07: 32, 64, 128, 256 for D 2, 10,
// 50, 200 at 0.01). Eight is a floor so that a tiny circle stays round.
int lightburn_circle_steps(double r, double tol) {
    int n = 8;
    while (n < (1 << 20) && r * (1.0 - std::cos(kPi / n)) > tol) n *= 2;
    return n;
}

}  // namespace

std::vector<Pt> lightburn_flatten(const Pt& c, const Pt& ax, const Pt& ay, double a0, double sweep, double tol,
                                  bool& closed) {
    const double turn = 2.0 * kPi;
    closed = std::fabs(sweep) >= turn - 1e-9;
    if (closed) sweep = sweep < 0 ? -turn : turn;
    // At most a quarter turn per Bezier; a full circle is four.
    const int pieces = std::max(1, static_cast<int>(std::ceil(std::fabs(sweep) / (kPi / 2.0) - 1e-9)));
    const double phi = sweep / pieces;
    const double r = std::max(std::hypot(ax.x, ax.y), std::hypot(ay.x, ay.y));
    const int full = lightburn_circle_steps(r, tol);
    const int steps = std::max(1, static_cast<int>(std::ceil(std::fabs(phi) / (turn / full) - 1e-9)));
    const double k = 4.0 / 3.0 * std::tan(phi / 4.0);  // 0.5522847498 for a quarter turn

    auto at = [&](double a) {
        return Pt{c.x + ax.x * std::cos(a) + ay.x * std::sin(a), c.y + ax.y * std::cos(a) + ay.y * std::sin(a)};
    };
    auto d = [&](double a) {
        return Pt{-ax.x * std::sin(a) + ay.x * std::cos(a), -ax.y * std::sin(a) + ay.y * std::cos(a)};
    };

    std::vector<Pt> out;
    for (int i = 0; i < pieces; ++i) {
        const double t0 = a0 + phi * i, t1 = t0 + phi;
        const Pt p0 = at(t0), p3 = at(t1), d0 = d(t0), d1 = d(t1);
        const Pt p1{p0.x + k * d0.x, p0.y + k * d0.y}, p2{p3.x - k * d1.x, p3.y - k * d1.y};
        const bool last = i == pieces - 1;
        // One piece's end is the next one's start; only an open arc keeps its last point.
        for (int j = 0; j < steps + (last && !closed ? 1 : 0); ++j) {
            const double t = static_cast<double>(j) / steps, u = 1.0 - t;
            const double b0 = u * u * u, b1 = 3 * u * u * t, b2 = 3 * u * t * t, b3 = t * t * t;
            out.push_back({b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x,
                           b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y});
        }
    }
    return out;
}

LaserRenderer::LaserRenderer(const Database& db, LaserJob& job, DrawingReport& report)
    : db_(db), job_(job), report_(report) {}

LaserRenderer::Resolved LaserRenderer::resolve(const EntityProps& p, const Resolved* enclosing) const {
    Resolved r;
    // R12: inside a block, layer 0 means the INSERT's layer.
    r.layer = (enclosing && p.layer == kLayerZero) ? enclosing->layer : p.layer;
    const bool known = r.layer < db_.layers().size();
    r.visible = known && !db_.layer(r.layer).frozen && !db_.layer(r.layer).off();
    if (enclosing && !enclosing->visible) r.visible = false;

    std::int16_t c = p.color;
    if (c == kColorByBlock) {
        // Outside a block NotoCAD's own display reads BYBLOCK as BYLAYER
        // (src/gui/qpainter_renderer.cpp, effective_color); match it.
        c = enclosing ? enclosing->color : kColorByLayer;
    }
    if (c == kColorByLayer) c = known ? db_.layer(r.layer).visible_color() : std::int16_t{7};
    if (c < 1 || c > 255) c = 7;
    r.color = c;
    return r;
}

void LaserRenderer::survey(const Entity& e, const Vec3& normal, bool visible) {
    if (!visible) return;
    switch (e.type()) {
        case EntityType::Insert: return;  // its contents are surveyed one by one
        case EntityType::Solid: ++report_.solids; break;
        case EntityType::Proxy: ++report_.proxies; return;
        default: break;
    }
    if (!flat(normal)) ++report_.not_flat;
}

// The mirror of draw_definition() in NotoCAD's src/core/insert.cpp: the same
// walk, in the same order, under the same depth guard, so that the n-th
// begin_entity() inside an INSERT is for nested_[n]. Each entry is checked by
// address as it arrives, so if the two ever disagree the renderer notices. The
// handle cannot do this part: every entity of an INSERT carries the INSERT's.
void LaserRenderer::expand(const BlockDef& def, const Mat4& m, const Resolved& enclosing, int depth) {
    if (depth >= kMaxBlockDepth) return;
    for (const EntityPtr& e : def.entities) {
        if (!e) continue;
        const Resolved r = resolve(e->props(), &enclosing);
        if (e->type() == EntityType::Insert) {
            const Insert& ins = static_cast<const Insert&>(*e);
            if (const BlockDef* inner = ins.definition()) {
                for (std::int16_t row = 0; row < ins.rows(); ++row) {
                    for (std::int16_t col = 0; col < ins.columns(); ++col) {
                        expand(*inner, m * ins.placement_for(row, col), r, depth + 1);
                    }
                }
            }
            continue;
        }
        nested_.push_back({&e->props(), r});
        if (!r.visible) ++report_.hidden_in_blocks;
        survey(*e, m.transform_vector(e->props().normal), r.visible);
    }
}

// True when the entity was a circle, arc or ellipse in the XY plane and its
// path is now in the job. Anything tilted is left to draw(), which projects it.
bool LaserRenderer::flatten_lightburn(const Entity& e) {
    const Vec3& n = e.props().normal;
    if (!(n.z > 0.0) || std::hypot(n.x, n.y) > 1e-9 * n.z) return false;
    Pt c{}, ax{}, ay{};
    double a0 = 0.0, sweep = 0.0;
    // A full turn starts at 180 degrees and goes clockwise (f07).
    switch (e.type()) {
        case EntityType::Circle: {
            const Circle& k = static_cast<const Circle&>(e);
            c = {k.center().x, k.center().y};
            ax = {k.radius(), 0.0};
            ay = {0.0, k.radius()};
            a0 = kPi;
            sweep = -2.0 * kPi;
            break;
        }
        case EntityType::Arc: {
            const Arc& k = static_cast<const Arc&>(e);
            c = {k.center().x, k.center().y};
            ax = {k.radius(), 0.0};
            ay = {0.0, k.radius()};
            a0 = k.start_angle();
            sweep = k.sweep();  // R12 arcs run counterclockwise, start to end
            break;
        }
        case EntityType::Ellipse: {
            const Ellipse& k = static_cast<const Ellipse&>(e);
            c = {k.center().x, k.center().y};
            ax = {k.major_axis().x, k.major_axis().y};
            ay = {k.minor_axis().x, k.minor_axis().y};
            a0 = k.start_param();
            sweep = k.sweep();
            if (k.is_full()) {
                a0 = kPi;
                sweep = -2.0 * kPi;
            }
            break;
        }
        default: return false;
    }
    if (e.type() == EntityType::Arc && sweep >= 2.0 * kPi - 1e-9) {
        a0 = kPi;
        sweep = -2.0 * kPi;
    }
    bool closed = false;
    LaserPath p;
    p.pen = current_.color;
    p.pts = lightburn_flatten(c, ax, ay, a0, sweep, tol_, closed);
    p.closed = closed;
    if (p.pts.size() < 2) return false;
    job_.paths.push_back(std::move(p));
    ++report_.paths;
    return true;
}

void LaserRenderer::begin_entity(const EntityProps& props, Handle source) {
    pending_width_ = 0.0;
    // An INSERT's contents arrive with the INSERT's handle, which is the only
    // thing a person can pick to name them.
    source_ = source;
    suppress_ = false;
    // Not selected: nothing of it is counted or cut. An INSERT's contents
    // carry the INSERT's handle, so they follow it; the nesting state is left
    // alone, as the next selected entity resets it.
    unselected_ = only_ != nullptr && !std::binary_search(only_->begin(), only_->end(), source);
    if (unselected_) return;
    if (in_insert_ && next_ < nested_.size() && nested_[next_].props == &props) {
        current_ = nested_[next_++].resolved;
        return;
    }
    const Entity* top = source != kNullHandle ? db_.get(source) : nullptr;
    // An entity's own props are the top-level call; the same handle with other
    // props is something inside the INSERT that nested_ did not predict.
    if (top != nullptr && &top->props() == &props) {
        const Entity& e = *top;
        ++report_.entities;
        current_ = resolve(props, nullptr);
        in_insert_ = false;
        nested_.clear();
        next_ = 0;
        if (e.type() == EntityType::Insert) {
            const Insert& ins = static_cast<const Insert&>(e);
            in_insert_ = true;
            insert_ = current_;
            if (const BlockDef* def = ins.definition()) {
                for (std::int16_t row = 0; row < ins.rows(); ++row) {
                    for (std::int16_t col = 0; col < ins.columns(); ++col) {
                        expand(*def, ins.placement_for(row, col), current_, 0);
                    }
                }
            }
        }
        survey(e, props.normal, current_.visible);
        if (lightburn_ && !in_insert_ && current_.visible) suppress_ = flatten_lightburn(e);
        return;
    }
    // Not identified: NotoCAD's expansion and ours disagree, or a caller drew
    // something that is not in the database. Inside an INSERT, take BYBLOCK
    // and layer 0 from the outermost one -- right for a single level.
    ++report_.unmatched;
    current_ = resolve(props, in_insert_ ? &insert_ : nullptr);
}

void LaserRenderer::polyline(const Vec3* pts, std::size_t count, bool closed) {
    const double width = pending_width_;
    pending_width_ = 0.0;
    if (unselected_ || !current_.visible || suppress_) return;
    if (count < 2) {
        ++report_.degenerate;
        return;
    }
    if (width * scale_ >= kRasterWidth) ++report_.wide_polylines;
    LaserPath p;
    p.pen = current_.color;
    p.closed = closed;
    p.id = static_cast<int>(job_.paths.size());
    p.source = source_;
    p.pts.reserve(count);
    for (std::size_t i = 0; i < count; ++i) p.pts.push_back({pts[i].x, pts[i].y});
    job_.paths.push_back(std::move(p));
    ++report_.paths;
}

bool render_drawing(const Database& db, const DrawingOptions& opts, LaserJob& job, DrawingReport& report,
                    std::string& error) {
    if (!(opts.chord_tolerance > 0.0) || !std::isfinite(opts.chord_tolerance)) {
        error = "the chord tolerance must be a positive number of mm";
        return false;
    }
    const DrawingUnits units = opts.units == DrawingUnits::Auto ? DrawingUnits::Mm : opts.units;
    const double scale = mm_per_unit(units);
    report.units = units;

    // The job origin is a fixed point in the drawing, so that partial jobs
    // register with each other: the LASER UCS, else world 0,0.
    job.origin = {0.0, 0.0};
    const UcsId laser = db.find_ucs("LASER");
    if (laser != kInvalidUcs) {
        const Ucs u = db.ucs(laser).ucs.normalized();
        report.laser_ucs = true;
        job.origin = {u.origin.x, u.origin.y};
        const double eps = 1e-9;
        report.laser_ucs_rotated = std::abs(u.xdir.x - 1.0) > eps || std::abs(u.xdir.y) > eps ||
                                   std::abs(u.xdir.z) > eps || std::abs(u.ydir.y - 1.0) > eps;
        if (report.laser_ucs_rotated) {
            report.warnings.push_back(
                "the LASER UCS is rotated; rotation is not supported yet, so only its origin is used");
        }
    }
    if (opts.has_origin) job.origin = opts.origin;
    job.origin = {job.origin.x * scale, job.origin.y * scale};

    std::vector<Handle> chosen = opts.selection;
    std::sort(chosen.begin(), chosen.end());
    for (const Handle h : db.order()) {
        if (opts.has_selection && !std::binary_search(chosen.begin(), chosen.end(), h)) continue;
        const Entity* e = db.get(h);
        if (e && !entity_visible(db, *e)) ++report.hidden;
    }

    DrawContext ctx;
    // The tolerance is in mm; the drawing is drawn in its own units.
    const double tol_units = opts.chord_tolerance / scale;
    ctx.chord_tolerance = tol_units;  // clip_active false: everything
    if (opts.has_view) {
        ctx.clip_active = true;
        ctx.clip_origin = opts.view.origin;
        ctx.clip_x = opts.view.ax;
        ctx.clip_y = opts.view.ay;
        ctx.clip_min_x = opts.view.min_x;
        ctx.clip_max_x = opts.view.max_x;
        ctx.clip_min_y = opts.view.min_y;
        ctx.clip_max_y = opts.view.max_y;
    }
    LaserRenderer r(db, job, report);
    if (opts.flatten == FlattenMode::LightBurn) r.set_lightburn_flatten(tol_units);
    r.set_unit_scale(scale);
    if (opts.has_selection) r.set_selection(&chosen);
    draw_database(db, ctx, r);
    if (scale != 1.0) {
        for (LaserPath& p : job.paths) {
            for (Pt& q : p.pts) q = {q.x * scale, q.y * scale};
        }
    }

    if (report.wide_polylines) {
        report.warnings.push_back(count_of(report.wide_polylines, "polyline is", "polylines are") + " " +
                                  "0.5 mm or wider, which should engrave as raster; raster is not supported "
                                  "yet, so they cut along their centre lines");
    }
    if (report.solids) {
        report.warnings.push_back(count_of(report.solids, "SOLID", "SOLIDs") +
                                  " should engrave as raster; raster is not supported yet, so they cut as "
                                  "outlines");
    }
    if (report.not_flat) {
        report.warnings.push_back(count_of(report.not_flat, "entity is", "entities are") +
                                  " not parallel to the XY plane; they cut as their projection onto it");
    }
    if (report.proxies) {
        report.warnings.push_back(count_of(report.proxies, "entity has", "entities have") +
                                  " no class in NotoCAD (HATCH, IMAGE, ...) and nothing is cut for them");
    }
    if (report.unmatched) {
        report.warnings.push_back(count_of(report.unmatched, "entity", "entities") +
                                  " could not be matched to the drawing; BYBLOCK colour and layer 0 were "
                                  "taken from the outermost INSERT, which is right only one level deep");
    }
    return true;
}

bool read_pen_table(const std::string& text, std::vector<LaserLayer>& pens, std::string& error) {
    // An .lpt is the layer lines of an .ljob, so the .ljob reader reads it;
    // only its line numbers need putting back.
    LaserJob job;
    if (!read_ljob("ljob 1\n" + text, job, error)) {
        unsigned long line = 0;
        if (std::sscanf(error.c_str(), "line %lu:", &line) == 1 && line > 1) {
            error = "line " + std::to_string(line - 1) + error.substr(error.find(':'));
        }
        return false;
    }
    if (!job.paths.empty() || job.bed_w > 0.0 || job.origin.x != 0.0 || job.origin.y != 0.0) {
        error = "a pen table holds 'layer' lines only";
        return false;
    }
    pens = std::move(job.layers);
    return true;
}

bool read_pen_table_file(const std::string& path, std::vector<LaserLayer>& pens, std::string& error) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        error = "cannot read '" + path + "'";
        return false;
    }
    const std::string text{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    if (!read_pen_table(text, pens, error)) {
        error = path + ": " + error;
        return false;
    }
    return true;
}

void apply_pens(LaserJob& job, const std::vector<LaserLayer>& pens, DrawingReport& report) {
    std::vector<int> used;
    for (const LaserPath& p : job.paths) used.push_back(p.pen);
    std::sort(used.begin(), used.end());
    used.erase(std::unique(used.begin(), used.end()), used.end());

    job.layers.clear();
    std::string unset;
    for (const int pen : used) {
        const auto it = std::find_if(pens.begin(), pens.end(), [pen](const LaserLayer& l) { return l.pen == pen; });
        if (it != pens.end()) {
            job.layers.push_back(*it);
            continue;
        }
        LaserLayer l;
        l.pen = pen;
        l.color = pen;
        l.mode = LayerMode::Auto;
        l.min_power = 0.0;
        l.max_power = 0.0;
        job.layers.push_back(l);
        report.unset_pens.push_back(pen);
        unset += (unset.empty() ? "" : ", ") + std::to_string(pen);
    }
    if (!unset.empty()) {
        report.warnings.push_back(std::string(report.unset_pens.size() == 1 ? "pen " : "pens ") + unset +
                                  (report.unset_pens.size() == 1 ? " has" : " have") +
                                  " no pen-table entry and will run at power 0");
    }
}

bool dxf_to_job(const std::string& dxf_path, const DrawingOptions& opts, const std::vector<LaserLayer>& pens,
                LaserJob& job, DrawingReport& report, std::string& error) {
    Database db;
    const DxfReadResult read = read_dxf_file(db, dxf_path);
    if (!read.ok) {
        error = dxf_path + ": " + read.error;
        return false;
    }
    if (read.newer_version) {
        report.warnings.push_back("the file is " + read.version + ", newer than this reader claims to handle");
    }
    DrawingOptions o = opts;
    if (o.units == DrawingUnits::Auto) {
        const int u = dxf_insunits(dxf_path);
        if (u == 1) {
            o.units = DrawingUnits::Inch;
        } else if (u == 4) {
            o.units = DrawingUnits::Mm;
        } else if (u > 0) {
            // Guessing would cut at the wrong size; say what to do instead.
            error = dxf_path + ": the file says its units are " + insunits_name(u) + " ($INSUNITS " +
                    std::to_string(u) + "), which is not mm or inches; give --units mm or --units in";
            return false;
        } else {
            o.units = DrawingUnits::Mm;
            report.warnings.push_back(
                "the file does not say what its units are ($INSUNITS); taking drawing units as mm "
                "(--units in if it is in inches)");
        }
    }
    if (read.unresolved_inserts) {
        report.warnings.push_back(count_of(read.unresolved_inserts, "INSERT names", "INSERTs name") +
                                  " a block the file never defines; nothing is cut for them");
    }

    LaserJob out;
    const std::size_t slash = dxf_path.find_last_of("/\\");
    out.source = slash == std::string::npos ? dxf_path : dxf_path.substr(slash + 1);
    const std::size_t dot = out.source.find_last_of('.');
    out.name = dot == std::string::npos || dot == 0 ? out.source : out.source.substr(0, dot);
    if (!render_drawing(db, o, out, report, error)) return false;
    apply_pens(out, pens, report);
    job = std::move(out);
    return true;
}

}  // namespace ncad::laser
