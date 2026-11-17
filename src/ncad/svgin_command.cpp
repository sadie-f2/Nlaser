// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

#include "ncad/laser/svgin_command.hpp"

#include "ncad/database.hpp"
#include "ncad/entities.hpp"

#include <cstdio>

namespace ncad::laser {

namespace {

// DXF forbids these in a table name; an Inkscape label can hold any of them.
std::string layer_name(const std::string& raw) {
    std::string s = raw;
    for (char& c : s) {
        if (std::string("<>/\\\":;?*|=`").find(c) != std::string::npos) c = '_';
    }
    return s.empty() ? "0" : s;
}

}  // namespace

SvgInReport add_svg_to_drawing(Database& db, const SvgImport& svg) {
    SvgInReport rep;

    // Layers first, in the order the SVG met them, so the layer table reads
    // like the document.
    for (const SvgLayer& l : svg.layers) {
        const std::string name = layer_name(l.name);
        if (db.find_layer(name) == kInvalidLayer) {
            db.add_layer(name, static_cast<std::int16_t>(l.aci));
            ++rep.layers_created;
        }
    }

    for (const SvgShape& s : svg.shapes) {
        const LayerId layer = db.find_layer(layer_name(s.layer));
        EntityProps props;
        props.layer = layer == kInvalidLayer ? kLayerZero : layer;
        // BYLAYER when the layer already has this colour, so a layer recoloured
        // later takes its entities with it; explicit only where they differ.
        props.color = db.layer(props.layer).visible_color() == s.aci ? kColorByLayer
                                                                     : static_cast<std::int16_t>(s.aci);
        if (s.curve) {
            std::vector<Vec3> cps;
            cps.reserve(s.pts.size());
            for (const Pt& p : s.pts) cps.push_back({p.x, p.y, 0.0});
            // Clamped cubic B-spline through a chain of Beziers: end knots
            // repeated four times, each interior joint three times, which makes
            // every segment its own Bezier and joins them at C0.
            const std::size_t segs = (s.pts.size() - 1) / 3;
            std::vector<double> knots(4, 0.0);
            for (std::size_t k = 1; k < segs; ++k) knots.insert(knots.end(), 3, static_cast<double>(k));
            knots.insert(knots.end(), 4, static_cast<double>(segs));
            auto sp = std::make_unique<Spline>(3, std::move(cps), std::move(knots));
            sp->props() = props;
            db.add(std::move(sp));
            ++rep.splines;
        } else {
            auto pl = std::make_unique<Polyline>();
            for (const Pt& p : s.pts) pl->add({p.x, p.y, 0.0});
            pl->set_closed(s.closed);
            if (s.width > 0.0) pl->set_uniform_width(s.width);
            pl->props() = props;
            db.add(std::move(pl));
            ++rep.polylines;
        }
    }
    return rep;
}

Step SvgInCommand::start(CommandContext&) {
    if (!session_) return Step::failed("the laser backend is not installed");
    Prompt p;
    p.kind = PromptKind::String;
    p.message = "SVG file to import";
    p.file = FileIntent::Open;
    p.file_extension = "svg";
    return Step::ask(std::move(p));
}

Step SvgInCommand::next(CommandContext& ctx, const InputValue& value) {
    if (value.kind != InputKind::String || value.text.empty()) return Step::cancelled();

    SvgOptions opts;
    const double px = ctx.db.sysvars().get_real(session_->svg_px);
    if (px > 0.0) opts.px_per_in = px;
    SvgImport svg;
    std::string error;
    if (!import_svg_file(value.text, opts, svg, error)) return Step::failed(error);

    const SvgInReport rep = add_svg_to_drawing(ctx.db, svg);
    char head[160];
    std::snprintf(head, sizeof head, "SVGIN: %zu polylines, %zu splines, %zu new layers (%.1f x %.1f mm)",
                  rep.polylines, rep.splines, rep.layers_created, svg.width_mm, svg.height_mm);
    std::string msg = head;
    for (const std::string& w : svg.warnings) msg += "\nwarning: " + w;
    return Step::done(msg);
}

}  // namespace ncad::laser
