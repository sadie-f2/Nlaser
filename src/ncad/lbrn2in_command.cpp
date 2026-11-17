// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

#include "ncad/laser/lbrn2in_command.hpp"

#include "ncad/database.hpp"
#include "ncad/entities.hpp"
#include "ncad/laser/lbrn2.hpp"
#include "ncad/laser/svgin_command.hpp"
#include "ncad/paths.hpp"

#include <cmath>
#include <cstdio>
#include <set>

namespace ncad::laser {

Step Lbrn2InCommand::start(CommandContext&) {
    if (!session_) return Step::failed("the laser backend is not installed");
    Prompt p;
    p.kind = PromptKind::String;
    p.message = "LightBurn project to import";
    p.file = FileIntent::Open;
    p.file_extension = "lbrn2";
    return Step::ask(std::move(p));
}

Step Lbrn2InCommand::next(CommandContext& ctx, const InputValue& value) {
    if (value.kind != InputKind::String || value.text.empty()) return Step::cancelled();

    Lbrn2Options opts;
    opts.keep_ellipses = true;
    const std::string corner = ctx.db.sysvars().get_string(session_->lbrn2_corner);
    if (!ruida::parse_machine_corner(corner, opts.machine.origin)) {
        return Step::failed("LBRN2CORNER must be bl, br, tl or tr, not '" + corner + "'");
    }
    // The bed turns machine distances into bed coordinates. No device is not an
    // error here: the import then keeps only the axis directions.
    DeviceConfig cfg;
    std::string ignore;
    if (resolve_laser_device(*session_, ctx.db, cfg, ignore)) {
        opts.machine.bed_w_mm = cfg.bed_w_mm;
        opts.machine.bed_h_mm = cfg.bed_h_mm;
    }

    LaserJob job;
    Lbrn2Info info;
    std::string error;
    if (!read_lbrn2_file(expand_user_path(value.text), job, info, opts, error)) return Step::failed(error);

    Database& db = ctx.db;
    std::vector<int> aci;
    const std::vector<LaserLayer> pens = lbrn2_pens(job, &aci);

    // One layer per CutSetting, named for the LightBurn layer. Two layers with
    // one name (LightBurn allows it) would merge, and so would their pens, so
    // the later one takes its index as a suffix.
    std::set<std::string> taken;
    std::size_t layers_created = 0;
    std::vector<LayerId> layer_of(job.layers.size(), kLayerZero);
    for (std::size_t i = 0; i < job.layers.size(); ++i) {
        std::string name;
        for (const auto& [idx, nm] : info.layer_names) {
            if (idx == job.layers[i].pen) name = nm;
        }
        if (name.empty()) name = "LB" + std::to_string(job.layers[i].pen);
        name = dxf_layer_name(name);
        if (!taken.insert(name).second) {
            name += "_" + std::to_string(job.layers[i].pen);
            taken.insert(name);
        }
        LayerId id = db.find_layer(name);
        if (id == kInvalidLayer) {
            id = db.add_layer(name, static_cast<std::int16_t>(aci[i]));
            ++layers_created;
        }
        layer_of[i] = id;
    }
    auto props_for = [&](int index) {
        EntityProps props;
        for (std::size_t i = 0; i < job.layers.size(); ++i) {
            if (job.layers[i].pen != index) continue;
            props.layer = layer_of[i];
            // BYLAYER unless the layer (one that existed already) is another colour.
            props.color = db.layer(props.layer).visible_color() == aci[i] ? kColorByLayer
                                                                          : static_cast<std::int16_t>(aci[i]);
        }
        return props;
    };

    std::size_t polylines = 0, circles = 0, ellipses = 0;
    for (const LaserPath& p : job.paths) {
        auto pl = std::make_unique<Polyline>();
        for (const Pt& q : p.pts) pl->add({q.x, q.y, 0.0});
        pl->set_closed(p.closed);
        pl->props() = props_for(p.pen);
        db.add(std::move(pl));
        ++polylines;
    }
    for (const Lbrn2Ellipse& e : info.ellipses) {
        const Vec3 centre{e.centre.x, e.centre.y, 0.0};
        std::unique_ptr<Entity> ent;
        if (e.ratio > 1.0 - 1e-9) {
            ent = std::make_unique<Circle>(centre, std::hypot(e.major.x, e.major.y));
            ++circles;
        } else {
            ent = std::make_unique<Ellipse>(centre, Vec3{e.major.x, e.major.y, 0.0}, e.ratio);
            ++ellipses;
        }
        ent->props() = props_for(e.pen);
        db.add(std::move(ent));
    }

    session_->imported_pens = pens;
    session_->has_imported_pens = true;
    db.sysvars().set_string(session_->pens, "");

    char head[200];
    std::snprintf(head, sizeof head,
                  "LBRN2IN: %zu polylines, %zu circles, %zu ellipses, %zu new layers, %zu pens for LASER", polylines,
                  circles, ellipses, layers_created, pens.size());
    std::string msg = head;
    for (const std::string& w : info.warnings) msg += "\nwarning: " + w;
    return Step::done(msg);
}

}  // namespace ncad::laser
