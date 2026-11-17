// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

#include "ncad/laser/ruida_job.hpp"

#include <algorithm>

namespace ncad::laser {

bool job_to_rd(const LaserJob& job, ruida::RdJob& out, std::string& error) {
    if (!job.planned) {
        error = "the job is not planned; run 'ncad-laser plan' first";
        return false;
    }
    if (!job.rasters.empty()) {
        error = "raster encoding not implemented yet (the job has " + std::to_string(job.rasters.size()) +
                " raster record(s))";
        return false;
    }
    ruida::RdJob rd;
    rd.origin = {job.origin.x, job.origin.y};
    rd.start = job.start == StartMode::Absolute ? ruida::RdStart::Absolute : ruida::RdStart::Current;
    std::vector<int> pen_of_layer;
    auto layer_index = [&](int pen) -> std::size_t {
        for (std::size_t i = 0; i < pen_of_layer.size(); ++i) {
            if (pen_of_layer[i] == pen) return i;
        }
        const LaserLayer* l = job.layer(pen);
        ruida::RdLayer r;
        r.rgb = aci_rgb(l->color);
        r.speed_mm_s = l->speed;
        r.min_power = l->min_power;
        r.max_power = l->max_power;
        if (l->has_power2) {
            r.min_power2 = l->min_power2;
            r.max_power2 = l->max_power2;
        }
        r.air = l->air;
        r.passes = l->passes;
        rd.layers.push_back(r);
        pen_of_layer.push_back(pen);
        return rd.layers.size() - 1;
    };
    for (const LaserPath& p : job.paths) {
        const LaserLayer* l = job.layer(p.pen);
        if (!l) {
            error = "a path uses pen " + std::to_string(p.pen) + ", which has no layer record";
            return false;
        }
        if (l->mode == LayerMode::Skip) continue;
        if (l->mode == LayerMode::Raster) {
            error = "layer " + std::to_string(p.pen) + " is raster; raster engraving is not supported yet";
            return false;
        }
        ruida::RdPath r;
        r.layer = layer_index(p.pen);
        r.closed = p.closed;
        for (const Pt& q : p.pts) r.pts.push_back({q.x, q.y});
        rd.paths.push_back(std::move(r));
    }
    out = std::move(rd);
    return true;
}

bool job_bounds(const LaserJob& job, Pt& min, Pt& max) {
    bool any = false;
    for (const LaserPath& p : job.paths) {
        for (const Pt& q : p.pts) {
            if (!any) {
                min = max = q;
                any = true;
            }
            min.x = std::min(min.x, q.x);
            min.y = std::min(min.y, q.y);
            max.x = std::max(max.x, q.x);
            max.y = std::max(max.y, q.y);
        }
    }
    return any;
}

}  // namespace ncad::laser
