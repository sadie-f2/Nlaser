// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

#include "ncad/laser/plan.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <utility>

namespace ncad::laser {

namespace {

double dist(const Pt& a, const Pt& b) { return std::hypot(a.x - b.x, a.y - b.y); }

double length(const LaserPath& p) {
    double s = 0.0;
    for (std::size_t i = 1; i < p.pts.size(); ++i) s += dist(p.pts[i - 1], p.pts[i]);
    if (p.closed && p.pts.size() > 1) s += dist(p.pts.back(), p.pts.front());
    return s;
}

// Endpoints bucketed on a grid of the join tolerance, so a lookup checks the
// nine cells around a point instead of every path.
class EndGrid {
public:
    explicit EndGrid(double tol) : cell_(std::max(tol, 1e-9)) {}

    void add(const Pt& p, std::size_t id) { cells_[key(p)].push_back(id); }

    template <typename F>
    void near(const Pt& p, F&& visit) const {
        const auto k = key(p);
        for (std::int64_t dx = -1; dx <= 1; ++dx) {
            for (std::int64_t dy = -1; dy <= 1; ++dy) {
                const auto it = cells_.find({k.first + dx, k.second + dy});
                if (it == cells_.end()) continue;
                for (std::size_t id : it->second) visit(id);
            }
        }
    }

private:
    std::pair<std::int64_t, std::int64_t> key(const Pt& p) const {
        return {static_cast<std::int64_t>(std::floor(p.x / cell_)),
                static_cast<std::int64_t>(std::floor(p.y / cell_))};
    }
    double cell_;
    std::map<std::pair<std::int64_t, std::int64_t>, std::vector<std::size_t>> cells_;
};

}  // namespace

std::vector<LaserPath> join_paths(const std::vector<LaserPath>& paths, double tol, PlanReport* report) {
    std::vector<LaserPath> out;
    std::vector<const LaserPath*> open;
    for (const LaserPath& p : paths) {
        if (p.pts.size() < 2) continue;
        if (p.closed) {
            LaserPath c = p;
            if (c.pts.size() > 2 && dist(c.pts.front(), c.pts.back()) <= tol) c.pts.pop_back();
            out.push_back(std::move(c));
        } else {
            open.push_back(&p);
        }
    }

    // Endpoint ids: 2*i is path i's start, 2*i+1 its end.
    EndGrid grid(tol);
    for (std::size_t i = 0; i < open.size(); ++i) {
        grid.add(open[i]->pts.front(), 2 * i);
        grid.add(open[i]->pts.back(), 2 * i + 1);
    }
    std::vector<bool> used(open.size(), false);

    // The nearest unused endpoint within tol of `p`, or SIZE_MAX.
    auto find = [&](const Pt& p) {
        std::size_t best = SIZE_MAX;
        double best_d = tol;
        grid.near(p, [&](std::size_t id) {
            const std::size_t i = id / 2;
            if (used[i]) return;
            const Pt& q = (id % 2 == 0) ? open[i]->pts.front() : open[i]->pts.back();
            const double d = dist(p, q);
            if (d <= best_d && (best == SIZE_MAX || d < best_d || id < best)) {
                best = id;
                best_d = d;
            }
        });
        return best;
    };

    for (std::size_t s = 0; s < open.size(); ++s) {
        if (used[s]) continue;
        used[s] = true;
        std::vector<Pt> chain = open[s]->pts;
        const int pen = open[s]->pen;

        // Forward from the end, then backward from the start.
        for (int pass = 0; pass < 2; ++pass) {
            for (;;) {
                const std::size_t id = find(chain.back());
                if (id == SIZE_MAX) break;
                const std::size_t i = id / 2;
                used[i] = true;
                const std::vector<Pt>& q = open[i]->pts;
                if (id % 2 == 0) {
                    chain.insert(chain.end(), q.begin() + 1, q.end());
                } else {
                    chain.insert(chain.end(), q.rbegin() + 1, q.rend());
                }
                if (report) ++report->joins;
            }
            std::reverse(chain.begin(), chain.end());
        }
        // Two reversals: the chain is back in the first path's direction.

        LaserPath p;
        p.pen = pen;
        p.pts = std::move(chain);
        if (p.pts.size() > 2 && dist(p.pts.front(), p.pts.back()) <= tol) {
            p.pts.pop_back();
            p.closed = true;
            if (report) ++report->closed_by_join;
        }
        out.push_back(std::move(p));
    }
    return out;
}

namespace {

struct Bounds {
    double x0{0}, y0{0}, x1{0}, y1{0};
    bool contains(const Bounds& o) const { return o.x0 >= x0 && o.x1 <= x1 && o.y0 >= y0 && o.y1 <= y1; }
};

Bounds bounds_of(const LaserPath& p) {
    Bounds b{p.pts[0].x, p.pts[0].y, p.pts[0].x, p.pts[0].y};
    for (const Pt& q : p.pts) {
        b.x0 = std::min(b.x0, q.x);
        b.x1 = std::max(b.x1, q.x);
        b.y0 = std::min(b.y0, q.y);
        b.y1 = std::max(b.y1, q.y);
    }
    return b;
}

double area(const LaserPath& p) {
    double a = 0.0;
    for (std::size_t i = 0, j = p.pts.size() - 1; i < p.pts.size(); j = i++) {
        a += (p.pts[j].x * p.pts[i].y) - (p.pts[i].x * p.pts[j].y);
    }
    return std::fabs(a) * 0.5;
}

bool inside(const Pt& q, const LaserPath& poly) {
    bool in = false;
    for (std::size_t i = 0, j = poly.pts.size() - 1; i < poly.pts.size(); j = i++) {
        const Pt& a = poly.pts[i];
        const Pt& b = poly.pts[j];
        if ((a.y > q.y) != (b.y > q.y)) {
            const double x = a.x + (q.y - a.y) * (b.x - a.x) / (b.y - a.y);
            if (q.x < x) in = !in;
        }
    }
    return in;
}

}  // namespace

std::vector<std::vector<std::size_t>> containment(const std::vector<LaserPath>& paths) {
    const std::size_t n = paths.size();
    std::vector<Bounds> b(n);
    std::vector<double> a(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        b[i] = bounds_of(paths[i]);
        if (paths[i].closed && paths[i].pts.size() >= 3) a[i] = area(paths[i]);
    }
    std::vector<std::vector<std::size_t>> contains(n);
    for (std::size_t outer = 0; outer < n; ++outer) {
        if (a[outer] <= 0.0) continue;
        for (std::size_t in = 0; in < n; ++in) {
            if (in == outer || !b[outer].contains(b[in])) continue;
            if (paths[in].closed && a[in] >= a[outer]) continue;  // equal shapes contain neither
            if (inside(paths[in].pts[0], paths[outer])) contains[outer].push_back(in);
        }
    }
    return contains;
}

namespace {

// Where a path would start if cut next from `head`, and how far away that is.
struct StartChoice {
    std::size_t vertex{0};
    bool reverse{false};
    double distance{0.0};
};

StartChoice choose_start(const LaserPath& p, const Pt& head, const PlanOptions& opts) {
    StartChoice c;
    if (!p.closed) {
        const double a = dist(head, p.pts.front());
        const double b = dist(head, p.pts.back());
        c.reverse = b < a;
        c.distance = std::min(a, b);
        return c;
    }
    const std::size_t n = p.pts.size();
    std::size_t nearest = 0;
    double nearest_d = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < n; ++i) {
        const double d = dist(head, p.pts[i]);
        if (d < nearest_d) {
            nearest_d = d;
            nearest = i;
        }
    }
    const double cos_limit = std::cos(opts.corner_angle * 3.14159265358979323846 / 180.0);
    std::size_t corner = SIZE_MAX;
    double corner_d = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < n; ++i) {
        const Pt& prev = p.pts[(i + n - 1) % n];
        const Pt& here = p.pts[i];
        const Pt& next = p.pts[(i + 1) % n];
        const double ax = here.x - prev.x, ay = here.y - prev.y;
        const double bx = next.x - here.x, by = next.y - here.y;
        const double la = std::hypot(ax, ay), lb = std::hypot(bx, by);
        if (la <= 0.0 || lb <= 0.0) continue;
        if ((ax * bx + ay * by) / (la * lb) > cos_limit) continue;  // turn too small
        const double d = dist(head, here);
        if (d < corner_d) {
            corner_d = d;
            corner = i;
        }
    }
    if (corner != SIZE_MAX && corner_d <= nearest_d + opts.corner_slack) {
        c.vertex = corner;
        c.distance = corner_d;
    } else {
        c.vertex = nearest;
        c.distance = nearest_d;
    }
    return c;
}

LaserPath apply_start(const LaserPath& p, const StartChoice& c) {
    LaserPath out = p;
    if (p.closed) {
        std::rotate(out.pts.begin(), out.pts.begin() + static_cast<std::ptrdiff_t>(c.vertex), out.pts.end());
    } else if (c.reverse) {
        std::reverse(out.pts.begin(), out.pts.end());
    }
    return out;
}

}  // namespace

std::vector<LaserPath> order_paths(const std::vector<LaserPath>& paths, Pt& head, const PlanOptions& opts,
                                   PlanReport* report) {
    const std::size_t n = paths.size();
    const auto contains = containment(paths);
    // waiting[i]: how many paths inside i are still to be cut.
    std::vector<std::size_t> waiting(n, 0);
    std::vector<std::vector<std::size_t>> containers(n);
    for (std::size_t o = 0; o < n; ++o) {
        waiting[o] = contains[o].size();
        for (std::size_t in : contains[o]) containers[in].push_back(o);
    }

    std::vector<bool> done(n, false);
    std::vector<LaserPath> out;
    out.reserve(n);
    for (std::size_t step = 0; step < n; ++step) {
        std::size_t best = SIZE_MAX;
        StartChoice best_c;
        for (std::size_t i = 0; i < n; ++i) {
            if (done[i] || waiting[i] != 0) continue;
            const StartChoice c = choose_start(paths[i], head, opts);
            if (best == SIZE_MAX || c.distance < best_c.distance) {
                best = i;
                best_c = c;
            }
        }
        if (best == SIZE_MAX) {
            // Only a containment cycle gets here, which strict area ordering
            // rules out; fall back to drawing order rather than loop.
            for (std::size_t i = 0; i < n; ++i) {
                if (!done[i]) {
                    best = i;
                    best_c = choose_start(paths[i], head, opts);
                    break;
                }
            }
        }
        done[best] = true;
        for (std::size_t o : containers[best]) --waiting[o];

        LaserPath p = apply_start(paths[best], best_c);
        if (report) {
            report->travel_mm += dist(head, p.pts.front());
            report->cut_mm += length(p);
        }
        head = p.closed ? p.pts.front() : p.pts.back();
        out.push_back(std::move(p));
    }
    return out;
}

bool plan_job(const LaserJob& raw, LaserJob& planned, const PlanOptions& opts, PlanReport& report,
              std::string& error) {
    report = PlanReport{};
    report.paths_in = raw.paths.size();

    std::vector<const LaserLayer*> layers;
    for (const LaserLayer& l : raw.layers) layers.push_back(&l);
    std::stable_sort(layers.begin(), layers.end(), [](const LaserLayer* a, const LaserLayer* b) {
        return a->order != b->order ? a->order < b->order : a->pen < b->pen;
    });

    for (const LaserPath& p : raw.paths) {
        if (!raw.layer(p.pen)) {
            error = "a path uses pen " + std::to_string(p.pen) + ", which has no layer record";
            return false;
        }
    }

    LaserJob out = raw;
    out.planned = true;
    out.paths.clear();
    Pt head = raw.origin;

    for (const LaserLayer* l : layers) {
        std::vector<LaserPath> mine;
        for (const LaserPath& p : raw.paths) {
            if (p.pen == l->pen) mine.push_back(p);
        }
        if (mine.empty()) continue;
        if (l->mode == LayerMode::Skip) {
            report.skipped += mine.size();
            continue;
        }
        if (l->mode == LayerMode::Raster) {
            error = "layer " + std::to_string(l->pen) + " is raster; raster engraving is not supported yet";
            return false;
        }
        const std::vector<LaserPath> joined = join_paths(mine, opts.join_tol, &report);
        for (LaserPath& p : order_paths(joined, head, opts, &report)) out.paths.push_back(std::move(p));
    }
    report.paths_out = out.paths.size();
    planned = std::move(out);
    return true;
}

}  // namespace ncad::laser
