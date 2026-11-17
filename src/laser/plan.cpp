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

std::vector<LaserPath> join_paths(const std::vector<LaserPath>& paths, double tol, PlanReport* report,
                                  std::map<int, int>* aliases) {
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
                if (aliases && open[i]->id >= 0 && open[s]->id >= 0) (*aliases)[open[i]->id] = open[s]->id;
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
        p.id = open[s]->id;
        p.source = open[s]->source;
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

// A start and direction the designer fixed for one path.
struct Forced {
    bool has{false};
    std::size_t vertex{0};
    bool reverse{false};
};

// What the cost of an order needs to know beyond the paths: which paths are
// parts whose interior is free once they are cut, and the starts a designer
// fixed. An empty `part` means crossings are not looked at.
struct Model {
    std::vector<char> part;
    std::vector<Bounds> box;
    std::vector<Forced> forced;
    double weight{0.0};
};

StartChoice choose_start(const LaserPath& p, const Pt& head, const PlanOptions& opts, const Forced* forced) {
    StartChoice c;
    if (forced && forced->has) {
        c.reverse = forced->reverse;
        if (p.closed) {
            c.vertex = forced->vertex % p.pts.size();
            c.distance = dist(head, p.pts[c.vertex]);
        } else {
            c.distance = dist(head, c.reverse ? p.pts.back() : p.pts.front());
        }
        return c;
    }
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

const Forced* forced_of(const Model& m, std::size_t i) { return i < m.forced.size() ? &m.forced[i] : nullptr; }

// A closed path starts at its chosen vertex; reversed, it goes round the other
// way from there.
LaserPath apply_start(const LaserPath& p, const StartChoice& c) {
    LaserPath out = p;
    if (p.closed) {
        std::rotate(out.pts.begin(), out.pts.begin() + static_cast<std::ptrdiff_t>(c.vertex), out.pts.end());
        if (c.reverse) std::reverse(out.pts.begin() + 1, out.pts.end());
    } else if (c.reverse) {
        std::reverse(out.pts.begin(), out.pts.end());
    }
    return out;
}

Pt end_after(const LaserPath& p, const StartChoice& c) {
    if (p.closed) return p.pts[c.vertex];
    return c.reverse ? p.pts.front() : p.pts.back();
}

Pt start_of(const LaserPath& p, const StartChoice& c) {
    if (p.closed) return p.pts[c.vertex];
    return c.reverse ? p.pts.back() : p.pts.front();
}

double cross2(double ax, double ay, double bx, double by) { return ax * by - ay * bx; }

// The length of segment a-b that lies inside the closed polygon `poly`. The
// segment is cut at every edge it crosses and each piece is classed by its
// midpoint, which is robust where a plain "is it inside" test is not.
double inside_length(const Pt& a, const Pt& b, const LaserPath& poly) {
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double len = std::hypot(dx, dy);
    if (len <= 0.0) return 0.0;
    std::vector<double> ts{0.0, 1.0};
    for (std::size_t i = 0, j = poly.pts.size() - 1; i < poly.pts.size(); j = i++) {
        const Pt& q0 = poly.pts[j];
        const double ex = poly.pts[i].x - q0.x, ey = poly.pts[i].y - q0.y;
        const double den = cross2(dx, dy, ex, ey);
        if (std::fabs(den) < 1e-12) continue;  // parallel: no single crossing
        const double wx = q0.x - a.x, wy = q0.y - a.y;
        const double t = cross2(wx, wy, ex, ey) / den;
        const double u = cross2(wx, wy, dx, dy) / den;
        if (t > 0.0 && t < 1.0 && u >= 0.0 && u <= 1.0) ts.push_back(t);
    }
    std::sort(ts.begin(), ts.end());
    double in = 0.0;
    for (std::size_t k = 1; k < ts.size(); ++k) {
        if (ts[k] - ts[k - 1] <= 1e-12) continue;
        const double m = 0.5 * (ts[k] + ts[k - 1]);
        if (inside({a.x + dx * m, a.y + dy * m}, poly)) in += (ts[k] - ts[k - 1]) * len;
    }
    return in;
}

// Length of the travel a-b that runs inside any of the cut-free parts. `work`
// is charged the vertices actually examined, for the 2-opt budget.
double crossing_length(const Pt& a, const Pt& b, const std::vector<LaserPath>& paths, const Model& m,
                       const std::vector<std::size_t>& freed, std::size_t* work) {
    double sum = 0.0;
    const double x0 = std::min(a.x, b.x), x1 = std::max(a.x, b.x);
    const double y0 = std::min(a.y, b.y), y1 = std::max(a.y, b.y);
    for (std::size_t j : freed) {
        const Bounds& bb = m.box[j];
        if (x1 < bb.x0 || x0 > bb.x1 || y1 < bb.y0 || y0 > bb.y1) continue;
        if (work) *work += paths[j].pts.size();
        sum += inside_length(a, b, paths[j]);
    }
    return sum;
}

// An order's travel, from `head`, with the starts the cut would choose, and the
// length of that travel that runs across parts already cut free.
struct Eval {
    double travel{0.0};
    double crossing{0.0};
    double cost(const Model& m) const { return travel + m.weight * crossing; }
};

Eval evaluate(const std::vector<LaserPath>& paths, const std::vector<std::size_t>& order, Pt head,
              const PlanOptions& opts, const Model& m, std::size_t* work) {
    Eval e;
    std::vector<std::size_t> freed;
    for (std::size_t i : order) {
        const StartChoice c = choose_start(paths[i], head, opts, forced_of(m, i));
        const Pt s = start_of(paths[i], c);
        e.travel += c.distance;
        if (m.weight > 0.0 && !m.part.empty()) {
            e.crossing += crossing_length(head, s, paths, m, freed, work);
            if (m.part[i]) freed.push_back(i);
        }
        head = end_after(paths[i], c);
        if (work) *work += 2 * paths[i].pts.size();
    }
    return e;
}

// Beyond these a layer keeps its nearest-neighbour order. Each 2-opt trial
// re-chooses every start after the reversal (a start depends on the end
// before it), so a pass is O(n^2) trials of O(n) paths each; 200 paths and a
// vertex budget keep that to a second or so however the vertices are spread,
// and a job of thousands of paths is not hung by it.
constexpr std::size_t kTwoOptMaxPaths = 200;
constexpr std::size_t kTwoOptMaxPasses = 10;
constexpr std::size_t kTwoOptVertexBudget = 50'000'000;
// "Strictly less" in the face of float noise from re-choosing starts.
constexpr double kTwoOptGain = 1e-9;

std::vector<std::size_t> flatten(const std::vector<std::vector<std::size_t>>& units) {
    std::vector<std::size_t> out;
    for (const auto& u : units) out.insert(out.end(), u.begin(), u.end());
    return out;
}

// 2-opt over units, each a fixed run of paths: a reversal reorders the units
// and leaves what is inside each alone. A unit is a single path (the plain
// case) or a whole region. `contains` constrains the flattened order, as in
// two_opt_order.
std::vector<std::vector<std::size_t>> two_opt_units(const std::vector<LaserPath>& paths,
                                                    std::vector<std::vector<std::size_t>> units, const Pt& head,
                                                    const PlanOptions& opts, const Model& m,
                                                    const std::vector<std::vector<std::size_t>>& contains) {
    const std::size_t n = units.size();
    if (n < 3 || n > kTwoOptMaxPaths) return units;

    std::vector<std::size_t> pos(paths.size(), 0);
    auto respects = [&](const std::vector<std::size_t>& o) {
        for (std::size_t k = 0; k < o.size(); ++k) pos[o[k]] = k;
        for (std::size_t outer = 0; outer < contains.size(); ++outer) {
            for (std::size_t in : contains[outer]) {
                if (pos[in] > pos[outer]) return false;
            }
        }
        return true;
    };

    std::size_t work = 0;
    double best = evaluate(paths, flatten(units), head, opts, m, &work).cost(m);
    std::vector<std::vector<std::size_t>> trial;
    for (std::size_t pass = 0; pass < kTwoOptMaxPasses; ++pass) {
        bool improved = false;
        for (std::size_t i = 0; i + 1 < n; ++i) {
            for (std::size_t j = i + 1; j < n; ++j) {
                if (work > kTwoOptVertexBudget) return units;
                trial = units;
                std::reverse(trial.begin() + static_cast<std::ptrdiff_t>(i),
                             trial.begin() + static_cast<std::ptrdiff_t>(j) + 1);
                const std::vector<std::size_t> flat = flatten(trial);
                if (!respects(flat)) continue;
                const double t = evaluate(paths, flat, head, opts, m, &work).cost(m);
                if (t < best - kTwoOptGain) {
                    best = t;
                    units = trial;
                    improved = true;
                }
            }
        }
        if (!improved) break;
    }
    return units;
}

}  // namespace

double order_travel(const std::vector<LaserPath>& paths, const std::vector<std::size_t>& order, const Pt& head,
                    const PlanOptions& opts) {
    return evaluate(paths, order, head, opts, Model{}, nullptr).travel;
}

std::vector<std::size_t> two_opt_order(const std::vector<LaserPath>& paths, std::vector<std::size_t> order,
                                       const Pt& head, const PlanOptions& opts,
                                       const std::vector<std::vector<std::size_t>>& contains) {
    std::vector<std::vector<std::size_t>> units;
    units.reserve(order.size());
    for (std::size_t i : order) units.push_back({i});
    return flatten(two_opt_units(paths, std::move(units), head, opts, Model{}, contains));
}

double segment_time_s(double length_mm, double speed_mm_s, double accel_mm_s2) {
    if (!(length_mm > 0.0) || !(speed_mm_s > 0.0)) return 0.0;
    if (!(accel_mm_s2 > 0.0)) return length_mm / speed_mm_s;
    // Distance to get up to speed and back down again.
    const double ramps = speed_mm_s * speed_mm_s / accel_mm_s2;
    if (length_mm >= ramps) return length_mm / speed_mm_s + speed_mm_s / accel_mm_s2;
    return 2.0 * std::sqrt(length_mm / accel_mm_s2);  // triangle: never reaches speed
}

namespace {

std::string path_name(const LaserPath& p, std::size_t index) {
    return p.id >= 0 ? "path " + std::to_string(p.id) : "unnumbered path " + std::to_string(index + 1);
}

}  // namespace

std::vector<LaserPath> order_paths(const std::vector<LaserPath>& paths, Pt& head, const PlanOptions& opts,
                                   PlanReport* report) {
    const std::size_t n = paths.size();
    const auto contains = containment(paths);
    std::vector<std::vector<std::size_t>> containers(n);
    for (std::size_t o = 0; o < n; ++o) {
        for (std::size_t in : contains[o]) containers[in].push_back(o);
    }

    // The designer's fixed paths. `chain` is the ones whose place in the order
    // is fixed, in that order; a step may also fix only a start.
    Model model;
    model.forced.assign(n, Forced{});
    std::vector<char> pinned(n, 0);
    std::vector<std::size_t> chain;
    if (!opts.overrides.empty()) {
        std::map<int, std::size_t> by_id;
        for (std::size_t i = 0; i < n; ++i) {
            if (paths[i].id >= 0) by_id.emplace(paths[i].id, i);
        }
        for (const FixedStep& s : opts.overrides.steps) {
            const auto it = by_id.find(s.id);
            if (it == by_id.end()) continue;
            const std::size_t i = it->second;
            if (model.forced[i].has || pinned[i]) continue;  // first mention wins
            model.forced[i].has = s.start.has_value() || s.reverse;
            model.forced[i].vertex = s.start.value_or(0);
            model.forced[i].reverse = s.reverse;
            if (!s.free_order) {
                pinned[i] = 1;
                chain.push_back(i);
            }
        }
    }

    // Parts: closed paths nested an even number of levels deep. Once cut, one
    // is free, and the head crossing it can snag a tipped-up part. A hole
    // (odd depth) is scrap and is not looked after.
    // (The parts are found even at weight 0, so the report can still say what
    // the order crosses.)
    {
        model.weight = std::max(opts.cutfree_weight, 0.0);
        model.part.assign(n, 0);
        model.box.resize(n);
        for (std::size_t i = 0; i < n; ++i) {
            model.box[i] = bounds_of(paths[i]);
            model.part[i] = (paths[i].closed && paths[i].pts.size() >= 3 && containers[i].size() % 2 == 0) ? 1 : 0;
        }
    }

    // Regions: the outermost container of each path, else the path itself.
    std::vector<double> areas(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        if (paths[i].closed && paths[i].pts.size() >= 3) areas[i] = area(paths[i]);
    }
    std::vector<std::size_t> region(n);
    for (std::size_t i = 0; i < n; ++i) {
        region[i] = i;
        double best = -1.0;
        for (std::size_t o : containers[i]) {
            if (areas[o] > best) {
                best = areas[o];
                region[i] = o;
            }
        }
    }
    if (report) {
        std::size_t roots = 0;
        for (std::size_t i = 0; i < n; ++i) roots += region[i] == i ? 1 : 0;
        report->regions += roots;
    }

    // waiting[i]: how many paths inside i are still to be cut. Between two
    // fixed paths the designer's order stands, so those do not wait.
    std::vector<std::size_t> waiting(n, 0);
    auto counted = [&](std::size_t outer, std::size_t in) { return !(pinned[outer] && pinned[in]); };
    for (std::size_t o = 0; o < n; ++o) {
        for (std::size_t in : contains[o]) {
            if (counted(o, in)) ++waiting[o];
        }
    }

    // Nearest neighbour, as an order of indices. Cost is travel plus the
    // weighted length that runs across parts already cut free; with by_region
    // a region that still has a path ready is finished first.
    std::vector<bool> done(n, false);
    std::vector<std::size_t> order;
    std::vector<std::size_t> freed;
    order.reserve(n);
    Pt at = head;
    std::size_t next_fixed = 0;
    std::size_t current_region = SIZE_MAX;
    for (std::size_t step = 0; step < n; ++step) {
        auto ready = [&](std::size_t i) {
            if (done[i] || waiting[i] != 0) return false;
            return !pinned[i] || chain[next_fixed] == i;
        };
        std::size_t best = SIZE_MAX;
        StartChoice best_c;
        double best_cost = 0.0;
        for (int sticky = opts.by_region && current_region != SIZE_MAX ? 1 : 0; sticky >= 0 && best == SIZE_MAX;
             --sticky) {
            for (std::size_t i = 0; i < n; ++i) {
                if (!ready(i)) continue;
                if (sticky && region[i] != current_region) continue;
                const StartChoice c = choose_start(paths[i], at, opts, forced_of(model, i));
                double cost = c.distance;
                if (model.weight > 0.0) {
                    cost += model.weight *
                            crossing_length(at, start_of(paths[i], c), paths, model, freed, nullptr);
                }
                if (best == SIZE_MAX || cost < best_cost) {
                    best = i;
                    best_c = c;
                    best_cost = cost;
                }
            }
        }
        if (best == SIZE_MAX) {
            // Nothing is ready: a fixed order has made the rest wait on each
            // other. Honour the fixed order first, then drawing order, and
            // let the warning below say what was broken.
            if (next_fixed < chain.size()) {
                best = chain[next_fixed];
            } else {
                for (std::size_t i = 0; i < n; ++i) {
                    if (!done[i]) {
                        best = i;
                        break;
                    }
                }
            }
            best_c = choose_start(paths[best], at, opts, forced_of(model, best));
        }
        done[best] = true;
        if (pinned[best] && next_fixed < chain.size() && chain[next_fixed] == best) ++next_fixed;
        for (std::size_t o : containers[best]) {
            if (counted(o, best) && waiting[o] > 0) --waiting[o];
        }
        if (!model.part.empty() && model.part[best]) freed.push_back(best);
        order.push_back(best);
        current_region = region[best];
        at = end_after(paths[best], best_c);
    }

    // 2-opt reorders regions when they are the unit, paths otherwise. Fixed
    // steps are a precedence between paths that a reversal could break, and
    // checking that for every trial is not worth it: with any fixed order in
    // the layer the nearest-neighbour order stands.
    if (opts.two_opt && chain.empty()) {
        std::vector<std::vector<std::size_t>> units;
        if (opts.by_region) {
            for (std::size_t i : order) {
                if (!units.empty() && region[units.back().front()] == region[i]) {
                    units.back().push_back(i);
                } else {
                    units.push_back({i});
                }
            }
            // Contiguous by construction, so no constraint is needed.
            units = two_opt_units(paths, std::move(units), head, opts, model, {});
        } else {
            for (std::size_t i : order) units.push_back({i});
            units = two_opt_units(paths, std::move(units), head, opts, model, contains);
        }
        order = flatten(units);
    }

    // Starts come from the final order, so a reversed stretch cuts each path
    // from the end it now meets first.
    std::vector<LaserPath> out;
    out.reserve(n);
    freed.clear();
    for (std::size_t i : order) {
        const StartChoice c = choose_start(paths[i], head, opts, forced_of(model, i));
        LaserPath p = apply_start(paths[i], c);
        if (report) {
            report->travel_mm += dist(head, p.pts.front());
            report->cut_mm += length(p);
            if (!model.part.empty()) {
                report->crossing_mm += crossing_length(head, p.pts.front(), paths, model, freed, nullptr);
            }
        }
        if (!model.part.empty() && model.part[i]) freed.push_back(i);
        head = p.closed ? p.pts.front() : p.pts.back();
        out.push_back(std::move(p));
    }

    // The one thing that is always wrong to do quietly: a part dropped out
    // before what is inside it is cut shifts everything cut after it.
    if (report && !opts.overrides.empty()) {
        std::vector<std::size_t> pos(n, 0);
        for (std::size_t k = 0; k < n; ++k) pos[order[k]] = k;
        constexpr std::size_t kMaxWarnings = 10;
        std::size_t extra = 0;
        for (std::size_t o = 0; o < n; ++o) {
            for (std::size_t in : contains[o]) {
                if (pos[in] < pos[o]) continue;
                if (report->warnings.size() >= kMaxWarnings) {
                    ++extra;
                    continue;
                }
                report->warnings.push_back(path_name(paths[o], o) + " is cut before " + path_name(paths[in], in) +
                                           ", which is inside it; the part may drop out first");
            }
        }
        if (extra) report->warnings.push_back("and " + std::to_string(extra) + " more cut-before-inside orders");
    }
    return out;
}

namespace {

// Step 8 for one layer's ordered paths, the head starting at `head`.
//
// Simplification: every vertex is a full stop, so each segment is its own
// trapezoid. A real controller blends corners and runs faster; corner-speed
// blending is a later refinement, and the figure errs long until then. Each
// pass over a path costs its cut time again; travel between passes is nil.
void add_time(const std::vector<LaserPath>& ordered, Pt head, const LaserLayer& layer, const PlanOptions& opts,
              PlanReport& report) {
    for (const LaserPath& p : ordered) {
        report.travel_s += segment_time_s(dist(head, p.pts.front()), opts.travel_speed_mm_s, opts.accel_mm_s2);
        double cut = 0.0;
        for (std::size_t i = 1; i < p.pts.size(); ++i) {
            cut += segment_time_s(dist(p.pts[i - 1], p.pts[i]), layer.speed, opts.accel_mm_s2);
        }
        if (p.closed && p.pts.size() > 1) {
            cut += segment_time_s(dist(p.pts.back(), p.pts.front()), layer.speed, opts.accel_mm_s2);
        }
        report.cut_s += cut * static_cast<double>(std::max(layer.passes, 1));
        head = p.closed ? p.pts.front() : p.pts.back();
    }
    report.estimated_s = report.cut_s + report.travel_s;
}

}  // namespace

bool plan_job(const LaserJob& raw, LaserJob& planned, const PlanOptions& opts, PlanReport& report,
              std::string& error) {
    report = PlanReport{};
    report.paths_in = raw.paths.size();
    if (!raw.rasters.empty()) {
        error = "raster encoding not implemented yet (the job has " + std::to_string(raw.rasters.size()) +
                " raster record(s))";
        return false;
    }

    std::vector<const LaserLayer*> layers;
    for (const LaserLayer& l : raw.layers) layers.push_back(&l);
    std::stable_sort(layers.begin(), layers.end(), [](const LaserLayer* a, const LaserLayer* b) {
        return a->order != b->order ? a->order < b->order : a->pen < b->pen;
    });

    for (const FixedStep& s : opts.overrides.steps) {
        bool found = false;
        for (const LaserPath& p : raw.paths) found = found || (p.id >= 0 && p.id == s.id);
        if (!found) report.warnings.push_back("an override names path " + std::to_string(s.id) +
                                              ", which is not in the job");
    }

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
        std::map<int, int> aliases;
        const std::vector<LaserPath> joined = join_paths(mine, opts.join_tol, &report, &aliases);
        // An override names a path by an id from the raw job; a path joined
        // into a chain is now the chain's. Its start vertex meant a vertex of
        // the piece, which the chain no longer has, so it is dropped.
        PlanOptions layer_opts = opts;
        for (FixedStep& s : layer_opts.overrides.steps) {
            const auto it = aliases.find(s.id);
            if (it == aliases.end()) continue;
            s.id = it->second;
            s.start.reset();
        }
        const Pt start = head;
        std::vector<LaserPath> ordered = order_paths(joined, head, layer_opts, &report);
        add_time(ordered, start, *l, opts, report);
        for (LaserPath& p : ordered) out.paths.push_back(std::move(p));
    }
    report.paths_out = out.paths.size();
    planned = std::move(out);
    return true;
}

}  // namespace ncad::laser
