// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// The planner: a raw job (paths in drawing order, unjoined) into a planned one
// (joined, ordered, with chosen start points). Each step is a pure function,
// tested on its own; `plan_job` runs them in the spec's order.
//
//   1. Flatten  -- done upstream, by draw() at LASERTOL.
//   2. Join     -- chain open paths whose ends meet within `join_tol`, and
//                  close a chain whose ends meet. DXF from other programs
//                  often arrives as separate LINEs; an unjoined rectangle cuts
//                  as four starts and stops.
//   3. Nest     -- containment of closed paths by point-in-polygon.
//   4. No kerf compensation, by design.
//   5. Layers   -- by the layer's `order`, then pen.
//   6. Paths    -- inner before outer, always; among the paths that are free
//                  to go, nearest-neighbour from the head, then 2-opt on
//                  travel, never reordering a path past a closed path that
//                  contains it. Paths are grouped into regions (an outer
//                  contour and everything inside it) and a region is finished
//                  before the next is begun; the regions are what 2-opt
//                  reorders. A travel move across a part already cut free is
//                  charged extra, so orders that avoid them win -- no detours
//                  are routed, the order just prefers not to cross.
//   7. Starts   -- a closed path starts at the vertex nearest the head,
//                  preferring a corner over the middle of a run, where the
//                  start mark would show.
//   8. Time     -- trapezoidal motion per segment, see segment_time_s.
//
// Designer overrides (PlanOverrides) fix some paths' order and starts; the
// rest is optimised around them. The nearest-neighbour pass is
// O(n^2) in paths per layer, fine for the thousands of paths a laser job has
// and not for a million.
#pragma once

#include "ncad/laser/job.hpp"

#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace ncad::laser {

// One path the designer has fixed, named by LaserPath::id.
struct FixedStep {
    int id{-1};
    // Closed path: the vertex (an index into the path as the planner is given
    // it, after joining) it starts at. An open path has only its two ends, so
    // this is ignored there.
    std::optional<std::size_t> start;
    // Cut it the other way round: from `start` backwards round a closed path,
    // from the far end of an open one.
    bool reverse{false};
    // Only the start and direction are fixed; where the path falls in the
    // order is still the optimiser's to choose.
    bool free_order{false};
};

// The fixed steps, in the order they are to be cut. Paths not named are
// optimised around them. Inner-before-outer still holds for everything except
// between two fixed paths: that order is honoured, with a warning in the
// report.
struct PlanOverrides {
    std::vector<FixedStep> steps;
    bool empty() const { return steps.empty(); }
};

struct PlanOptions {
    double join_tol{0.005};     // mm
    // A corner this much farther than the nearest vertex is still preferred.
    double corner_slack{2.0};   // mm
    double corner_angle{30.0};  // degrees of turn that make a vertex a corner
    // Improve the nearest-neighbour order with 2-opt on travel. Off gives the
    // plain nearest-neighbour order, which is what to compare against.
    bool two_opt{true};
    // Finish one region before starting another. Off orders paths as one pool.
    bool by_region{true};
    // How much worse a unit of travel across an already cut-free part is than
    // a unit of travel over clear material (a part that has been cut out may
    // tip up and catch the head). 0 turns the avoidance off.
    double cutfree_weight{5.0};
    PlanOverrides overrides;
    // Where the head is when the job starts, in job coordinates. Absent, it is
    // the job origin. With a pointer offset the beam is not at the origin (the
    // red dot is): LightBurn orders and starts paths from the beam, which in
    // design terms is the origin shifted by the offset, so a caller that wants
    // LightBurn's order says so here.
    bool has_head{false};
    Pt head{};
    // Placeholders for the time estimate; the real values come from the device
    // config (acceleration, travel speed) once the planner is handed one.
    double accel_mm_s2{3000.0};
    double travel_speed_mm_s{400.0};
};

// What `--order lightburn` and LASERORDER=lightburn mean: the plain
// nearest-neighbour order LightBurn makes, so a job can be compared with its
// file. Off is the default order (regions, cut-free avoidance, 2-opt).
inline void use_lightburn_order(PlanOptions& o) {
    o.by_region = false;
    o.cutfree_weight = 0.0;
    o.two_opt = false;
}

struct PlanReport {
    std::size_t paths_in{0};
    std::size_t paths_out{0};
    std::size_t joins{0};           // open paths merged into a neighbour
    std::size_t closed_by_join{0};  // chains whose ends met
    std::size_t skipped{0};         // paths on mode=skip layers
    double cut_mm{0.0};
    double travel_mm{0.0};
    double estimated_s{0.0};  // cut_s + travel_s; reported, not guaranteed
    double cut_s{0.0};
    double travel_s{0.0};
    std::size_t regions{0};
    // Travel that still runs across a part already cut free, after the
    // optimiser did what it could.
    double crossing_mm{0.0};
    std::vector<std::string> warnings;
};

// Step 2, for one pen's paths. Closed paths pass through (a repeated closing
// point is dropped); open paths are chained, the chain taking its first
// member's id. `aliases`, if given, maps every other member's id to the chain's.
std::vector<LaserPath> join_paths(const std::vector<LaserPath>& paths, double tol, PlanReport* report,
                                  std::map<int, int>* aliases = nullptr);

// Step 3. contains[i] lists the paths that path i strictly contains; only
// closed paths contain anything. An open path is inside a closed one when its
// first point is.
std::vector<std::vector<std::size_t>> containment(const std::vector<LaserPath>& paths);

// Steps 6 and 7 for one layer's paths, starting with the head at `head`;
// `head` is left where the last path ends. `opts.overrides` is matched to
// `paths` by id.
std::vector<LaserPath> order_paths(const std::vector<LaserPath>& paths, Pt& head, const PlanOptions& opts,
                                   PlanReport* report);

// Step 6, the 2-opt half. `order` is a permutation of indices into `paths`;
// the result is one with no more travel from `head` (starts re-chosen as the
// cutting order would choose them), every move taken strictly reducing it, in
// which every contains[o] member still precedes o. Pass an empty `contains`
// to see what it would do without the constraint. Bounded: a layer with more
// than 200 paths is returned as given, and too much vertex work stops early,
// so a large job cannot hang.
std::vector<std::size_t> two_opt_order(const std::vector<LaserPath>& paths, std::vector<std::size_t> order,
                                       const Pt& head, const PlanOptions& opts,
                                       const std::vector<std::vector<std::size_t>>& contains);

// Travel distance of cutting `paths` in `order` from `head`: the head to the
// first start, then each end to the next start, starts chosen as in the cut.
double order_travel(const std::vector<LaserPath>& paths, const std::vector<std::size_t>& order, const Pt& head,
                    const PlanOptions& opts);

// Step 8 for one straight segment: time to move `length_mm` from rest to rest
// at up to `speed_mm_s`, accelerating and decelerating at `accel_mm_s2`.
// A segment at least speed^2/accel long reaches speed, cruises, and takes
// L/v + v/a; a shorter one peaks at sqrt(a*L) and takes 2*sqrt(L/a). A
// non-positive acceleration means no ramps (L/v); no length or speed, 0.
double segment_time_s(double length_mm, double speed_mm_s, double accel_mm_s2);

// The whole thing. The head starts at the job origin, which is where a
// current-position job begins. False, with `error`, for a job the planner
// cannot plan (a raster layer, a path on a pen with no layer).
bool plan_job(const LaserJob& raw, LaserJob& planned, const PlanOptions& opts, PlanReport& report,
              std::string& error);

}  // namespace ncad::laser
