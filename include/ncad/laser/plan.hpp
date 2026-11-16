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
//                  contains it.
//   7. Starts   -- a closed path starts at the vertex nearest the head,
//                  preferring a corner over the middle of a run, where the
//                  start mark would show.
//   8. Time     -- trapezoidal motion per segment, see segment_time_s.
//
// Not yet: designer overrides (LASER Order). The nearest-neighbour pass is
// O(n^2) in paths per layer, fine for the thousands of paths a laser job has
// and not for a million.
#pragma once

#include "ncad/laser/job.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace ncad::laser {

struct PlanOptions {
    double join_tol{0.005};     // mm
    // A corner this much farther than the nearest vertex is still preferred.
    double corner_slack{2.0};   // mm
    double corner_angle{30.0};  // degrees of turn that make a vertex a corner
    // Improve the nearest-neighbour order with 2-opt on travel. Off gives the
    // plain nearest-neighbour order, which is what to compare against.
    bool two_opt{true};
    // Placeholders for the time estimate; the real values come from the device
    // config (acceleration, travel speed) once the planner is handed one.
    double accel_mm_s2{3000.0};
    double travel_speed_mm_s{400.0};
};

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
};

// Step 2, for one pen's paths. Closed paths pass through (a repeated closing
// point is dropped); open paths are chained.
std::vector<LaserPath> join_paths(const std::vector<LaserPath>& paths, double tol, PlanReport* report);

// Step 3. contains[i] lists the paths that path i strictly contains; only
// closed paths contain anything. An open path is inside a closed one when its
// first point is.
std::vector<std::vector<std::size_t>> containment(const std::vector<LaserPath>& paths);

// Steps 6 and 7 for one layer's paths, starting with the head at `head`;
// `head` is left where the last path ends.
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
