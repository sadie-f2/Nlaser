// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// The bridge from a NotoCAD drawing to a raw LaserJob: the spec's
// LaserRenderer. The only header in this tree that includes NotoCAD, and it
// belongs to its own target, ncad_laser_ncad, so ncad_laser_core never sees a
// drawing and the dependency shows in the build graph.
//
// It is a Renderer like PLOT's PdfRenderer, fed by the same draw_database()
// walk, so what cuts is what NotoCAD draws: TEXT as its Hershey strokes,
// SPLINE, ELLIPSE and block INSERTs with no switch on entity type. Each
// emitted polyline becomes one LaserPath; the pen is the resolved ACI colour.
//
// What the Renderer interface does not carry, and how it is made up here:
//
//  - Which entity begin_entity() is for comes from NotoCAD now: it passes the
//    database handle of the entity (the INSERT's, for its contents). Only
//    one address comparison remains, to tell an INSERT's own begin_entity()
//    from the calls for its contents, which carry the same handle. Where the
//    handle is not in the database the renderer falls back and counts it
//    rather than guessing silently.
//  - BYBLOCK colour and layer-0 inheritance inside an INSERT. NotoCAD passes
//    the inner entity's props through unchanged and never calls begin_entity()
//    for a nested INSERT, so the renderer replays the expansion itself, in the
//    same order, to know each entity's enclosing inserts.
//  - Visibility inside a block. draw_database() skips a top-level entity on an
//    off or frozen layer and draws everything inside a visible INSERT; R12
//    hides block contents on off or frozen layers too, and a laser must not
//    cut what the drawing hides, so the renderer drops those itself.
//  - Polyline width arrives through Renderer::polyline_width(); wide
//    polylines are counted so the caller can warn (raster is not supported
//    yet).
//
// Drawing units are taken to be millimetres: a laser plots at 1:1.
#pragma once

#include "ncad/entity.hpp"
#include "ncad/laser/job.hpp"
#include "ncad/plot.hpp"
#include "ncad/render.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ncad {
class Database;
struct BlockDef;
}  // namespace ncad

namespace ncad::laser {

// LASERTOL's default: ten times finer than a typical kerf.
inline constexpr double kLaserTol = 0.01;  // mm

// The spec's vector/raster threshold.
inline constexpr double kRasterWidth = 0.5;  // mm

struct DrawingOptions {
    double chord_tolerance{kLaserTol};
    // The job origin. Absent, it is the LASER UCS's origin, or 0,0.
    bool has_origin{false};
    Pt origin{};
    // Only what a view can see, as PLOT's areas give it. Whole entities: one
    // whose box touches the view is cut entire, never a fragment of it, so a
    // window never leaves half a part. Absent, everything is drawn.
    bool has_view{false};
    PlotView view{};
};

struct DrawingReport {
    std::size_t entities{0};         // top-level entities drawn
    std::size_t paths{0};
    std::size_t hidden{0};           // top-level, on off or frozen layers
    std::size_t hidden_in_blocks{0}; // block contents on off or frozen layers
    std::size_t not_flat{0};         // normal not parallel to Z: cut as projected
    std::size_t wide_polylines{0};   // width >= kRasterWidth: cut on the centre line
    std::size_t solids{0};           // SOLID: cut as an outline
    std::size_t proxies{0};          // no class here, so nothing to cut
    std::size_t unmatched{0};        // begin_entity() calls not in the database
    std::size_t degenerate{0};       // polylines of fewer than two points
    bool laser_ucs{false};
    bool laser_ucs_rotated{false};
    std::vector<int> unset_pens;     // pens the pen table left at power 0
    std::vector<std::string> warnings;
};

class LaserRenderer final : public Renderer {
public:
    // Paths are appended to `job.paths`; counts go to `report`. Both must
    // outlive the renderer, as must `db`.
    LaserRenderer(const Database& db, LaserJob& job, DrawingReport& report);

    void begin_entity(const EntityProps& props, Handle source) override;
    void polyline_width(double width) override { pending_width_ = width; }
    void polyline(const Vec3* pts, std::size_t count, bool closed) override;

private:
    // What an entity's props resolve to where it is drawn.
    struct Resolved {
        std::int16_t color{7};
        LayerId layer{kLayerZero};
        bool visible{true};
    };
    // One entity of the INSERT being drawn, in the order NotoCAD draws them.
    struct Nested {
        const EntityProps* props{nullptr};
        Resolved resolved;
    };

    Resolved resolve(const EntityProps& p, const Resolved* enclosing) const;
    void survey(const Entity& e, const Vec3& normal, bool visible);
    void expand(const BlockDef& def, const Mat4& m, const Resolved& enclosing, int depth);

    const Database& db_;
    LaserJob& job_;
    DrawingReport& report_;

    Resolved current_;
    bool in_insert_{false};
    Resolved insert_;  // the top-level INSERT, for the fallback
    std::vector<Nested> nested_;
    std::size_t next_{0};
    double pending_width_{0.0};
};

// The whole drawing into a raw job: paths, origin and the report. Layer
// records are left to apply_pens(). False only for a bad option.
bool render_drawing(const Database& db, const DrawingOptions& opts, LaserJob& job, DrawingReport& report,
                    std::string& error);

// A `.lpt` pen table: `layer` lines in .ljob syntax and nothing else.
bool read_pen_table(const std::string& text, std::vector<LaserLayer>& pens, std::string& error);
bool read_pen_table_file(const std::string& path, std::vector<LaserLayer>& pens, std::string& error);

// Gives every pen the job uses a layer record, from `pens` where listed.
// A pen not listed gets power 0,0 and mode=auto -- a job that runs and marks
// nothing is the safe default -- and is added to `report.unset_pens`, with a
// warning.
void apply_pens(LaserJob& job, const std::vector<LaserLayer>& pens, DrawingReport& report);

// dxf2ljob: read a DXF, render it, apply the pen table. The job is raw.
bool dxf_to_job(const std::string& dxf_path, const DrawingOptions& opts, const std::vector<LaserLayer>& pens,
                LaserJob& job, DrawingReport& report, std::string& error);

}  // namespace ncad::laser
