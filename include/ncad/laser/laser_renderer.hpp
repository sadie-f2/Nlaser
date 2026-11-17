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
// Drawing units are millimetres unless DrawingOptions::units says inches
// (see below): a laser plots at 1:1.
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

// How curves become polylines. Ncad is what NotoCAD's draw() gives every
// other renderer. LightBurn reproduces LightBurn's own flattening of CIRCLE,
// ARC and ELLIPSE (cubic Beziers of at most a quarter turn, each cut into
// equal steps of the Bezier parameter, which spaces the vertices unevenly) so
// that a DXF cuts the same polygon LightBurn would have made of it.
//
// It cannot be had from the polylines draw() delivers: those are uniform in
// angle, and a circle's start vertex and direction are not recoverable from
// them. So in this mode the renderer re-flattens those three entity types
// from the entity itself, found by the handle begin_entity() carries, and
// ignores the polyline draw() then emits for it. Everything else (SPLINE,
// TEXT, polylines, block contents) is as in Ncad mode.
enum class FlattenMode { Ncad, LightBurn };

// The polygon LightBurn makes of the elliptical arc
//     P(a) = c + ax*cos(a) + ay*sin(a),  a from a0 through `sweep` (signed)
// within `tol` mm of the true curve. ax and ay are the conjugate semi-axes
// (for a circle (r,0) and (0,r)). Closed, with no repeated end point, when
// |sweep| is a full turn. Measured on f07 for circles only; arcs and
// ellipses follow the same construction but are not checked against a file.
std::vector<Pt> lightburn_flatten(const Pt& c, const Pt& ax, const Pt& ay, double a0, double sweep, double tol,
                                  bool& closed);

// What one drawing unit is. NotoCAD main has no drawing units, so a drawing in
// inches would cut 25.4 times too small unless told. Auto is for dxf_to_job()
// only: it reads $INSUNITS; render_drawing() takes Auto as Mm.
enum class DrawingUnits { Auto, Mm, Inch };

inline double mm_per_unit(DrawingUnits u) { return u == DrawingUnits::Inch ? 25.4 : 1.0; }
inline const char* units_name(DrawingUnits u) { return u == DrawingUnits::Inch ? "in" : "mm"; }

struct DrawingOptions {
    // The job is always in mm: coordinates, origin and widths are scaled by
    // mm_per_unit() as it is rendered. chord_tolerance and the raster width
    // are in mm; the view and `origin` are in drawing units, as drawn.
    DrawingUnits units{DrawingUnits::Auto};
    double chord_tolerance{kLaserTol};
    FlattenMode flatten{FlattenMode::Ncad};
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
    DrawingUnits units{DrawingUnits::Mm};  // what the job was scaled from
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

    // FlattenMode::LightBurn for this renderer; `tol` is the chord tolerance.
    void set_lightburn_flatten(double tol) {
        lightburn_ = true;
        tol_ = tol;
    }
    // mm per drawing unit, for the raster-width test; paths are scaled by the caller.
    void set_unit_scale(double s) { scale_ = s; }

private:
    bool flatten_lightburn(const Entity& e);
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
    Handle source_{kNullHandle};  // the entity (an INSERT's own, for its contents) being drawn
    bool lightburn_{false};
    double tol_{kLaserTol};
    double scale_{1.0};
    bool suppress_{false};  // the entity was flattened here; drop draw()'s polyline
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
