// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// SVGIN: import an SVG into the drawing as real entities.
//
//   SVG file to import: art.svg
//
// Cubic Beziers become SPLINEs (degree 3, clamped, a chain of cubics with
// triple interior knots, so the curve is exact), straight runs become
// POLYLINEs, with the SVG stroke as polyline width when it is 0.5 mm or more.
// Each named SVG group gets a layer of the same name, coloured as its first
// shape is. Units come from the SVG and from SVGPXIN (pixels per inch for an
// SVG with no real units, default 96; Illustrator writes 72). Fills are
// outline-only, with a warning: filled regions are raster.
//
// One undoable step, because the command engine opens an undo group around
// every command; the import adds entities and layers through the Database like
// any other command and needs nothing of its own.
#pragma once

#include "ncad/command.hpp"
#include "ncad/laser/laser_command.hpp"
#include "ncad/laser/svg_import.hpp"

#include <memory>
#include <string>

namespace ncad::laser {

// A layer name DXF accepts: its forbidden characters become '_', empty becomes "0".
std::string dxf_layer_name(const std::string& raw);

struct SvgInReport {
    std::size_t polylines{0};
    std::size_t splines{0};
    std::size_t layers_created{0};
};

// Adds the shapes to `db`, in world coordinates on the XY plane.
SvgInReport add_svg_to_drawing(Database& db, const SvgImport& svg);

class SvgInCommand final : public Command {
public:
    explicit SvgInCommand(std::shared_ptr<LaserSession> session) : session_(std::move(session)) {}

    const char* name() const override { return "SVGIN"; }
    Step start(CommandContext& ctx) override;
    Step next(CommandContext& ctx, const InputValue& value) override;

private:
    std::shared_ptr<LaserSession> session_;
};

}  // namespace ncad::laser
