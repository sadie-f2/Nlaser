// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// LBRN2IN: import a LightBurn project (.lbrn2) into the drawing as entities.
//
//   LightBurn project to import: sign.lbrn2
//
// Paths become POLYLINEs (closed or open); an Ellipse shape becomes a CIRCLE
// when its axes are equal and an ELLIPSE otherwise. Each CutSetting becomes a
// layer named after the LightBurn layer ("LB<index>" when it has no name),
// coloured with the ACI nearest LightBurn's colour for that layer index
// (lbrn2_assign_aci; the palette is partly recalled, see docs/lbrn2-format.md).
//
// Coordinates: LightBurn's workspace is machine coordinates. LBRN2CORNER (bl,
// br, tl, tr; default tr) says where the machine counts from; the chosen
// device's bed size, when there is one, makes the result bed coordinates
// (X right, Y up from the bed's bottom-left), otherwise the axes are only
// negated and positions mean something relative to each other.
//
// The CutSettings are also kept as the LASER session's pen table (keyed by the
// ACI each layer got), so `LASER Job` then `Send` cuts with LightBurn's speeds
// and powers. It is held in the session rather than written to an .lpt next to
// the drawing: the drawing may have no name, a stray file would outlive the
// import, and the session is where the planned job lives too. Pens Save still
// makes a file when one is wanted, and Pens Load replaces the imported table.
//
// One undoable step: the command engine opens an undo group around every command.
#pragma once

#include "ncad/command.hpp"
#include "ncad/laser/laser_command.hpp"

#include <memory>
#include <string>

namespace ncad::laser {

class Lbrn2InCommand final : public Command {
public:
    explicit Lbrn2InCommand(std::shared_ptr<LaserSession> session) : session_(std::move(session)) {}

    const char* name() const override { return "LBRN2IN"; }
    Step start(CommandContext& ctx) override;
    Step next(CommandContext& ctx, const InputValue& value) override;

private:
    std::shared_ptr<LaserSession> session_;
};

}  // namespace ncad::laser
