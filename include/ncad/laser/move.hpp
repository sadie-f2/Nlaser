// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// Moving the head with the laser off: where to, in which coordinates, and the
// arithmetic of a jog. Pure functions (no I/O), so the bounds and the sums are
// tested without a controller; the transport supplies the head and the bed.
#pragma once

#include "ncad/laser/transport.hpp"

#include <string>
#include <vector>

namespace ncad::laser {

inline constexpr double kJogFineMm = 0.1;
inline constexpr double kJogNormalMm = 1.0;
inline constexpr double kJogCoarseMm = 10.0;

// The head and the bed as far as they are known: `head` supplies the
// position and (when the controller reports it) the bed; `cfg` supplies the
// machine mapping and a configured bed, which wins.
// Resolves a move to machine coordinates and checks it against the bed.
// Refuses (false, words in `error`) when:
//  - the frame is Bed on a machine that counts from its right or top edge and
//    no bed size is known;
//  - the frame is Offset and the head position is unknown;
//  - the target lies outside the bed ("leaves the bed"); with no bed size
//    known only the home-corner side (negative machine coordinates) is checked,
//    and `report.note` says so.
bool plan_move(const DeviceConfig& cfg, const MachineStatus& head, MoveFrame frame, double x, double y,
               MoveReport& report, std::string& error);

// `jog` arguments to a displacement in mm (bed directions). Each argument is
// a direction (+x -x +y -y, also x/y alone for +) scaled by `step_mm` (0 = the normal
// 1 mm step), or two plain numbers (explicit mm; `step_mm` must then be 0). Up to two
// directions, one per axis, may be combined (a diagonal).
bool parse_jog(const std::vector<std::string>& args, double step_mm, double& dx, double& dy, std::string& error);

}  // namespace ncad::laser
