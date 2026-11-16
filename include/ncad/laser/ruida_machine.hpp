// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// Where the machine's origin is, and so which way its axes run.
//
// A job is planned in design coordinates (X right, Y up). A Ruida machine
// counts from its own origin corner: a top-right machine counts X leftwards and
// Y downwards. LightBurn writes machine coordinates, and so must we; the
// encoder applies this mapping to every coordinate it writes, after planning,
// so a .ljob never depends on the machine (docs/lightburn-fixtures.md, "The
// machine frame", is the evidence).
//
// The mapping is a pure sign change per axis. Where the head is when the job
// starts is the job's origin, not a property of the machine.
#pragma once

#include <string>
#include <string_view>

namespace ncad::laser::ruida {

enum class MachineCorner { BottomLeft, BottomRight, TopLeft, TopRight };

struct RdMachine {
    MachineCorner origin{MachineCorner::BottomLeft};  // bottom-left: the identity
    // Extra mirrors for a machine whose motors are wired the other way round;
    // DeviceConfig's flip_x / flip_y.
    bool flip_x{false};
    bool flip_y{false};

    // +1 or -1: the sign of a design displacement as the file counts it.
    int sign_x() const {
        const bool right = origin == MachineCorner::BottomRight || origin == MachineCorner::TopRight;
        return right != flip_x ? -1 : 1;
    }
    int sign_y() const {
        const bool top = origin == MachineCorner::TopLeft || origin == MachineCorner::TopRight;
        return top != flip_y ? -1 : 1;
    }
};

// "bl" "br" "tl" "tr"; false for anything else.
inline bool parse_machine_corner(std::string_view s, MachineCorner& out) {
    if (s == "bl") out = MachineCorner::BottomLeft;
    else if (s == "br") out = MachineCorner::BottomRight;
    else if (s == "tl") out = MachineCorner::TopLeft;
    else if (s == "tr") out = MachineCorner::TopRight;
    else return false;
    return true;
}

}  // namespace ncad::laser::ruida
