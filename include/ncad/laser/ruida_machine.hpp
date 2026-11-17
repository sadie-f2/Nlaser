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

// Where the controller places a job: "Ref Point 0" puts the job's origin at the
// head's current position; "Ref Point 2" + "Set Absolute" use machine
// coordinates (LightBurn's Start From = Absolute Coords). "Ref Point 1" (the
// anchor / user origin set on the panel) is LightBurn's Start From = User
// Origin (f04-user): the same coordinates as Current, only the opening differs.
enum class RdStart { Current, Absolute, User };

inline const char* start_name(RdStart s) {
    return s == RdStart::Absolute ? "absolute" : s == RdStart::User ? "user" : "current";
}

enum class MachineCorner { BottomLeft, BottomRight, TopLeft, TopRight };

struct RdMachine {
    MachineCorner origin{MachineCorner::BottomLeft};  // bottom-left: the identity
    // Extra mirrors for a machine whose motors are wired the other way round;
    // DeviceConfig's flip_x / flip_y.
    bool flip_x{false};
    bool flip_y{false};
    // The bed, in mm, as the machine counts it (0 = not known). Needed only to
    // turn a BED coordinate into a MACHINE coordinate on a machine that counts
    // from its right or top edge; see bed_to_machine().
    double bed_w_mm{0.0};
    double bed_h_mm{0.0};

    // +1 or -1: the sign of a design displacement as the file counts it.
    int sign_x() const {
        const bool right = origin == MachineCorner::BottomRight || origin == MachineCorner::TopRight;
        return right != flip_x ? -1 : 1;
    }
    int sign_y() const {
        const bool top = origin == MachineCorner::TopLeft || origin == MachineCorner::TopRight;
        return top != flip_y ? -1 : 1;
    }

    bool has_bed() const { return bed_w_mm > 0.0 && bed_h_mm > 0.0; }
    // True when either axis counts from the far side of the bed.
    bool mirrored() const { return sign_x() < 0 || sign_y() < 0; }

    // Bed coordinates -- (0,0) at the bed's bottom-left corner, X right, Y up,
    // mm -- to machine coordinates, the distances from machine zero that an
    // absolute move or job carries. An axis that counts from the far side
    // is W - x (or H - y). False when such an axis is present and the bed size
    // is not known; the outputs are then untouched.
    bool bed_to_machine(double x, double y, double& mx, double& my) const {
        const bool nx = sign_x() < 0, ny = sign_y() < 0;
        if ((nx || ny) && !has_bed()) return false;
        mx = nx ? bed_w_mm - x : x;
        my = ny ? bed_h_mm - y : y;
        return true;
    }
    // The inverse (the same arithmetic: W - (W - x) = x).
    bool machine_to_bed(double mx, double my, double& x, double& y) const { return bed_to_machine(mx, my, x, y); }
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
