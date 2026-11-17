// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

#include "ncad/laser/move.hpp"

#include "ncad/laser/ruida_machine.hpp"
#include "ncad/laser/ruida_transport.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace ncad::laser {

bool plan_move(const DeviceConfig& cfg, const MachineStatus& head, MoveFrame frame, double x, double y,
               MoveReport& report, std::string& error) {
    report = MoveReport{};
    if (!std::isfinite(x) || !std::isfinite(y)) {
        error = "the target is not a number";
        return false;
    }
    ruida::RdMachine m = ruida::machine_of(cfg);
    if (!m.has_bed() && head.has_bed) {
        m.bed_w_mm = head.bed_w_mm;
        m.bed_h_mm = head.bed_h_mm;
    }
    double mx = x, my = y;
    switch (frame) {
        case MoveFrame::Machine: break;
        case MoveFrame::Bed:
            if (!m.bed_to_machine(x, y, mx, my)) {
                error = "this machine counts from its right or top edge, so a bed coordinate needs the bed size to "
                        "become a machine coordinate, and it is not known -- set 'bed W H' in lasers.conf, give "
                        "--bed, or use --machine";
                return false;
            }
            break;
        case MoveFrame::Offset:
            if (!head.has_position) {
                error = "the head position could not be read, so a move relative to it cannot be made; use an "
                        "absolute goto";
                return false;
            }
            mx = head.x_mm + m.sign_x() * x;
            my = head.y_mm + m.sign_y() * y;
            break;
    }
    // Both the bed edges and the origin side: a head sent past machine zero
    // or past the far limit drives into the frame.
    constexpr double kEps = 0.0005;
    char buf[320];
    const bool low = mx < -kEps || my < -kEps;
    const bool high = m.has_bed() && (mx > m.bed_w_mm + kEps || my > m.bed_h_mm + kEps);
    if (low || high) {
        std::snprintf(buf, sizeof buf, "the target leaves the bed: machine %.3f, %.3f mm%s", mx, my,
                      m.has_bed() ? "" : " (below machine zero)");
        error = buf;
        if (m.has_bed()) {
            std::snprintf(buf, sizeof buf, "; the bed is 0..%.3f by 0..%.3f", m.bed_w_mm, m.bed_h_mm);
            error += buf;
        }
        return false;
    }
    if (!m.has_bed()) report.note = "the bed size is not known: only the home-corner side was checked";
    report.machine_x = mx;
    report.machine_y = my;
    report.has_bed_pos = m.machine_to_bed(mx, my, report.bed_x, report.bed_y);
    return true;
}

bool parse_jog(const std::vector<std::string>& args, double step_mm, double& dx, double& dy, std::string& error) {
    dx = dy = 0.0;
    if (args.empty() || args.size() > 2) {
        error = "jog takes a direction (+x -x +y -y, one per axis) or two distances DX DY in mm";
        return false;
    }
    auto number = [](const std::string& s, double& v) {
        char* end = nullptr;
        v = std::strtod(s.c_str(), &end);
        return !s.empty() && end && *end == '\0' && std::isfinite(v);
    };
    double a = 0.0, b = 0.0;
    if (args.size() == 2 && number(args[0], a) && number(args[1], b)) {
        if (step_mm != 0.0) {
            error = "--fine, --coarse and --step apply to directions; with explicit distances give the mm";
            return false;
        }
        dx = a;
        dy = b;
        return true;
    }
    if (step_mm < 0.0) {
        error = "the step must be positive";
        return false;
    }
    if (step_mm == 0.0) step_mm = kJogNormalMm;
    bool seen_x = false, seen_y = false;
    for (const std::string& t : args) {
        std::string d = t;
        double sign = 1.0;
        if (!d.empty() && (d[0] == '+' || d[0] == '-')) {
            sign = d[0] == '-' ? -1.0 : 1.0;
            d.erase(0, 1);
        }
        if (d == "x" || d == "X") {
            if (seen_x) {
                error = "two directions on the X axis";
                return false;
            }
            seen_x = true;
            dx = sign * step_mm;
        } else if (d == "y" || d == "Y") {
            if (seen_y) {
                error = "two directions on the Y axis";
                return false;
            }
            seen_y = true;
            dy = sign * step_mm;
        } else {
            error = "'" + t + "' is neither a direction (+x -x +y -y) nor a distance";
            return false;
        }
    }
    return true;
}

}  // namespace ncad::laser
