// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// LASER: the laser backend as a NotoCAD command.
//
// A resumable command in the style of PLOT -- every prompt is a state, never a
// blocking read -- so the terminal, a script, the GUI and
// `(command "LASER" ...)` all drive the same code. Nothing here exists only in
// a window.
//
// install_laser() is the whole integration: an explicit call at startup that
// registers the command and its four system variables through NotoCAD's
// extension hooks. NotoCAD's core names nothing laser.
//
//   LASERDEV      device name, a [section] of lasers.conf
//   LASERPENS     pen table file (.lpt)
//   LASERTOL      chord tolerance in mm, default 0.01
//   LASERCONFIRM  1 = Send always asks first (default)
//
// The planned job lives in a LaserSession shared by every LASER invocation, so
// `Job` in one command and `Send` in a later one see the same job. It is
// session state, not drawing state: it is not journalled and not saved.
//
// THREADING. Device operations (Send, Frame, Dryrun, Status, Counters, STop) run on the
// calling thread and block for as long as the transport does. The transport
// bounds every wait by the device's ack_timeout_ms, so a dead controller costs
// a few seconds, never forever -- but a window would freeze for them. The place
// to move them off-thread is LaserCommand::run_device(): it is the one function
// that talks to a controller, and everything before it only decides what to do.
#pragma once

#include "ncad/command.hpp"
#include "ncad/laser/connect.hpp"
#include "ncad/laser/job.hpp"
#include "ncad/laser/laser_renderer.hpp"
#include "ncad/laser/plan.hpp"
#include "ncad/sysvar.hpp"

#include <memory>
#include <string>

namespace ncad::laser {

struct LaserOptions {
    // How a device is opened. Null: connect_ruida when built with the network
    // target, otherwise every device operation refuses with a message.
    LaserConnector connector;
    // lasers.conf. Empty: the default location, which may be absent.
    std::string config_path;
};

struct LaserSession {
    LaserOptions options;

    // Ids of the registered system variables.
    Sysvar dev{};
    Sysvar pens{};
    Sysvar tol{};
    Sysvar confirm{};

    // The last job built by `Job`: planned, with its reports.
    bool has_job{false};
    LaserJob job;
    PlanReport plan;
    DrawingReport drawing;
    std::string area;  // what it was built from, for messages
};

// Registers LASER and its system variables. Returns the shared session, or
// null if a name clashes (a different type already registered under it).
// Idempotent in the sense register_command is: calling it again replaces the
// command's factory with one over the new session.
std::shared_ptr<LaserSession> install_laser(const LaserOptions& options = {});

class LaserCommand final : public Command {
public:
    explicit LaserCommand(std::shared_ptr<LaserSession> session) : session_(std::move(session)) {}

    const char* name() const override { return "LASER"; }
    Step start(CommandContext& ctx) override;
    Step next(CommandContext& ctx, const InputValue& value) override;

private:
    enum class State : std::uint8_t {
        Top,
        DeviceName,
        PensFile,
        JobArea,
        JobWindowFirst,
        JobWindowSecond,
        OrderOption,
        SaveFile,
        SaveOverwrite,
        SendConfirm,
    };
    enum class DeviceOp : std::uint8_t { Send, Frame, Dryrun, Status, Counters, Stop };

    Step top(CommandContext& ctx, const InputValue& value);
    Step ask_top() const;
    Step choose_device(CommandContext& ctx, const InputValue& value);
    Step choose_pens(CommandContext& ctx, const InputValue& value);
    Step build_job(CommandContext& ctx);
    Step order_list() const;
    Step save_job(CommandContext& ctx, const std::string& path);
    Step begin_send(CommandContext& ctx);
    Step run_device(CommandContext& ctx, DeviceOp op);

    bool resolve_device(CommandContext& ctx, DeviceConfig& cfg, std::string& error) const;

    std::shared_ptr<LaserSession> session_;
    State state_{State::Top};
    PlotArea area_{PlotArea::Display};
    Vec3 corner_{};
    PlotView view_{};
    std::string note_;
    std::string pending_;  // a path awaiting an overwrite answer
};

}  // namespace ncad::laser
