// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// LASER: the laser backend as a NotoCAD command.
//
// A resumable command in the style of PLOT -- every prompt is a state, never a
// blocking read -- so the terminal, a script, the GUI and
// `(command "LASER" ...)` all drive the same code. Nothing here exists only in
// a window.
//
// MODAL, as R12's PEDIT is: LASER keeps prompting until Enter or Quit, and each
// keyword does its work and returns to the Laser prompt. Move and Pens are
// sub-modes with prompts of their own, which Enter leaves. Escape (which the
// engine handles, the command never sees it) ends the whole command; what was
// already done -- a job built, a file written, a jog sent -- stays done.
//
//   Laser [DEvice/PEns/Origin/Job/Move/PReview/Frame/DRyrun/SAve/SEnd/PUlse/
//          STAtus/Counters/STOp/Quit] <Quit>
//   Move  [Goto/+X/-X/+Y/-Y/Fine/Coarse/Step] <done>
//   Pens  [LIst/LOad/SEt/SAve] <done>
//
// Output has nowhere to go between prompts (only a finished command carries a
// message), so what a keyword reports is the first line(s) of the next prompt.
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
#include "ncad/laser/device_config.hpp"
#include "ncad/laser/job.hpp"
#include "ncad/laser/laser_renderer.hpp"
#include "ncad/laser/plan.hpp"
#include "ncad/sysvar.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace ncad::laser {

// What the Move sub-mode needs from a machine, and no more, so the command does
// not care how a head is moved. Coordinates are millimetres.
class MoveTarget {
public:
    virtual ~MoveTarget() = default;
    // A relative move by (dx, dy) in drawing axes, laser off.
    virtual bool jog(double dx, double dy, std::string& error) = 0;
    // An absolute move to a point in the drawing's world coordinates, laser off.
    // Mapping that onto the machine is the target's business.
    virtual bool goto_point(double x, double y, std::string& error) = 0;
};

// Builds a MoveTarget for a device. Null in LaserOptions means the stub, which
// refuses every move with a message: goto/jog are not in the transport yet.
using MoveConnector =
    std::function<bool(const DeviceConfig& cfg, std::unique_ptr<MoveTarget>& out, std::string& error)>;

struct LaserOptions {
    // How a device is opened. Null: connect_ruida when built with the network
    // target, otherwise every device operation refuses with a message.
    LaserConnector connector;
    MoveConnector move_connector;
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

    // Start From, set by the Origin keyword's sub-prompt and applied to every
    // job built after it (and to the one already built).
    StartMode start{StartMode::Current};

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
        PensMenu,
        PensLoadFile,
        PensSetNumber,
        PensSetSpeed,
        PensSetPower,
        PensSetPasses,
        PensSetAir,
        JobArea,
        JobWindowFirst,
        JobWindowSecond,
        OriginStart,
        SaveFile,  // a job, or a pen table when menu_ is Pens
        SaveOverwrite,
        SendConfirm,
        PulsePower,
        PulseDuration,
        PulseConfirm,
        MoveMenu,
        MoveGoto,
        MoveStep,
    };
    enum class DeviceOp : std::uint8_t { Send, Frame, Dryrun, Status, Counters, Stop, Pulse };

    // The prompt of a menu level, with `note` (what the last keyword reported)
    // ahead of it. Sets state_ and menu_.
    Step ask_menu(State menu, const std::string& note);
    // A one-shot step finished: its result becomes the note of the menu it was
    // started from. A failure is reported the same way and the mode goes on.
    Step back(const Step& s);

    Step top(CommandContext& ctx, const InputValue& value);
    Step pens_menu(CommandContext& ctx, const InputValue& value);
    Step pens_set(CommandContext& ctx, const InputValue& value);
    Step move_menu(CommandContext& ctx, const InputValue& value);
    Step choose_device(CommandContext& ctx, const InputValue& value);
    Step load_pens(CommandContext& ctx, const InputValue& value);
    bool ensure_pens(CommandContext& ctx, std::string& error);
    Step build_job(CommandContext& ctx);
    Step preview_order() const;
    Step save_job(CommandContext& ctx, const std::string& path);
    Step save_pens(const std::string& path);
    Step begin_send(CommandContext& ctx);
    Step begin_pulse(CommandContext& ctx);
    Step run_device(CommandContext& ctx, DeviceOp op);
    Step jog(CommandContext& ctx, double dx, double dy);
    bool ensure_move_target(CommandContext& ctx, std::string& error);

    bool resolve_device(CommandContext& ctx, DeviceConfig& cfg, std::string& error) const;

    std::shared_ptr<LaserSession> session_;
    std::string origin_note_;  // the Origin report, kept for the Start-from answer
    State state_{State::Top};
    State menu_{State::Top};  // the level one-shot steps return to

    // Mode state. The planned job and Start From live in the session (a later
    // LASER sees the same job); these last as long as the mode does.
    double move_step_{1.0};  // mm
    std::unique_ptr<MoveTarget> move_target_;
    std::vector<LaserLayer> pens_;  // the working pen table; LASERPENS is its default
    bool pens_loaded_{false};
    bool pens_dirty_{false};
    std::size_t set_pen_{0};  // index into pens_ while Pens Set runs
    double pulse_power_{0.0};
    int pulse_ms_{500};
    bool swallow_list_{false};  // "Order List", the old spelling of Preview
    PlotArea area_{PlotArea::Display};
    Vec3 corner_{};
    PlotView view_{};
    std::string note_;
    std::string pending_;  // a path awaiting an overwrite answer
};

}  // namespace ncad::laser
