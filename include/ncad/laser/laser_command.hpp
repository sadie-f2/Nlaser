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
//   Laser [DEvice/PEns/ORIgin/ORDer/Job/Move/PReview/Frame/DRyrun/SAve/SEnd/
//          PUlse/STAtus/Counters/STOp/Quit] <Quit>
//   Move  [Goto/+X/-X/+Y/-Y/Fine/Coarse/Step] <done>
//   Pens  [LIst/LOad/SEt/SAve] <done>      (Load takes .lpt or a LightBurn .lbrn2; Save .lbrn2 writes the job)
//   Order [List/Pick/Start/Clear] <done>
//
// Order is the designer's say in the cutting order. List prints the sequence;
// Pick takes entities in the order to cut them; Start takes a point on a closed
// path to start it there, and asks whether to cut it the other way round;
// Clear drops every override. Each re-plans the current job, with the picked
// paths fixed and the rest optimised around them. The overrides are session
// state, named by path id, so a new Job (whose ids are numbered afresh) clears
// them. (Show, the viewport numbering, and keeping them in XDATA are not here.)
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
//   LBRN2CORNER   machine origin corner (bl br tl tr) of LightBurn projects, for
//                 LBRN2IN and Save .lbrn2 when no device is chosen; default tr
//   SVGPXIN       pixels per inch for SVGIN, default 96 (the SVGIN command is
//                 registered by the same call; see svgin_command.hpp)
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
    // Where the last successful move put the head, for the prompt note. Empty
    // for a target with nothing to add.
    virtual std::string last_report() const { return {}; }
};

// Builds a MoveTarget for a device. Null in LaserOptions means the default,
// which moves the head through the device's LaserTransport (from `connector`);
// with no connector at all it is a stub that refuses every move.
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
    Sysvar svg_px{};  // SVGPXIN: pixels per inch for an SVG with no real units
    Sysvar units{};   // LASERUNITS: "mm" or "in", what a drawing unit is
    Sysvar lbrn2_corner{};  // LBRN2CORNER: the machine origin corner of LightBurn files, "tr" default

    // The pen table LBRN2IN took from a LightBurn project. It lives here, not
    // in a file, because a drawing may have no name yet and a stray .lpt beside
    // it would outlive the import; every LASER starts from it until the person
    // chooses a table with Pens Load.
    bool has_imported_pens{false};
    std::vector<LaserLayer> imported_pens;

    // Start From, set by the Origin keyword's sub-prompt and applied to every
    // job built after it (and to the one already built).
    StartMode start{StartMode::Current};

    // The last job built by `Job`: planned, with its reports.
    bool has_job{false};
    LaserJob job;
    PlanReport plan;
    DrawingReport drawing;
    std::string area;  // what it was built from, for messages

    // The job before planning (pens applied), kept so Order can re-plan it, and
    // the designer's overrides on it.
    LaserJob raw;
    PlanOverrides overrides;
};

// Registers LASER and its system variables. Returns the shared session, or
// null if a name clashes (a different type already registered under it).
// Idempotent in the sense register_command is: calling it again replaces the
// command's factory with one over the new session.
std::shared_ptr<LaserSession> install_laser(const LaserOptions& options = {});

// The device LASERDEV names (or the only one in lasers.conf). False with a
// message when there is none; commands that merely like a device's bed size
// (LBRN2IN) ignore that.
bool resolve_laser_device(const LaserSession& session, const Database& db, DeviceConfig& cfg, std::string& error);

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
        OrderMenu,
        OrderPick,
        OrderStartPoint,
        OrderStartReverse,
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
    Step order_menu(const InputValue& value);
    Step order_pick(const InputValue& value);
    Step order_start_point(const InputValue& value);
    Step order_start_reverse(const InputValue& value);
    // Re-plans session_->raw with the overrides and reports `what` and the result.
    Step replan(const std::string& what);
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
    Pt job_origin(CommandContext& ctx) const;
    std::string report_suffix() const;

    bool resolve_device(CommandContext& ctx, DeviceConfig& cfg, std::string& error) const;

    std::shared_ptr<LaserSession> session_;
    std::string origin_note_;  // the Origin report, kept for the Start-from answer
    State state_{State::Top};
    State menu_{State::Top};  // the level one-shot steps return to

    // Mode state. The planned job and Start From live in the session (a later
    // LASER sees the same job); these last as long as the mode does.
    double move_step_{1.0};  // mm
    std::unique_ptr<MoveTarget> move_target_;
    // Non-owning: the default target when move_target_ is one, so each Move
    // visit can hand it the job origin and start mode and drop its anchor.
    class TransportMoveTarget* default_move_{nullptr};
    std::vector<LaserLayer> pens_;  // the working pen table; LASERPENS is its default
    bool pens_loaded_{false};
    bool pens_dirty_{false};
    std::size_t set_pen_{0};  // index into pens_ while Pens Set runs
    double pulse_power_{0.0};
    int pulse_ms_{500};
    std::vector<int> picked_;      // path ids, in the order picked, while Order Pick runs
    int start_id_{-1};             // Order Start: the path chosen,
    std::size_t start_vertex_{0};  // and the vertex of it (as the planner is given the path)
    PlotArea area_{PlotArea::Display};
    Vec3 corner_{};
    PlotView view_{};
    std::string note_;
    std::string pending_;  // a path awaiting an overwrite answer
};

}  // namespace ncad::laser
