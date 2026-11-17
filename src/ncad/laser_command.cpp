// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

#include "ncad/laser/laser_command.hpp"
#include "ncad/laser/pdfin_command.hpp"
#include "ncad/laser/lbrn2in_command.hpp"
#include "ncad/laser/svgin_command.hpp"

#include "ncad/commands.hpp"
#include "ncad/database.hpp"
#include "ncad/laser/device_config.hpp"
#include "ncad/laser/lbrn2.hpp"
#include "ncad/laser/ruida_codec.hpp"
#include "ncad/laser/ruida_encode.hpp"
#include "ncad/laser/ruida_job.hpp"
#include "ncad/laser/ruida_transport.hpp"
#include "ncad/laser/status_words.hpp"
#include "ncad/paths.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <thread>

namespace ncad::laser {

using ruida::prepare_dryrun;
using ruida::prepare_frame;
using ruida::prepare_send;

namespace {

std::string fmt(const char* f, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, f);
    std::vsnprintf(buf, sizeof buf, f, ap);
    va_end(ap);
    return buf;
}

bool keyword_is(const InputValue& v, const char* name) {
    return v.kind == InputKind::Keyword && v.text == name;
}

bool ends_with_ci(const std::string& s, const char* suffix) {
    const std::size_t n = std::char_traits<char>::length(suffix);
    if (s.size() < n) return false;
    for (std::size_t i = 0; i < n; ++i) {
        if (std::tolower(static_cast<unsigned char>(s[s.size() - n + i])) != suffix[i]) return false;
    }
    return true;
}

bool has_extension(const std::string& path) {
    const std::size_t slash = path.find_last_of("/\\");
    const std::size_t dot = path.find_last_of('.');
    return dot != std::string::npos && (slash == std::string::npos || dot > slash) && dot + 1 < path.size();
}

// lasers.conf. An unset path means the default location, which may be absent:
// that is "no devices configured", not an error.
bool load_devices(const std::string& configured, std::vector<DeviceConfig>& devices, std::string& error) {
    std::string path = configured;
    if (path.empty()) {
        path = default_device_config_path();
        if (path.empty() || !std::ifstream(path)) {
            devices.clear();
            return true;
        }
    }
    return read_device_config_file(path, devices, error);
}

std::string device_names(const std::vector<DeviceConfig>& devices) {
    std::string s;
    for (const DeviceConfig& d : devices) s += (s.empty() ? "" : ", ") + d.name;
    return s;
}

std::string minutes(double s) {
    if (s < 90.0) return fmt("%.1f s", s);
    return fmt("%d min %02d s", static_cast<int>(s / 60.0), static_cast<int>(std::fmod(s, 60.0)));
}

// What a controller would be told, for the prompt that comes before it.
std::string job_summary(const LaserJob& job, const DeviceConfig& cfg, std::size_t bytes) {
    std::string s = fmt("Job %s on %s\n", job.name.empty() ? "(unnamed)" : job.name.c_str(), cfg.name.c_str());
    for (const LaserLayer& l : job.layers) {
        s += fmt("  layer %d: %g mm/s, power %g-%g%%, passes %d, air %s\n", l.pen, l.speed, l.min_power,
                 l.max_power, l.passes, l.air ? "on" : "off");
    }
    Pt lo, hi;
    if (job_bounds(job, lo, hi)) {
        s += fmt("  bounds %.3f,%.3f .. %.3f,%.3f mm (%.3f x %.3f)\n", lo.x, lo.y, hi.x, hi.y, hi.x - lo.x,
                 hi.y - lo.y);
    }
    s += fmt("  origin %.3f,%.3f, %s; %zu bytes\n", job.origin.x, job.origin.y,
             job.start == StartMode::Absolute ? "absolute machine coordinates"
             : job.start == StartMode::User   ? "placed at the user origin"
                                              : "placed at the head's current position",
             bytes);
    return s;
}

// A job larger than the bed cannot be run, wherever the head is. Where on the
// bed it lands depends on the head's position, which is not known here, so
// this is the check that can be made: size, not position.
bool fits_bed(const LaserJob& job, const DeviceConfig& cfg, std::string& error) {
    if (!(cfg.bed_w_mm > 0.0) || !(cfg.bed_h_mm > 0.0)) return true;
    Pt lo, hi;
    if (!job_bounds(job, lo, hi)) return true;
    const double w = hi.x - lo.x, h = hi.y - lo.y;
    if (w > cfg.bed_w_mm || h > cfg.bed_h_mm) {
        error = fmt("the job is %.1f x %.1f mm and the bed of %s is %.1f x %.1f mm", w, h, cfg.name.c_str(),
                    cfg.bed_w_mm, cfg.bed_h_mm);
        return false;
    }
    return true;
}

// A transport's remark about the last job (a placement it could not verify is
// a WARNING, not a refusal), appended so it is never lost.
std::string with_note(const LaserTransport& t) {
    const std::string n = t.note();
    return n.empty() ? std::string() : "\n" + n;
}

}  // namespace

// --- installation -------------------------------------------------------------

std::shared_ptr<LaserSession> install_laser(const LaserOptions& options) {
    auto session = std::make_shared<LaserSession>();
    session->options = options;
#ifdef NLASER_HAVE_NET
    if (!session->options.connector) session->options.connector = connect_ruida;
#endif

    // Installation settings, not drawing content: NEW and OPEN must not reset
    // which laser the person chose, and none of them is written to the DXF.
    SysvarSpec spec;
    spec.type = SysvarType::String;
    spec.name = "LASERDEV";
    if (!register_sysvar(spec, session->dev)) return nullptr;
    spec.name = "LASERPENS";
    if (!register_sysvar(spec, session->pens)) return nullptr;

    SysvarSpec tol;
    tol.name = "LASERTOL";
    tol.type = SysvarType::Real;
    tol.real_default = kLaserTol;
    if (!register_sysvar(tol, session->tol)) return nullptr;

    SysvarSpec confirm;
    confirm.name = "LASERCONFIRM";
    confirm.type = SysvarType::Int;
    confirm.int_default = 1;
    confirm.int_min = 0;
    confirm.int_max = 1;
    if (!register_sysvar(confirm, session->confirm)) return nullptr;

    SysvarSpec svgpx;
    svgpx.name = "SVGPXIN";
    svgpx.type = SysvarType::Real;
    svgpx.real_default = 96.0;
    if (!register_sysvar(svgpx, session->svg_px)) return nullptr;

    SysvarSpec corner;
    corner.name = "LBRN2CORNER";
    corner.type = SysvarType::String;
    corner.string_default = "tr";
    if (!register_sysvar(corner, session->lbrn2_corner)) return nullptr;

    SysvarSpec units;
    units.name = "LASERUNITS";
    units.type = SysvarType::String;
    units.string_default = "mm";
    if (!register_sysvar(units, session->units)) return nullptr;

    SysvarSpec order;
    order.name = "LASERORDER";
    order.type = SysvarType::String;
    order.string_default = "best";
    if (!register_sysvar(order, session->order)) return nullptr;

    if (!register_command("LASER", [session] { return std::make_unique<LaserCommand>(session); })) {
        return nullptr;
    }
    if (!register_command("LBRN2IN", [session] { return std::make_unique<Lbrn2InCommand>(session); })) {
        return nullptr;
    }
    if (!register_command("SVGIN", [session] { return std::make_unique<SvgInCommand>(session); })) {
        return nullptr;
    }
    if (!register_command("PDFIN", [session] { return std::make_unique<PdfInCommand>(session); })) {
        return nullptr;
    }
    return session;
}

// --- prompts --------------------------------------------------------------------

namespace {

// The prompt texts. The capitalised prefix of each word is what the keyword
// matcher needs to tell it from the others -- "DE" for Device because DRyrun
// also starts with D, "STA"/"STO" because Status and Stop share ST -- and the
// tests check every one of them against the real matcher.
constexpr const char* kTopPrompt =
    "Laser [DEvice/PEns/ORIgin/ORDer/Job/Move/PReview/Frame/DRyrun/SAve/SEnd/PUlse/PAuse/REsume/STAtus/Watch/Counters/STOp/Quit] <Quit>";
constexpr const char* kMovePrompt = "Move [Goto/+X/-X/+Y/-Y/Fine/Coarse/Step] <done>";
constexpr const char* kPensPrompt = "Pens [LIst/LOad/SEt/SAve] <done>";
constexpr const char* kOrderPrompt = "Order [List/Pick/Start/Clear] <done>";

constexpr double kFineStepMm = 0.1;
constexpr double kCoarseStepMm = 10.0;

Step ask_string(std::string message, std::vector<std::string> keywords, bool allow_empty = true) {
    Prompt p;
    p.kind = PromptKind::String;
    p.message = std::move(message);
    p.keywords = std::move(keywords);
    p.allow_empty = allow_empty;
    return Step::ask(std::move(p));
}

Step ask_number(PromptKind kind, std::string message) {
    Prompt p;
    p.kind = kind;
    p.message = std::move(message);
    p.allow_empty = true;
    return Step::ask(std::move(p));
}

// A pen table is `layer` lines in .ljob syntax; write_ljob owns that syntax, so
// borrow it rather than keep a second copy of the format here.
std::string pen_lines(const std::vector<LaserLayer>& pens) {
    LaserJob j;
    j.layers = pens;
    const std::string all = write_ljob(j);
    std::string out;
    std::size_t at = 0;
    while (at < all.size()) {
        std::size_t nl = all.find('\n', at);
        if (nl == std::string::npos) nl = all.size();
        if (all.compare(at, 6, "layer ") == 0) out.append(all, at, nl - at + (nl < all.size() ? 1 : 0));
        at = nl + 1;
    }
    return out;
}

// Used only when there is no way to reach a machine at all (no connector).
class StubMoveTarget final : public MoveTarget {
public:
    bool jog(double, double, std::string& error) override { return unavailable(error); }
    bool goto_point(double, double, std::string& error) override { return unavailable(error); }

private:
    static bool unavailable(std::string& error) {
        error = "this build has no network support (NLASER_WITH_NET is off)";
        return false;
    }
};

}  // namespace

// Move over a device's transport. These are pointer moves (laser off, no
// red-dot offset), like frame and the transport's goto.
//
// Goto has two meanings, set by Start From. Absolute: the drawing is the bed,
// so the point is a bed coordinate. Current: the job sits wherever the head
// was, so the head's position when Move was entered is the anchor, and a
// point goes to anchor + (p - job origin) as the machine counts it. The
// anchor is read once and never re-read, so repeated Gotos cannot drift and
// jogs (which move the head) do not disturb it.
class TransportMoveTarget final : public MoveTarget {
public:
    TransportMoveTarget(std::unique_ptr<LaserTransport> t, const DeviceConfig& cfg)
        : t_(std::move(t)), cfg_(cfg), machine_(ruida::machine_of(cfg)) {}

    bool open(std::string& error) { return t_->open(error); }

    // Called as Move is entered, and before each Goto for the origin.
    void enter() { have_anchor_ = false; }
    void set_frame(StartMode start, Pt origin) {
        start_ = start;
        origin_ = origin;
    }

    bool jog(double dx, double dy, std::string& error) override {
        return go(MoveFrame::Offset, dx, dy, error);
    }

    bool goto_point(double x, double y, std::string& error) override {
        if (start_ == StartMode::Absolute) return go(MoveFrame::Bed, x, y, error);
        if (!have_anchor_) {
            MachineStatus st;
            std::string why;
            if (!t_->poll(st, why) || !st.has_position) {
                error = "cannot read the head position" + (why.empty() ? std::string() : " (" + why + ")") +
                        ", so Goto from the current position is unavailable; jog still works, or use Start From Absolute";
                return false;
            }
            anchor_x_ = st.x_mm;
            anchor_y_ = st.y_mm;
            have_anchor_ = true;
        }
        return go(MoveFrame::Machine, anchor_x_ + machine_.sign_x() * (x - origin_.x),
                  anchor_y_ + machine_.sign_y() * (y - origin_.y), error);
    }

    std::string last_report() const override { return report_; }

private:
    bool go(MoveFrame frame, double x, double y, std::string& error) {
        MoveReport r;
        report_.clear();
        if (!t_->move_head(frame, x, y, 0.0, r, error)) return false;
        report_ = fmt("head at machine %.3f,%.3f", r.machine_x, r.machine_y);
        if (r.has_bed_pos) report_ += fmt(" (bed %.3f,%.3f)", r.bed_x, r.bed_y);
        if (!r.note.empty()) report_ += "; " + r.note;
        return true;
    }

    std::unique_ptr<LaserTransport> t_;
    DeviceConfig cfg_;
    ruida::RdMachine machine_;
    StartMode start_{StartMode::Current};
    Pt origin_{};
    bool have_anchor_{false};
    double anchor_x_{0.0}, anchor_y_{0.0};
    std::string report_;
};

Step LaserCommand::ask_menu(State menu, const std::string& note) {
    state_ = menu_ = menu;
    const char* text = menu == State::MoveMenu    ? kMovePrompt
                       : menu == State::PensMenu  ? kPensPrompt
                       : menu == State::OrderMenu ? kOrderPrompt
                                                  : kTopPrompt;
    std::string msg = note.empty() ? std::string() : note + "\n";
    msg += text;
    if (menu == State::MoveMenu) {
        msg += fmt(" (step %g mm)", move_step_);
        return ask_string(msg, {"Goto", "+X", "-X", "+Y", "-Y", "Fine", "Coarse", "Step"});
    }
    if (menu == State::PensMenu) return ask_string(msg, {"List", "Load", "Set", "Save"});
    if (menu == State::OrderMenu) return ask_string(msg, {"List", "Pick", "Start", "Clear"});
    return ask_string(msg, {"Device", "Pens", "Origin", "Order", "Job", "Move", "Preview", "Frame", "DRyrun", "Save", "Send",
                            "Pulse", "PAuse", "REsume", "STatus", "Watch", "Counters", "STop", "Quit"});
}

Step LaserCommand::back(const Step& s) {
    if (s.kind == StepKind::Prompt || s.kind == StepKind::Cancelled) return s;
    return ask_menu(menu_, s.kind == StepKind::Failed ? "error: " + s.message : s.message);
}

Step LaserCommand::start(CommandContext&) {
    if (!session_) return Step::failed("the laser backend is not installed");
    return ask_menu(State::Top, "");
}

Step LaserCommand::top(CommandContext& ctx, const InputValue& value) {
    // Enter and Quit leave. Unsaved pen edits are said so: they are the one
    // thing that dies with the mode.
    if (value.kind == InputKind::None || keyword_is(value, "QUIT")) {
        return Step::done(pens_dirty_ ? "Pen table edits were not saved (Pens Save)" : "");
    }

    if (keyword_is(value, "ORDER")) {
        if (!session_->has_job) return back(Step::failed("no job yet: run LASER Job first"));
        return ask_menu(State::OrderMenu, "");
    }

    if (keyword_is(value, "JOB")) {
        state_ = State::JobArea;
        return ask_string("Area [Extents/Window/Display/Limits] <Extents>", {"Extents", "Window", "Display", "Limits"});
    }

    if (keyword_is(value, "DEVICE")) {
        std::vector<DeviceConfig> devices;
        std::string error;
        if (!load_devices(session_->options.config_path, devices, error)) return back(Step::failed(error));
        const std::string& current = ctx.db.sysvars().get_string(session_->dev);
        state_ = State::DeviceName;
        std::string msg = "Device";
        msg += devices.empty() ? " (none in lasers.conf)" : " (" + device_names(devices) + ")";
        msg += " [Offset]";
        if (!current.empty()) msg += " <" + current + ">";
        return ask_string(msg, {"Offset"});
    }

    if (keyword_is(value, "PENS")) return ask_menu(State::PensMenu, "");
    if (keyword_is(value, "MOVE")) {
        // Each visit takes a new anchor: the head may have been moved since.
        if (default_move_) default_move_->enter();
        return ask_menu(State::MoveMenu, "");
    }

    if (keyword_is(value, "ORIGIN")) {
        // The job origin is a fixed point in the drawing, never a corner of the
        // selection: it is the origin of the UCS named LASER. Shown, not set
        // here, because UCS already knows how to define one (by points, by
        // snap) and a second way to do it would only disagree. What this does
        // set is where the controller places the job: Start From.
        std::string s;
        const UcsId id = ctx.db.find_ucs("LASER");
        if (id == kInvalidUcs) {
            s = "Job origin 0.000,0.000 (world; there is no UCS named LASER). Save a UCS named LASER to set it.";
        } else {
            const Ucs u = ctx.db.ucs(id).ucs.normalized();
            s = fmt("Job origin %.3f,%.3f (UCS LASER)", u.origin.x, u.origin.y);
            if (std::abs(u.xdir.x - 1.0) > 1e-9 || std::abs(u.xdir.y) > 1e-9) {
                s += "; its rotation is not used yet, only its origin";
            }
        }
        state_ = State::OriginStart;
        origin_note_ = s;
        return ask_string(s + "\nStart from [Current/Absolute/User] <" +
                              (session_->start == StartMode::Absolute ? "Absolute"
                               : session_->start == StartMode::User   ? "User"
                                                                      : "Current") + ">",
                          {"Current", "Absolute", "User"});
    }

    if (keyword_is(value, "PREVIEW")) {
        if (!session_->has_job) return back(Step::failed("no job yet: run LASER Job first"));
        return back(preview_order());
    }

    if (keyword_is(value, "SAVE")) {
        if (!session_->has_job) return back(Step::failed("no job yet: run LASER Job first"));
        state_ = State::SaveFile;
        Prompt p;
        p.kind = PromptKind::String;
        p.message = "Save job as (.ljob, .rd or .lbrn2)";
        p.file = FileIntent::Save;
        p.file_extension = "ljob";
        return Step::ask(p);
    }

    if (keyword_is(value, "SEND")) return back(begin_send(ctx));
    if (keyword_is(value, "PULSE")) return back(begin_pulse(ctx));
    if (keyword_is(value, "FRAME")) return back(run_device(ctx, DeviceOp::Frame));
    if (keyword_is(value, "DRYRUN")) return back(run_device(ctx, DeviceOp::Dryrun));
    if (keyword_is(value, "STATUS")) return back(run_device(ctx, DeviceOp::Status));
    if (keyword_is(value, "COUNTERS")) return back(run_device(ctx, DeviceOp::Counters));
    if (keyword_is(value, "STOP")) return back(run_device(ctx, DeviceOp::Stop));
    if (keyword_is(value, "PAUSE")) return back(run_device(ctx, DeviceOp::Pause));
    if (keyword_is(value, "RESUME")) return back(run_device(ctx, DeviceOp::Resume));
    if (keyword_is(value, "WATCH")) return begin_watch();

    return back(Step::failed("unknown LASER option"));
}

// --- Device and Pens ----------------------------------------------------------------

// The red-dot pointer offset of the current device: shown, then asked as X and
// Y in mm (Enter keeps each). It applies at once (the session remembers it)
// and is written into the device's section of lasers.conf, that one key only.
// power_limit has no such path, on purpose (docs/design-notes.md).
Step LaserCommand::begin_offset(CommandContext& ctx) {
    DeviceConfig cfg;
    std::string error;
    if (!resolve_device(ctx, cfg, error)) return back(Step::failed(error));
    offset_x_ = cfg.pointer_offset_x;
    offset_y_ = cfg.pointer_offset_y;
    state_ = State::DeviceOffsetX;
    Prompt p;
    p.kind = PromptKind::Real;
    p.message = fmt("Pointer offset of %s is %g, %g mm (machine frame; the beam is this far from the red dot)\n"
                    "New X mm <%g>",
                    cfg.name.c_str(), offset_x_, offset_y_, offset_x_);
    p.allow_empty = true;
    return Step::ask(p);
}

Step LaserCommand::offset_x(const InputValue& value) {
    if (value.kind == InputKind::Real) offset_x_ = value.real;
    else if (value.kind != InputKind::None) return back(Step::failed("an offset in mm is required"));
    state_ = State::DeviceOffsetY;
    return ask_number(PromptKind::Real, fmt("New Y mm <%g>", offset_y_));
}

Step LaserCommand::offset_y(CommandContext& ctx, const InputValue& value) {
    if (value.kind == InputKind::Real) offset_y_ = value.real;
    else if (value.kind != InputKind::None) return back(Step::failed("an offset in mm is required"));
    DeviceConfig cfg;
    std::string error;
    if (!resolve_device(ctx, cfg, error)) return back(Step::failed(error));
    if (!std::isfinite(offset_x_) || !std::isfinite(offset_y_) || std::fabs(offset_x_) > 1000.0 ||
        std::fabs(offset_y_) > 1000.0) {
        return back(Step::failed("the pointer offset must be within 1000 mm"));
    }
    session_->pointer_offsets[cfg.name] = {offset_x_, offset_y_};

    std::string path = session_->options.config_path;
    if (path.empty()) path = default_device_config_path();
    std::string what = fmt("Pointer offset of %s is %g, %g mm", cfg.name.c_str(), offset_x_, offset_y_);
    if (path.empty() || !set_pointer_offset_file(path, cfg.name, offset_x_, offset_y_, error)) {
        if (path.empty()) error = "no lasers.conf location";
        return back(Step::done(what + " for this session only; not saved: " + error));
    }
    session_->pointer_offsets.erase(cfg.name);  // the file now says it
    return back(Step::done(what + ", saved in " + path));
}

// Watch blocks the UI thread for the whole duration (text mode); the GUI panel
// will move it to a worker thread (docs/design-notes.md).
Step LaserCommand::begin_watch() {
    state_ = State::WatchSeconds;
    return ask_number(PromptKind::Integer, fmt("Watch for how many seconds <%d>", watch_s_));
}

Step LaserCommand::choose_device(CommandContext& ctx, const InputValue& value) {
    std::vector<DeviceConfig> devices;
    std::string error;
    if (!load_devices(session_->options.config_path, devices, error)) return Step::failed(error);

    if (value.kind == InputKind::None) {
        const std::string& cur = ctx.db.sysvars().get_string(session_->dev);
        return Step::done(cur.empty() ? "No device chosen" : "Device " + cur);
    }
    if (keyword_is(value, "OFFSET")) return begin_offset(ctx);
    if (value.kind != InputKind::String || value.text.empty()) return Step::failed("a device name is required");

    DeviceConfig cfg;
    if (!find_device(devices, value.text, cfg)) {
        return Step::failed("no device '" + value.text + "' in lasers.conf" +
                            (devices.empty() ? "" : " (" + device_names(devices) + ")"));
    }
    ctx.db.sysvars().set_string(session_->dev, cfg.name);
    // A move target belongs to the device it was built for.
    move_target_.reset();
    default_move_ = nullptr;
    std::string where = cfg.serial.empty() ? cfg.host : cfg.serial;
    return Step::done("Device " + cfg.name + (where.empty() ? "" : " at " + where));
}

// A pen table from .lpt, or from the layers of a LightBurn project (keyed by
// the ACI each layer takes on LBRN2IN, so they match a drawing's entities).
static bool read_pens_any(const std::string& path, std::vector<LaserLayer>& pens, std::string& error) {
    if (!ends_with_ci(path, ".lbrn2")) return read_pen_table_file(path, pens, error);
    LaserJob job;
    Lbrn2Info info;
    if (!read_lbrn2_file(path, job, info, Lbrn2Options{}, error)) return false;
    pens = lbrn2_pens(job);
    return true;
}

// The working pen table is read from LASERPENS the first time anything needs
// it; after that it is the table, edits included. A table LBRN2IN imported
// comes first: it is the person's latest word until Pens Load says otherwise.
bool LaserCommand::ensure_pens(CommandContext& ctx, std::string& error) {
    if (pens_loaded_) return true;
    if (session_->has_imported_pens) {
        pens_ = session_->imported_pens;
        pens_loaded_ = true;
        return true;
    }
    const std::string& path = ctx.db.sysvars().get_string(session_->pens);
    if (!path.empty() && !read_pens_any(path, pens_, error)) return false;
    pens_loaded_ = true;
    return true;
}

Step LaserCommand::load_pens(CommandContext& ctx, const InputValue& value) {
    if (value.kind == InputKind::None) {
        const std::string& cur = ctx.db.sysvars().get_string(session_->pens);
        return Step::done(cur.empty() ? "No pen table: every pen runs at power 0" : "Pen table " + cur);
    }
    if (value.kind != InputKind::String || value.text.empty()) return Step::failed("a file name is required");

    if (value.text == ".") {
        ctx.db.sysvars().set_string(session_->pens, "");
        session_->has_imported_pens = false;
        pens_.clear();
        pens_loaded_ = true;
        pens_dirty_ = false;
        return Step::done("No pen table");
    }
    const std::string path = ensure_extension(expand_user_path(value.text), ".lpt");
    std::vector<LaserLayer> pens;
    std::string error;
    // Read now rather than at Job, so a typo is reported where it was typed.
    if (!read_pens_any(path, pens, error)) return Step::failed(error);
    session_->has_imported_pens = false;
    ctx.db.sysvars().set_string(session_->pens, path);
    pens_ = std::move(pens);
    pens_loaded_ = true;
    pens_dirty_ = false;
    return Step::done(fmt("Pen table %s (%zu pens)", path.c_str(), pens_.size()));
}

Step LaserCommand::pens_menu(CommandContext& ctx, const InputValue& value) {
    if (value.kind == InputKind::None) return ask_menu(State::Top, "");

    if (keyword_is(value, "LIST")) {
        std::string error;
        if (!ensure_pens(ctx, error)) return back(Step::failed(error));
        const std::string& path = ctx.db.sysvars().get_string(session_->pens);
        std::string s = path.empty() ? "Pen table (not from a file)" : "Pen table " + path;
        s += pens_dirty_ ? " (edited, not saved)\n" : "\n";
        s += pens_.empty() ? "  no pens: every pen runs at power 0" : pen_lines(pens_);
        if (!s.empty() && s.back() == '\n') s.pop_back();
        return back(Step::done(s));
    }
    if (keyword_is(value, "LOAD")) {
        state_ = State::PensLoadFile;
        const std::string& cur = ctx.db.sysvars().get_string(session_->pens);
        Prompt p;
        p.kind = PromptKind::String;
        p.message = "Pen table file (.lpt or .lbrn2, . for none)";
        if (!cur.empty()) p.message += " <" + cur + ">";
        p.file = FileIntent::Open;
        p.file_extension = "lpt";
        p.allow_empty = true;
        return Step::ask(p);
    }
    if (keyword_is(value, "SET")) {
        std::string error;
        if (!ensure_pens(ctx, error)) return back(Step::failed(error));
        state_ = State::PensSetNumber;
        return ask_number(PromptKind::Integer, "Pen number (1-255) <done>");
    }
    if (keyword_is(value, "SAVE")) {
        std::string error;
        if (!ensure_pens(ctx, error)) return back(Step::failed(error));
        if (pens_.empty()) return back(Step::failed("no pens to save: Load or Set some first"));
        state_ = State::SaveFile;
        Prompt p;
        p.kind = PromptKind::String;
        p.message = "Save pen table as (.lpt)";
        p.file = FileIntent::Save;
        p.file_extension = "lpt";
        return Step::ask(p);
    }
    return back(Step::failed("unknown Pens option"));
}

// Pens Set: pen, then speed, power, passes, air. Enter keeps a value, and Enter
// at the pen number returns to the Pens prompt. A pen the table lacks is made,
// at power 0 like any pen the table does not list.
Step LaserCommand::pens_set(CommandContext&, const InputValue& value) {
    auto numeric_ok = [&](InputKind want) { return value.kind == InputKind::None || value.kind == want; };
    LaserLayer* l = state_ == State::PensSetNumber || set_pen_ >= pens_.size() ? nullptr : &pens_[set_pen_];

    switch (state_) {
        case State::PensSetNumber: {
            if (value.kind == InputKind::None) return ask_menu(State::PensMenu, "");
            if (value.kind != InputKind::Integer || value.integer < 1 || value.integer > 255) {
                return back(Step::failed("a pen number from 1 to 255 is required"));
            }
            set_pen_ = pens_.size();
            for (std::size_t i = 0; i < pens_.size(); ++i) {
                if (pens_[i].pen == value.integer) set_pen_ = i;
            }
            if (set_pen_ == pens_.size()) {
                LaserLayer n;
                n.pen = n.color = value.integer;
                pens_.push_back(n);
                pens_dirty_ = true;
            }
            state_ = State::PensSetSpeed;
            return ask_number(PromptKind::Real, fmt("Speed mm/s <%g>", pens_[set_pen_].speed));
        }
        case State::PensSetSpeed:
            if (!l || !numeric_ok(InputKind::Real)) return back(Step::failed("a speed in mm/s is required"));
            if (value.kind == InputKind::Real) {
                if (!(value.real > 0.0) || !std::isfinite(value.real)) return back(Step::failed("speed must be positive"));
                l->speed = value.real;
                pens_dirty_ = true;
            }
            state_ = State::PensSetPower;
            return ask_number(PromptKind::Real, fmt("Power %% <%g>", l->max_power));
        case State::PensSetPower:
            if (!l || !numeric_ok(InputKind::Real)) return back(Step::failed("a power in percent is required"));
            if (value.kind == InputKind::Real) {
                if (!(value.real >= 0.0) || !(value.real <= 100.0)) return back(Step::failed("power is 0 to 100 percent"));
                l->max_power = value.real;
                if (l->min_power > value.real) l->min_power = value.real;
                pens_dirty_ = true;
            }
            state_ = State::PensSetPasses;
            return ask_number(PromptKind::Integer, fmt("Passes <%d>", l->passes));
        case State::PensSetPasses:
            if (!l || !numeric_ok(InputKind::Integer)) return back(Step::failed("a pass count is required"));
            if (value.kind == InputKind::Integer) {
                if (value.integer < 1 || value.integer > 99) return back(Step::failed("passes is 1 to 99"));
                l->passes = value.integer;
                pens_dirty_ = true;
            }
            state_ = State::PensSetAir;
            return ask_string(std::string("Air assist [ON/OFf] <") + (l->air ? "On" : "Off") + ">", {"On", "Off"});
        default: break;  // PensSetAir
    }
    if (!l) return back(Step::failed("internal state error"));
    if (value.kind != InputKind::None) {
        if (keyword_is(value, "ON")) l->air = true;
        else if (keyword_is(value, "OFF")) l->air = false;
        else return back(Step::failed("Air: On or Off"));
        pens_dirty_ = true;
    }
    menu_ = State::PensMenu;
    return back(Step::done(fmt("pen %d: %g mm/s, power %g-%g%%, passes %d, air %s", l->pen, l->speed, l->min_power,
                               l->max_power, l->passes, l->air ? "on" : "off")));
}

// --- Move -----------------------------------------------------------------------------

bool LaserCommand::ensure_move_target(CommandContext& ctx, std::string& error) {
    if (move_target_) return true;
    default_move_ = nullptr;
    DeviceConfig cfg;
    if (!resolve_device(ctx, cfg, error)) return false;
    if (session_->options.move_connector) {
        return session_->options.move_connector(cfg, move_target_, error) && move_target_ != nullptr;
    }
    if (!session_->options.connector) {
        move_target_ = std::make_unique<StubMoveTarget>();
        return true;
    }
    std::unique_ptr<LaserTransport> t;
    if (!session_->options.connector(cfg, t, error) || !t) return false;
    auto target = std::make_unique<TransportMoveTarget>(std::move(t), cfg);
    if (!target->open(error)) return false;
    default_move_ = target.get();
    move_target_ = std::move(target);
    return true;
}

// The job origin is the origin of UCS LASER, or the world origin.
Pt LaserCommand::job_origin(CommandContext& ctx) const {
    const UcsId id = ctx.db.find_ucs("LASER");
    if (id == kInvalidUcs) return {};
    const Ucs u = ctx.db.ucs(id).ucs.normalized();
    return {u.origin.x, u.origin.y};
}

Step LaserCommand::jog(CommandContext& ctx, double dx, double dy) {
    std::string error;
    if (!ensure_move_target(ctx, error) || !move_target_->jog(dx, dy, error)) {
        return ask_menu(State::MoveMenu, "error: " + error);
    }
    return ask_menu(State::MoveMenu, fmt("moved %+g,%+g mm", dx, dy) + report_suffix());
}

std::string LaserCommand::report_suffix() const {
    const std::string r = move_target_ ? move_target_->last_report() : std::string();
    return r.empty() ? r : "; " + r;
}

Step LaserCommand::move_menu(CommandContext& ctx, const InputValue& value) {
    if (value.kind == InputKind::None) return ask_menu(State::Top, "");
    if (keyword_is(value, "+X")) return jog(ctx, move_step_, 0.0);
    if (keyword_is(value, "-X")) return jog(ctx, -move_step_, 0.0);
    if (keyword_is(value, "+Y")) return jog(ctx, 0.0, move_step_);
    if (keyword_is(value, "-Y")) return jog(ctx, 0.0, -move_step_);
    if (keyword_is(value, "FINE")) {
        move_step_ = kFineStepMm;
        return ask_menu(State::MoveMenu, "");
    }
    if (keyword_is(value, "COARSE")) {
        move_step_ = kCoarseStepMm;
        return ask_menu(State::MoveMenu, "");
    }
    if (keyword_is(value, "STEP")) {
        state_ = State::MoveStep;
        return ask_number(PromptKind::Real, fmt("Step in mm <%g>", move_step_));
    }
    if (keyword_is(value, "GOTO")) {
        state_ = State::MoveGoto;
        Prompt p;
        p.kind = PromptKind::Point;
        p.message = "Go to point";
        return Step::ask(p);
    }
    return ask_menu(State::MoveMenu, "error: unknown Move option");
}

// --- Job ----------------------------------------------------------------------------

Step LaserCommand::build_job(CommandContext& ctx) {
    const Vec3 normal = ctx.db.construction_normal();
    note_.clear();
    // Extents draws everything and needs no view, which matters: a drawing
    // that is one straight line has a box with no height, an invalid page for
    // PLOT but a perfectly good job.
    bool use_view = area_ != PlotArea::Extents;

    switch (area_) {
        case PlotArea::Display:
            if (ctx.view != nullptr) {
                const DrawContext dc = ctx.view->draw_context();
                if (dc.clip_active) {
                    view_.origin = dc.clip_origin;
                    view_.ax = dc.clip_x;
                    view_.ay = dc.clip_y;
                    view_.min_x = dc.clip_min_x;
                    view_.max_x = dc.clip_max_x;
                    view_.min_y = dc.clip_min_y;
                    view_.max_y = dc.clip_max_y;
                    break;
                }
            }
            // Said plainly, as PLOT does, rather than cutting something else
            // without a word.
            note_ = ctx.view == nullptr ? "No display here; using Extents. " : "The view reports no bounds; using Extents. ";
            use_view = false;
            break;
        case PlotArea::Extents: break;
        case PlotArea::Limits: {
            BBox box;
            box.expand(ctx.db.sysvars().get_point(Sysvar::LimMin));
            box.expand(ctx.db.sysvars().get_point(Sysvar::LimMax));
            view_ = plot_view_for_box(box, normal);
            break;
        }
        case PlotArea::Window: break;  // built from the two picks
    }
    if (use_view && !view_.valid()) {
        return Step::failed(std::string("nothing to cut: ") + plot_area_name(area_) + " is empty");
    }

    const std::string& order = ctx.db.sysvars().get_string(session_->order);
    if (order != "best" && order != "lightburn") {
        return Step::failed("LASERORDER must be best or lightburn, not '" + order + "'");
    }
    const double tol = ctx.db.sysvars().get_real(session_->tol);
    if (!(tol > 0.0) || !std::isfinite(tol)) return Step::failed("LASERTOL must be a positive number of mm");

    DrawingOptions opts;
    const std::string& units = ctx.db.sysvars().get_string(session_->units);
    if (units == "mm") {
        opts.units = DrawingUnits::Mm;
    } else if (units == "in") {
        opts.units = DrawingUnits::Inch;
    } else {
        return Step::failed("LASERUNITS must be mm or in, not '" + units + "'");
    }
    opts.chord_tolerance = tol;
    opts.has_view = use_view;
    opts.view = view_;

    std::string error;
    if (!ensure_pens(ctx, error)) return Step::failed(error);
    const std::vector<LaserLayer>& pens = pens_;

    LaserJob raw;
    DrawingReport report;
    if (!render_drawing(ctx.db, opts, raw, report, error)) return Step::failed(error);
    if (raw.paths.empty()) {
        return Step::failed(std::string("nothing to cut in ") + plot_area_name(area_));
    }

    const std::string& dwg = ctx.db.sysvars().get_string(Sysvar::DwgName);
    raw.source = dwg;
    raw.name = dwg;
    if (ends_with_ci(raw.name, ".dxf")) raw.name.resize(raw.name.size() - 4);
    apply_pens(raw, pens, report);

    // Path ids are numbered afresh for each drawing read, so overrides made
    // against the last job would name other paths.
    const bool dropped = !session_->overrides.empty();
    session_->overrides = PlanOverrides{};
    LaserJob planned;
    PlanReport plan;
    session_->lightburn_order = order == "lightburn";
    PlanOptions popts;
    if (session_->lightburn_order) use_lightburn_order(popts);
    if (!plan_job(raw, planned, popts, plan, error)) return Step::failed(error);

    session_->raw = raw;
    session_->job = std::move(planned);
    session_->job.start = session_->start;
    session_->plan = plan;
    session_->drawing = report;
    session_->area = plot_area_name(area_);
    session_->has_job = true;

    const LaserJob& job = session_->job;
    std::string s = note_;
    s += fmt("Job from %s: %zu paths on %zu pens, %zu entities", session_->area.c_str(), job.paths.size(),
             job.layers.size(), report.entities);
    if (report.hidden + report.hidden_in_blocks) {
        s += fmt(" (%zu on off or frozen layers not cut)", report.hidden + report.hidden_in_blocks);
    }
    s += "\n";
    for (const LaserLayer& l : job.layers) {
        s += fmt("  layer %d: %s, %g mm/s, power %g-%g%%, passes %d\n", l.pen, layer_mode_name(l.mode), l.speed,
                 l.min_power, l.max_power, l.passes);
    }
    Pt lo, hi;
    if (job_bounds(job, lo, hi)) {
        s += fmt("Bounds %.3f,%.3f .. %.3f,%.3f mm (%.3f x %.3f)\n", lo.x, lo.y, hi.x, hi.y, hi.x - lo.x,
                 hi.y - lo.y);
    }
    s += fmt("Drawing units %s%s\n", units_name(report.units), report.units == DrawingUnits::Inch ? " (x25.4 to mm)" : "");
    s += fmt("Origin %.3f,%.3f (%s)\n", job.origin.x, job.origin.y, report.laser_ucs ? "UCS LASER" : "world");
    s += fmt("Cut %.1f mm, travel %.1f mm, estimated %s", plan.cut_mm, plan.travel_mm, minutes(plan.estimated_s).c_str());
    if (plan.crossing_mm > 0.0) s += fmt("\nTravel crosses cut-free parts for %.1f mm", plan.crossing_mm);
    if (dropped) s += "\nOrder overrides were cleared by the new job";
    for (const std::string& w : plan.warnings) s += "\nwarning: " + w;
    for (const std::string& w : report.warnings) s += "\nwarning: " + w;
    return Step::done(s);
}

Step LaserCommand::preview_order() const {
    const LaserJob& job = session_->job;
    std::string s = fmt("Cutting order: %zu paths\n", job.paths.size());
    constexpr std::size_t kMaxLines = 200;
    std::size_t n = 0;
    for (const LaserPath& p : job.paths) {
        if (++n > kMaxLines) {
            s += fmt("  ... %zu more\n", job.paths.size() - kMaxLines);
            break;
        }
        double len = 0.0;
        for (std::size_t i = 1; i < p.pts.size(); ++i) {
            len += std::hypot(p.pts[i].x - p.pts[i - 1].x, p.pts[i].y - p.pts[i - 1].y);
        }
        if (p.closed && p.pts.size() > 1) {
            len += std::hypot(p.pts.front().x - p.pts.back().x, p.pts.front().y - p.pts.back().y);
        }
        const Pt& a = p.pts.front();
        const Pt& b = p.closed ? p.pts.front() : p.pts.back();
        s += fmt("  %3zu  pen %d  %s  %zu pts  %.3f,%.3f -> %.3f,%.3f  %.2f mm\n", n, p.pen,
                 p.closed ? "closed" : "open  ", p.pts.size(), a.x, a.y, b.x, b.y, len);
        if (p.id >= 0) s.insert(s.size() - 1, fmt("  id %d", p.id));
    }
    return Step::done(s);
}

// --- Order --------------------------------------------------------------------------

Step LaserCommand::replan(const std::string& what) {
    PlanOptions opts;
    if (session_->lightburn_order) use_lightburn_order(opts);
    opts.overrides = session_->overrides;
    LaserJob planned;
    PlanReport plan;
    std::string error;
    if (!plan_job(session_->raw, planned, opts, plan, error)) return Step::failed(error);
    planned.start = session_->start;
    session_->job = std::move(planned);
    session_->plan = plan;
    std::string s = what.empty() ? std::string() : what + "\n";
    s += fmt("Cut %.1f mm, travel %.1f mm, estimated %s", plan.cut_mm, plan.travel_mm, minutes(plan.estimated_s).c_str());
    if (plan.crossing_mm > 0.0) s += fmt("\nTravel crosses cut-free parts for %.1f mm", plan.crossing_mm);
    for (const std::string& w : plan.warnings) s += "\nwarning: " + w;
    return Step::done(s);
}

Step LaserCommand::order_menu(const InputValue& value) {
    if (value.kind == InputKind::None) return ask_menu(State::Top, "");
    if (keyword_is(value, "LIST")) return back(preview_order());
    if (keyword_is(value, "PICK")) {
        picked_.clear();
        state_ = State::OrderPick;
        return ask_number(PromptKind::Entity, "Pick entities in cutting order <done>");
    }
    if (keyword_is(value, "START")) {
        state_ = State::OrderStartPoint;
        return ask_number(PromptKind::Point, "Point on a closed path to start it at");
    }
    if (keyword_is(value, "CLEAR")) {
        const std::size_t n = session_->overrides.steps.size();
        session_->overrides = PlanOverrides{};
        return back(replan(fmt("Cleared %zu override%s", n, n == 1 ? "" : "s")));
    }
    return back(Step::failed("unknown Order option"));
}

Step LaserCommand::order_pick(const InputValue& value) {
    if (value.kind == InputKind::None) {
        if (picked_.empty()) return ask_menu(State::OrderMenu, "");
        // The picked paths take the order; a start already set for one stays.
        // A path with only a start set (free_order) keeps its start and goes
        // back to the optimiser unless it was picked.
        std::vector<FixedStep> steps;
        for (int id : picked_) {
            FixedStep s;
            s.id = id;
            for (const FixedStep& old : session_->overrides.steps) {
                if (old.id == id) s = old;
            }
            s.free_order = false;
            steps.push_back(s);
        }
        for (const FixedStep& old : session_->overrides.steps) {
            if (std::find(picked_.begin(), picked_.end(), old.id) == picked_.end() && old.free_order) {
                steps.push_back(old);
            }
        }
        const std::size_t n = picked_.size();
        session_->overrides.steps = std::move(steps);
        picked_.clear();
        return back(replan(fmt("Fixed the order of %zu path%s", n, n == 1 ? "" : "s")));
    }
    if (value.kind != InputKind::Entity) return ask_menu(State::OrderMenu, "error: pick an entity");

    std::size_t added = 0;
    for (const LaserPath& p : session_->raw.paths) {
        if (p.source != value.entity || p.id < 0) continue;
        if (std::find(picked_.begin(), picked_.end(), p.id) != picked_.end()) continue;
        picked_.push_back(p.id);
        ++added;
    }
    state_ = State::OrderPick;
    return ask_number(PromptKind::Entity,
                      (added ? fmt("%zu path%s added (%zu picked)\n", added, added == 1 ? "" : "s", picked_.size())
                             : std::string("that entity has no path in the job\n")) +
                          "Pick next entity <done>");
}

namespace {

double point_segment_distance(const Pt& p, const Pt& a, const Pt& b) {
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double len2 = dx * dx + dy * dy;
    double t = len2 > 0.0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2 : 0.0;
    t = std::min(1.0, std::max(0.0, t));
    return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

}  // namespace

Step LaserCommand::order_start_point(const InputValue& value) {
    if (value.kind != InputKind::Point) return ask_menu(State::OrderMenu, "error: a point is required");
    const Pt q{value.point.x, value.point.y};

    // The closed path whose outline passes nearest the point, then its nearest
    // vertex.
    const LaserPath* best = nullptr;
    double best_d = 0.0;
    for (const LaserPath& p : session_->job.paths) {
        if (!p.closed || p.id < 0 || p.pts.size() < 3) continue;
        for (std::size_t i = 0; i < p.pts.size(); ++i) {
            const double d = point_segment_distance(q, p.pts[i], p.pts[(i + 1) % p.pts.size()]);
            if (!best || d < best_d) {
                best = &p;
                best_d = d;
            }
        }
    }
    if (!best) return ask_menu(State::OrderMenu, "error: the job has no closed path to start");
    std::size_t v = 0;
    for (std::size_t i = 1; i < best->pts.size(); ++i) {
        if (std::hypot(best->pts[i].x - q.x, best->pts[i].y - q.y) <
            std::hypot(best->pts[v].x - q.x, best->pts[v].y - q.y)) {
            v = i;
        }
    }
    const Pt at = best->pts[v];

    // The planner counts vertices in the path as it is before ordering, not as
    // it was rotated to start; find the same vertex there.
    std::vector<LaserPath> same_pen;
    for (const LaserPath& p : session_->raw.paths) {
        if (p.pen == best->pen) same_pen.push_back(p);
    }
    const std::vector<LaserPath> joined = join_paths(same_pen, PlanOptions{}.join_tol, nullptr);
    bool found = false;
    for (const LaserPath& p : joined) {
        if (p.id != best->id) continue;
        std::size_t k = 0;
        for (std::size_t i = 1; i < p.pts.size(); ++i) {
            if (std::hypot(p.pts[i].x - at.x, p.pts[i].y - at.y) < std::hypot(p.pts[k].x - at.x, p.pts[k].y - at.y)) {
                k = i;
            }
        }
        start_vertex_ = k;
        found = true;
        break;
    }
    if (!found) return ask_menu(State::OrderMenu, "error: cannot find that path again");
    start_id_ = best->id;
    state_ = State::OrderStartReverse;
    return ask_string(fmt("Path %d will start at %.3f,%.3f\nReverse [Yes/No] <No>", start_id_, at.x, at.y),
                      {"No", "Yes"});
}

Step LaserCommand::order_start_reverse(const InputValue& value) {
    const bool reverse = keyword_is(value, "YES");
    FixedStep* step = nullptr;
    for (FixedStep& s : session_->overrides.steps) {
        if (s.id == start_id_) step = &s;
    }
    if (!step) {
        FixedStep s;
        s.id = start_id_;
        s.free_order = true;  // a start is not a place in the order
        session_->overrides.steps.push_back(s);
        step = &session_->overrides.steps.back();
    }
    step->start = start_vertex_;
    step->reverse = reverse;
    return back(replan(fmt("Path %d starts at vertex %zu%s", start_id_, start_vertex_, reverse ? ", reversed" : "")));
}

// --- Save ---------------------------------------------------------------------------

bool resolve_laser_device(const LaserSession& session, const Database& db, DeviceConfig& cfg, std::string& error) {
    std::vector<DeviceConfig> devices;
    if (!load_devices(session.options.config_path, devices, error)) return false;
    const std::string& name = db.sysvars().get_string(session.dev);
    if (name.empty() && devices.empty()) {
        error = "no device: add one to lasers.conf and choose it with LASER Device";
        return false;
    }
    if (!select_device(devices, name, cfg, error)) return false;
    // Device Offset: a value set in this session that could not be saved.
    const auto it = session.pointer_offsets.find(cfg.name);
    if (it != session.pointer_offsets.end()) {
        cfg.pointer_offset_x = it->second.first;
        cfg.pointer_offset_y = it->second.second;
    }
    return true;
}

bool LaserCommand::resolve_device(CommandContext& ctx, DeviceConfig& cfg, std::string& error) const {
    return resolve_laser_device(*session_, ctx.db, cfg, error);
}

Step LaserCommand::save_pens(const std::string& path) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) return Step::failed("cannot write '" + path + "'");
    out << pen_lines(pens_);
    if (!out) return Step::failed("cannot write '" + path + "'");
    pens_dirty_ = false;
    return Step::done(fmt("Saved %zu pens to %s", pens_.size(), path.c_str()));
}

Step LaserCommand::save_job(CommandContext& ctx, const std::string& path) {
    const LaserJob& job = session_->job;
    std::string error;
    if (ends_with_ci(path, ".lbrn2")) {
        // LightBurn writes machine coordinates, so the file needs the machine's
        // origin corner: the chosen device's, else LBRN2CORNER.
        Lbrn2Options opts;
        DeviceConfig cfg;
        std::string ignore;
        std::string corner = ctx.db.sysvars().get_string(session_->lbrn2_corner);
        if (resolve_device(ctx, cfg, ignore)) {
            if (!cfg.origin_corner.empty()) corner = cfg.origin_corner;
            opts.machine.flip_x = cfg.flip_x;
            opts.machine.flip_y = cfg.flip_y;
            opts.machine.bed_w_mm = cfg.bed_w_mm;
            opts.machine.bed_h_mm = cfg.bed_h_mm;
        }
        if (!ruida::parse_machine_corner(corner, opts.machine.origin)) {
            return Step::failed("LBRN2CORNER must be bl, br, tl or tr, not '" + corner + "'");
        }
        opts.start = job.start;
        if (!write_lbrn2_file(path, job, opts, error)) return Step::failed(error);
        return Step::done("Saved " + path);
    }
    if (!ends_with_ci(path, ".rd")) {
        if (!write_ljob_file(path, job, error)) return Step::failed(error);
        return Step::done("Saved " + path);
    }

    // An .rd is encoded for one machine, so the chosen device supplies its
    // origin corner, flips and magic. With none chosen the defaults are
    // LightBurn's bottom-left machine and the common swizzle, and the message
    // says so rather than leaving the file's frame a surprise.
    DeviceConfig cfg;
    std::string plain, note;
    const bool have_device = resolve_device(ctx, cfg, error);
    if (have_device) {
        // Same power ceiling as Send: a file is a job waiting to be run.
        if (!prepare_send(job, cfg, plain, error)) return Step::failed(error);
    } else {
        note = " (no device chosen: bottom-left machine, magic 0x88)";
        cfg = DeviceConfig{};
        ruida::RdJob rd;
        if (!job_to_rd(job, rd, error) || !ruida::encode_rd(rd, plain, error)) return Step::failed(error);
    }
    const std::string data = ruida::Swizzle(cfg.magic).encode(plain);
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) return Step::failed("cannot write '" + path + "'");
    out << data;
    if (!out) return Step::failed("cannot write '" + path + "'");
    return Step::done(fmt("Saved %s (%zu bytes)", path.c_str(), data.size()) + note);
}

// --- Send and the other device operations ------------------------------------------------

Step LaserCommand::begin_send(CommandContext& ctx) {
    if (!session_->has_job) return Step::failed("no job yet: run LASER Job first");
    DeviceConfig cfg;
    std::string error, plain;
    if (!resolve_device(ctx, cfg, error)) return Step::failed(error);
    // The same checks Send will make, now, so a job that would be refused is
    // refused before anyone is asked to confirm it.
    if (!prepare_send(session_->job, cfg, plain, error) || !fits_bed(session_->job, cfg, error)) {
        return Step::failed(error);
    }

    if (ctx.db.sysvars().get_int(session_->confirm) == 0) return run_device(ctx, DeviceOp::Send);

    state_ = State::SendConfirm;
    Prompt p;
    p.kind = PromptKind::String;
    p.message = job_summary(session_->job, cfg, plain.size()) +
                fmt("Cut %.1f mm, estimated %s.\nRun it on %s now? [Yes/No] <No>", session_->plan.cut_mm,
                    minutes(session_->plan.estimated_s).c_str(), cfg.name.c_str());
    p.keywords = {"No", "Yes"};
    p.allow_empty = true;
    return Step::ask(p);
}

// Pulse FIRES THE LASER, so it is validated and confirmed like Send: the
// power ceiling is checked before anyone is asked, the answer defaults to No.
Step LaserCommand::begin_pulse(CommandContext& ctx) {
    DeviceConfig cfg;
    std::string error, plain;
    if (!resolve_device(ctx, cfg, error)) return Step::failed(error);
    if (!session_->options.connector) {
        return Step::failed("this build has no network support (NLASER_WITH_NET is off)");
    }
    state_ = State::PulsePower;
    Prompt p;
    p.kind = PromptKind::Real;
    p.message = "Pulse power % (this FIRES THE LASER on " + cfg.name + ")";
    return Step::ask(p);
}

// One line per poll until the head has been still and the state idle for about
// a second, or watch_s_ seconds have passed. Time is counted in polls (the
// interval each), so it is the same under a fake sleep.
Step LaserCommand::watch_device(LaserTransport& t, const std::string& name) {
    const int interval = std::max(50, session_->options.watch_interval_ms);
    const int quiet_needed = 1000;  // ms of still head and idle state
    std::string out = "Watching " + name + " for up to " + std::to_string(watch_s_) + " s\n";
    int quiet_ms = 0;
    bool have_last = false;
    double lx = 0.0, ly = 0.0;
    std::string why = "time is up";
    for (long at = 0; at <= static_cast<long>(watch_s_) * 1000; at += interval) {
        if (at > 0) {
            if (session_->options.sleep_ms) session_->options.sleep_ms(interval);
            else std::this_thread::sleep_for(std::chrono::milliseconds(interval));
        }
        MachineStatus s;
        std::string error;
        out += fmt("%7.1f s  ", static_cast<double>(at) / 1000.0);
        if (!t.poll(s, error)) {
            out += "no reply (" + error + ")\n";
            quiet_ms = 0;
            have_last = false;
            continue;
        }
        if (s.has_state) {
            out += fmt("state 0x%llX", static_cast<unsigned long long>(s.state));
            if (const char* w = state_seen(s.state)) out += std::string(" (") + w + ")";
        } else {
            out += "state n/a";
        }
        if (s.has_position) out += fmt("  head %.3f, %.3f mm", s.x_mm, s.y_mm);
        else out += "  head n/a";
        out += s.has_progress ? "  file " + reading(s.file_progress) + "/" + reading(s.file_total) : "  file n/a";
        out += "\n";

        const bool still = s.has_position && have_last && s.x_mm == lx && s.y_mm == ly;
        if (s.has_position) {
            lx = s.x_mm;
            ly = s.y_mm;
            have_last = true;
        } else {
            have_last = false;
        }
        if (still && s.has_state && state_is_idle(s.state)) quiet_ms += interval;
        else quiet_ms = 0;
        if (quiet_ms >= quiet_needed) {
            why = "head still and idle";
            break;
        }
    }
    out += "Watch ended: " + why;
    return Step::done(out);
}

Step LaserCommand::run_device(CommandContext& ctx, DeviceOp op) {
    DeviceConfig cfg;
    std::string error;
    if (!resolve_device(ctx, cfg, error)) return Step::failed(error);
    const bool needs_job = op == DeviceOp::Send || op == DeviceOp::Frame || op == DeviceOp::Dryrun;
    if (needs_job && !session_->has_job) return Step::failed("no job yet: run LASER Job first");

    std::string plain;
    if (op == DeviceOp::Send && !prepare_send(session_->job, cfg, plain, error)) return Step::failed(error);
    if (op == DeviceOp::Frame && !prepare_frame(session_->job, cfg, plain, error)) return Step::failed(error);
    if (op == DeviceOp::Dryrun && !prepare_dryrun(session_->job, cfg, plain, error)) return Step::failed(error);
    if (op == DeviceOp::Pulse) {
        double length = 0.0;
        if (!ruida::prepare_pulse(pulse_power_, pulse_ms_, cfg, plain, length, error)) return Step::failed(error);
    }
    if (needs_job && !fits_bed(session_->job, cfg, error)) return Step::failed(error);

    if (!session_->options.connector) {
        return Step::failed("this build has no network support (NLASER_WITH_NET is off)");
    }
    std::unique_ptr<LaserTransport> t;
    if (!session_->options.connector(cfg, t, error) || !t) return Step::failed(error);

    const std::string where = cfg.serial.empty() ? cfg.host : cfg.serial;

    // Everything below this line blocks on the controller, bounded by
    // cfg.ack_timeout_ms per wait. This is the seam for a worker thread: run
    // the transport calls there and finish the command from its completion.
    if (op == DeviceOp::Stop) {
        // Stop does not wait on the handshake: if ENQ fails, try anyway.
        std::string ignore;
        t->open(ignore);
        if (!t->stop(error)) return Step::failed("stop: " + error);
        return Step::done("Stop sent to " + where);
    }
    if (op == DeviceOp::Pause || op == DeviceOp::Resume) {
        // Like Stop: no waiting on the handshake. The codes (D8 02, D8 03) are
        // MeerK40t's and not yet verified on our controller.
        std::string ignore;
        t->open(ignore);
        const bool pausing = op == DeviceOp::Pause;
        if (!(pausing ? t->pause(error) : t->resume(error))) return Step::failed((pausing ? "pause: " : "resume: ") + error);
        return Step::done(std::string(pausing ? "Pause" : "Resume") + " sent to " + where +
                          " (code from MeerK40t, not yet verified on our controller)");
    }
    if (!t->open(error)) return Step::failed(where + ": " + error);

    switch (op) {
        case DeviceOp::Send:
            if (!t->send(session_->job, Progress{}, error)) return Step::failed(error);
            return Step::done(fmt("Sent %zu bytes to %s", plain.size(), cfg.name.c_str()) + with_note(*t));
        case DeviceOp::Frame:
            if (!t->frame(session_->job, error)) return Step::failed(error);
            return Step::done("Framing the job's bounding box on " + cfg.name + " (laser off)" + with_note(*t));
        case DeviceOp::Dryrun:
            if (!t->dryrun(session_->job, error)) return Step::failed(error);
            return Step::done("Tracing the whole planned path on " + cfg.name + " with the laser off (no cut command, power 0)" + with_note(*t));
        case DeviceOp::Status: {
            MachineStatus s;
            if (!t->status(s, error)) return Step::failed(error);
            std::string out = "Status of " + cfg.name + "\n";
            if (s.has_state) out += fmt("  state     %llu (0x%llX, raw)\n", static_cast<unsigned long long>(s.state), static_cast<unsigned long long>(s.state));
            if (s.has_position) out += fmt("  position  %.3f, %.3f mm\n", s.x_mm, s.y_mm);
            if (s.has_bed) out += fmt("  bed       %.3f x %.3f mm\n", s.bed_w_mm, s.bed_h_mm);
            if (s.has_progress) out += fmt("  file      %llu of %llu\n", static_cast<unsigned long long>(s.file_progress), static_cast<unsigned long long>(s.file_total));
            return Step::done(out);
        }
        case DeviceOp::Counters: {
            Counters k;
            if (!t->counters(k, error)) return Step::failed(error);
            auto field = [](bool has, std::uint64_t v) { return has ? std::to_string(v) : std::string("-"); };
            std::string out = "Counters of " + cfg.name + "\n";
            if (k.has_card_id) out += fmt("  card id          0x%08llX\n", static_cast<unsigned long long>(k.card_id));
            if (k.has_firmware) out += "  firmware         " + k.firmware + "\n";
            out += "  power-on time    " + field(k.has_power_on, k.power_on) + "\n";
            out += "  work time        " + field(k.has_work_time, k.work_time) + "\n";
            out += "  laser-on time    " + field(k.has_laser_on, k.laser_on) + "\n";
            out += "  jobs             " + field(k.has_jobs, k.jobs) + "\n";
            out += "  travel x         " + field(k.has_travel_x, k.travel_x) + "\n";
            out += "  travel y         " + field(k.has_travel_y, k.travel_y) + "\n";
            out += "  (raw values; units to be confirmed against the panel)";
            return Step::done(out);
        }
        case DeviceOp::Pulse:
            if (!t->pulse(pulse_power_, pulse_ms_, error)) return Step::failed(error);
            return Step::done(fmt("Fired %g%% for %d ms on %s", pulse_power_, pulse_ms_, cfg.name.c_str()));
        case DeviceOp::Watch: return watch_device(*t, cfg.name);
        case DeviceOp::Stop:
        case DeviceOp::Pause:
        case DeviceOp::Resume: break;  // handled above
    }
    return Step::failed("internal state error");
}

// --- the state machine ------------------------------------------------------------------

Step LaserCommand::next(CommandContext& ctx, const InputValue& value) {
    switch (state_) {
        case State::Top: return top(ctx, value);
        case State::DeviceName: return back(choose_device(ctx, value));
        case State::DeviceOffsetX: return offset_x(value);
        case State::DeviceOffsetY: return offset_y(ctx, value);
        case State::WatchSeconds:
            if (value.kind == InputKind::Integer) {
                if (value.integer < 1 || value.integer > 3600) return back(Step::failed("watch 1 to 3600 seconds"));
                watch_s_ = value.integer;
            } else if (value.kind != InputKind::None) {
                return back(Step::failed("a number of seconds is required"));
            }
            return back(run_device(ctx, DeviceOp::Watch));
        case State::PensMenu: return pens_menu(ctx, value);
        case State::PensLoadFile: return back(load_pens(ctx, value));
        case State::PensSetNumber:
        case State::PensSetSpeed:
        case State::PensSetPower:
        case State::PensSetPasses:
        case State::PensSetAir: return pens_set(ctx, value);
        case State::MoveMenu: return move_menu(ctx, value);
        case State::OrderMenu: return order_menu(value);
        case State::OrderPick: return order_pick(value);
        case State::OrderStartPoint: return order_start_point(value);
        case State::OrderStartReverse: return order_start_reverse(value);

        case State::MoveGoto: {
            if (value.kind != InputKind::Point) return ask_menu(State::MoveMenu, "error: a point is required");
            std::string error;
            if (!ensure_move_target(ctx, error)) return ask_menu(State::MoveMenu, "error: " + error);
            if (default_move_) default_move_->set_frame(session_->start, job_origin(ctx));
            if (!move_target_->goto_point(value.point.x, value.point.y, error)) {
                return ask_menu(State::MoveMenu, "error: " + error);
            }
            return ask_menu(State::MoveMenu,
                            fmt("moved to %.3f,%.3f", value.point.x, value.point.y) + report_suffix());
        }

        case State::MoveStep:
            if (value.kind == InputKind::None) return ask_menu(State::MoveMenu, "");
            if (value.kind != InputKind::Real || !(value.real > 0.0) || !std::isfinite(value.real)) {
                return ask_menu(State::MoveMenu, "error: the step is a positive number of mm");
            }
            move_step_ = value.real;
            return ask_menu(State::MoveMenu, "");

        case State::PulsePower:
            if (value.kind != InputKind::Real) return back(Step::failed("a power in percent is required"));
            pulse_power_ = value.real;
            state_ = State::PulseDuration;
            return ask_number(PromptKind::Integer, fmt("Duration ms <%d>", pulse_ms_));

        case State::PulseDuration: {
            if (value.kind == InputKind::Integer) pulse_ms_ = value.integer;
            else if (value.kind != InputKind::None) return back(Step::failed("a duration in ms is required"));
            DeviceConfig cfg;
            std::string error, plain;
            double length = 0.0;
            if (!resolve_device(ctx, cfg, error) ||
                !ruida::prepare_pulse(pulse_power_, pulse_ms_, cfg, plain, length, error)) {
                return back(Step::failed(error));
            }
            if (ctx.db.sysvars().get_int(session_->confirm) == 0) return back(run_device(ctx, DeviceOp::Pulse));
            state_ = State::PulseConfirm;
            return ask_string(fmt("Fire %g%% for %d ms on %s (the head moves %.2f mm)? [Yes/No] <No>", pulse_power_,
                                  pulse_ms_, cfg.name.c_str(), length),
                              {"No", "Yes"});
        }

        case State::PulseConfirm:
            if (!keyword_is(value, "YES")) return back(Step::done("Not fired"));
            return back(run_device(ctx, DeviceOp::Pulse));

        case State::JobArea:
            if (value.kind == InputKind::None || keyword_is(value, "EXTENTS")) {
                area_ = PlotArea::Extents;
            } else if (keyword_is(value, "DISPLAY")) {
                area_ = PlotArea::Display;
            } else if (keyword_is(value, "LIMITS")) {
                area_ = PlotArea::Limits;
            } else if (keyword_is(value, "WINDOW")) {
                area_ = PlotArea::Window;
                state_ = State::JobWindowFirst;
                Prompt p;
                p.kind = PromptKind::Point;
                p.message = "First corner";
                return Step::ask(p);
            } else {
                return back(Step::failed("unknown job area"));
            }
            return back(build_job(ctx));

        case State::JobWindowFirst: {
            if (value.kind != InputKind::Point) return back(Step::failed("a point is required"));
            corner_ = value.point;
            state_ = State::JobWindowSecond;
            Prompt p;
            p.kind = PromptKind::Point;
            p.message = "Other corner";
            p.base = corner_;
            p.has_base = true;
            p.rubber_band = RubberBand::Box;
            return Step::ask(p);
        }

        case State::JobWindowSecond: {
            if (value.kind != InputKind::Point) return back(Step::failed("a point is required"));
            BBox box;
            box.expand(corner_);
            box.expand(value.point);
            view_ = plot_view_for_box(box, ctx.db.construction_normal());
            return back(build_job(ctx));
        }

        case State::OriginStart: {
            if (value.kind != InputKind::None) {
                if (keyword_is(value, "CURRENT")) session_->start = StartMode::Current;
                else if (keyword_is(value, "ABSOLUTE")) session_->start = StartMode::Absolute;
                else if (keyword_is(value, "USER")) session_->start = StartMode::User;
                else return back(Step::failed("Start from: Current, Absolute or User"));
            }
            if (session_->has_job) session_->job.start = session_->start;
            return back(Step::done(origin_note_ + fmt("; start from %s", session_->start == StartMode::Absolute
                                                                              ? "absolute machine coordinates"
                                                                          : session_->start == StartMode::User
                                                                              ? "the user origin"
                                                                              : "the head's current position")));
        }

        case State::SaveFile: {
            if (value.kind != InputKind::String || value.text.empty()) return back(Step::failed("a file name is required"));
            std::string path = expand_user_path(value.text);
            if (!has_extension(path)) path += menu_ == State::PensMenu ? ".lpt" : ".ljob";
            pending_ = path;
            if (path_exists(path)) {
                state_ = State::SaveOverwrite;
                Prompt p;
                p.kind = PromptKind::String;
                p.message = path + " exists. Overwrite? [Yes/No] <No>";
                p.file_overwrite = true;
                p.keywords = {"No", "Yes"};
                p.allow_empty = true;
                return Step::ask(p);
            }
            break;
        }

        case State::SaveOverwrite:
            if (!keyword_is(value, "YES")) return back(Step::done("Not saved"));
            break;

        case State::SendConfirm:
            // Default No: the answer that moves nothing is the one Enter gives.
            if (!keyword_is(value, "YES")) return back(Step::done("Not sent"));
            return back(run_device(ctx, DeviceOp::Send));
    }

    // Only the two Save states fall out of the switch.
    return back(menu_ == State::PensMenu ? save_pens(pending_) : save_job(ctx, pending_));
}

}  // namespace ncad::laser
