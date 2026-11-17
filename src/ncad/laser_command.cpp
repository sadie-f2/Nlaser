// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

#include "ncad/laser/laser_command.hpp"

#include "ncad/commands.hpp"
#include "ncad/database.hpp"
#include "ncad/laser/device_config.hpp"
#include "ncad/laser/ruida_codec.hpp"
#include "ncad/laser/ruida_encode.hpp"
#include "ncad/laser/ruida_job.hpp"
#include "ncad/laser/ruida_transport.hpp"
#include "ncad/paths.hpp"

#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <fstream>

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
             job.start == StartMode::Absolute ? "absolute machine coordinates" : "placed at the head's current position",
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

    if (!register_command("LASER", [session] { return std::make_unique<LaserCommand>(session); })) {
        return nullptr;
    }
    return session;
}

// --- prompts --------------------------------------------------------------------

Step LaserCommand::ask_top() const {
    Prompt p;
    p.kind = PromptKind::String;
    p.message = "Laser [DEvice/Pens/Origin/Job/Order/Save/Send/Frame/DRyrun/Status/Counters/STop] <Job>";
    p.keywords = {"Device", "Pens", "Origin", "Job", "Order", "Save", "Send", "Frame", "DRyrun", "Status", "Counters", "STop"};
    p.allow_empty = true;
    return Step::ask(p);
}

Step LaserCommand::start(CommandContext&) {
    if (!session_) return Step::failed("the laser backend is not installed");
    state_ = State::Top;
    return ask_top();
}

Step LaserCommand::top(CommandContext& ctx, const InputValue& value) {
    if (value.kind == InputKind::None || keyword_is(value, "JOB")) {
        state_ = State::JobArea;
        Prompt p;
        p.kind = PromptKind::String;
        p.message = "Job area [Display/Extents/Limits/Window] <Display>";
        p.keywords = {"Display", "Extents", "Limits", "Window"};
        p.allow_empty = true;
        return Step::ask(p);
    }

    if (keyword_is(value, "DEVICE")) {
        std::vector<DeviceConfig> devices;
        std::string error;
        if (!load_devices(session_->options.config_path, devices, error)) return Step::failed(error);
        const std::string& current = ctx.db.sysvars().get_string(session_->dev);
        state_ = State::DeviceName;
        Prompt p;
        p.kind = PromptKind::String;
        p.message = "Device";
        if (devices.empty()) {
            p.message += " (none in lasers.conf)";
        } else {
            p.message += " (" + device_names(devices) + ")";
        }
        if (!current.empty()) p.message += " <" + current + ">";
        p.allow_empty = true;
        return Step::ask(p);
    }

    if (keyword_is(value, "PENS")) {
        const std::string& current = ctx.db.sysvars().get_string(session_->pens);
        state_ = State::PensFile;
        Prompt p;
        p.kind = PromptKind::String;
        p.message = "Pen table file (.lpt, . for none)";
        if (!current.empty()) p.message += " <" + current + ">";
        p.file = FileIntent::Open;
        p.file_extension = "lpt";
        p.allow_empty = true;
        return Step::ask(p);
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
        Prompt p;
        p.kind = PromptKind::String;
        p.message = s + "\nStart from [Current/Absolute] <" +
                    (session_->start == StartMode::Absolute ? "Absolute" : "Current") + ">";
        p.keywords = {"Current", "Absolute"};
        p.allow_empty = true;
        return Step::ask(p);
    }

    if (keyword_is(value, "ORDER")) {
        if (!session_->has_job) return Step::failed("no job yet: run LASER Job first");
        state_ = State::OrderOption;
        Prompt p;
        p.kind = PromptKind::String;
        // Honest about what is built: the designer overrides are the part of
        // the spec that comes later, and a prompt that offered them would be a
        // promise.
        p.message = "Order [List] <List> (Show, Pick and Start come later)";
        p.keywords = {"List"};
        p.allow_empty = true;
        return Step::ask(p);
    }

    if (keyword_is(value, "SAVE")) {
        if (!session_->has_job) return Step::failed("no job yet: run LASER Job first");
        state_ = State::SaveFile;
        Prompt p;
        p.kind = PromptKind::String;
        p.message = "Save job as (.ljob or .rd)";
        p.file = FileIntent::Save;
        p.file_extension = "ljob";
        return Step::ask(p);
    }

    if (keyword_is(value, "SEND")) return begin_send(ctx);
    if (keyword_is(value, "FRAME")) return run_device(ctx, DeviceOp::Frame);
    if (keyword_is(value, "DRYRUN")) return run_device(ctx, DeviceOp::Dryrun);
    if (keyword_is(value, "STATUS")) return run_device(ctx, DeviceOp::Status);
    if (keyword_is(value, "COUNTERS")) return run_device(ctx, DeviceOp::Counters);
    if (keyword_is(value, "STOP")) return run_device(ctx, DeviceOp::Stop);

    return Step::failed("unknown LASER option");
}

// --- Device and Pens ----------------------------------------------------------------

Step LaserCommand::choose_device(CommandContext& ctx, const InputValue& value) {
    std::vector<DeviceConfig> devices;
    std::string error;
    if (!load_devices(session_->options.config_path, devices, error)) return Step::failed(error);

    if (value.kind == InputKind::None) {
        const std::string& cur = ctx.db.sysvars().get_string(session_->dev);
        return Step::done(cur.empty() ? "No device chosen" : "Device " + cur);
    }
    if (value.kind != InputKind::String || value.text.empty()) return Step::failed("a device name is required");

    DeviceConfig cfg;
    if (!find_device(devices, value.text, cfg)) {
        return Step::failed("no device '" + value.text + "' in lasers.conf" +
                            (devices.empty() ? "" : " (" + device_names(devices) + ")"));
    }
    ctx.db.sysvars().set_string(session_->dev, cfg.name);
    std::string where = cfg.serial.empty() ? cfg.host : cfg.serial;
    return Step::done("Device " + cfg.name + (where.empty() ? "" : " at " + where));
}

Step LaserCommand::choose_pens(CommandContext& ctx, const InputValue& value) {
    if (value.kind == InputKind::None) {
        const std::string& cur = ctx.db.sysvars().get_string(session_->pens);
        return Step::done(cur.empty() ? "No pen table: every pen runs at power 0" : "Pen table " + cur);
    }
    if (value.kind != InputKind::String || value.text.empty()) return Step::failed("a file name is required");

    if (value.text == ".") {
        ctx.db.sysvars().set_string(session_->pens, "");
        return Step::done("No pen table");
    }
    const std::string path = ensure_extension(expand_user_path(value.text), ".lpt");
    std::vector<LaserLayer> pens;
    std::string error;
    // Read now rather than at Job, so a typo is reported where it was typed.
    if (!read_pen_table_file(path, pens, error)) return Step::failed(error);
    ctx.db.sysvars().set_string(session_->pens, path);
    return Step::done(fmt("Pen table %s (%zu pens)", path.c_str(), pens.size()));
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

    const double tol = ctx.db.sysvars().get_real(session_->tol);
    if (!(tol > 0.0) || !std::isfinite(tol)) return Step::failed("LASERTOL must be a positive number of mm");

    DrawingOptions opts;
    opts.chord_tolerance = tol;
    opts.has_view = use_view;
    opts.view = view_;

    std::vector<LaserLayer> pens;
    std::string error;
    const std::string& pens_path = ctx.db.sysvars().get_string(session_->pens);
    if (!pens_path.empty() && !read_pen_table_file(pens_path, pens, error)) return Step::failed(error);

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

    LaserJob planned;
    PlanReport plan;
    if (!plan_job(raw, planned, PlanOptions{}, plan, error)) return Step::failed(error);

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
    s += fmt("Origin %.3f,%.3f (%s)\n", job.origin.x, job.origin.y, report.laser_ucs ? "UCS LASER" : "world");
    s += fmt("Cut %.1f mm, travel %.1f mm, estimated %s", plan.cut_mm, plan.travel_mm, minutes(plan.estimated_s).c_str());
    for (const std::string& w : report.warnings) s += "\nwarning: " + w;
    return Step::done(s);
}

Step LaserCommand::order_list() const {
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
    }
    return Step::done(s);
}

// --- Save ---------------------------------------------------------------------------

bool LaserCommand::resolve_device(CommandContext& ctx, DeviceConfig& cfg, std::string& error) const {
    std::vector<DeviceConfig> devices;
    if (!load_devices(session_->options.config_path, devices, error)) return false;
    const std::string& name = ctx.db.sysvars().get_string(session_->dev);
    if (name.empty() && devices.empty()) {
        error = "no device: add one to lasers.conf and choose it with LASER Device";
        return false;
    }
    return select_device(devices, name, cfg, error);
}

Step LaserCommand::save_job(CommandContext& ctx, const std::string& path) {
    const LaserJob& job = session_->job;
    std::string error;
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
    if (!t->open(error)) return Step::failed(where + ": " + error);

    switch (op) {
        case DeviceOp::Send:
            if (!t->send(session_->job, Progress{}, error)) return Step::failed(error);
            return Step::done(fmt("Sent %zu bytes to %s", plain.size(), cfg.name.c_str()));
        case DeviceOp::Frame:
            if (!t->frame(session_->job, error)) return Step::failed(error);
            return Step::done("Framing the job's bounding box on " + cfg.name + " (laser off)");
        case DeviceOp::Dryrun:
            if (!t->dryrun(session_->job, error)) return Step::failed(error);
            return Step::done("Tracing the whole planned path on " + cfg.name + " with the laser off (no cut command, power 0)");
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
        case DeviceOp::Stop: break;  // handled above
    }
    return Step::failed("internal state error");
}

// --- the state machine ------------------------------------------------------------------

Step LaserCommand::next(CommandContext& ctx, const InputValue& value) {
    switch (state_) {
        case State::Top: return top(ctx, value);
        case State::DeviceName: return choose_device(ctx, value);
        case State::PensFile: return choose_pens(ctx, value);

        case State::JobArea:
            if (value.kind == InputKind::None || keyword_is(value, "DISPLAY")) {
                area_ = PlotArea::Display;
            } else if (keyword_is(value, "EXTENTS")) {
                area_ = PlotArea::Extents;
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
                return Step::failed("unknown job area");
            }
            return build_job(ctx);

        case State::JobWindowFirst: {
            if (value.kind != InputKind::Point) return Step::failed("a point is required");
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
            if (value.kind != InputKind::Point) return Step::failed("a point is required");
            BBox box;
            box.expand(corner_);
            box.expand(value.point);
            view_ = plot_view_for_box(box, ctx.db.construction_normal());
            return build_job(ctx);
        }

        case State::OriginStart: {
            if (value.kind != InputKind::None) {
                if (keyword_is(value, "CURRENT")) session_->start = StartMode::Current;
                else if (keyword_is(value, "ABSOLUTE")) session_->start = StartMode::Absolute;
                else return Step::failed("Start from: Current or Absolute");
            }
            if (session_->has_job) session_->job.start = session_->start;
            return Step::done(origin_note_ + fmt("; start from %s", session_->start == StartMode::Absolute
                                                                         ? "absolute machine coordinates"
                                                                         : "the head's current position"));
        }

        case State::OrderOption:
            if (value.kind == InputKind::None || keyword_is(value, "LIST")) return order_list();
            return Step::failed("only Order List is built; Show, Pick and Start come later");

        case State::SaveFile: {
            if (value.kind != InputKind::String || value.text.empty()) return Step::failed("a file name is required");
            std::string path = expand_user_path(value.text);
            if (!has_extension(path)) path += ".ljob";
            if (path_exists(path)) {
                pending_ = path;
                state_ = State::SaveOverwrite;
                Prompt p;
                p.kind = PromptKind::String;
                p.message = path + " exists. Overwrite? [Yes/No] <No>";
                p.file_overwrite = true;
                p.keywords = {"No", "Yes"};
                p.allow_empty = true;
                return Step::ask(p);
            }
            pending_ = path;
            break;
        }

        case State::SaveOverwrite:
            if (!keyword_is(value, "YES")) return Step::done("Not saved");
            break;

        case State::SendConfirm:
            // Default No: the answer that moves nothing is the one Enter gives.
            if (!keyword_is(value, "YES")) return Step::done("Not sent");
            return run_device(ctx, DeviceOp::Send);
    }

    // Only the two Save states fall out of the switch.
    return save_job(ctx, pending_);
}

}  // namespace ncad::laser
