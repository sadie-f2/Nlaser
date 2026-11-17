// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

#include "ncad/laser/lbrn2.hpp"

#include "ncad/laser/xml.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>

namespace ncad::laser {

bool parse_job_origin(const std::string& s, JobOrigin& out) {
    if (s == "bl") out = JobOrigin::BottomLeft;
    else if (s == "br") out = JobOrigin::BottomRight;
    else if (s == "tl") out = JobOrigin::TopLeft;
    else if (s == "tr") out = JobOrigin::TopRight;
    else if (s == "c") out = JobOrigin::Centre;
    else return false;
    return true;
}

// The one mapping the samples show: LightBurn layer 2 is red in the .rd
// ("Layer Color #FF0000"), which is ACI 1. No other index is evidenced, so
// the rest are 7 rather than a guess from memory of LightBurn's palette.
int lbrn2_index_to_aci(int index) { return index == 2 ? 1 : 7; }

namespace {

constexpr double kPi = 3.14159265358979323846;

// --- frame ---------------------------------------------------------------------

// Workspace (machine) coordinates to design, and back. With a bed, the machine's
// own bed arithmetic; without one, the sign change alone.
Pt to_design(const ruida::RdMachine& m, Pt w) {
    if (m.has_bed()) {
        Pt d;
        m.machine_to_bed(w.x, w.y, d.x, d.y);
        return d;
    }
    return {w.x * m.sign_x(), w.y * m.sign_y()};
}

Pt to_workspace(const ruida::RdMachine& m, Pt d) {
    if (m.has_bed()) {
        Pt w;
        m.bed_to_machine(d.x, d.y, w.x, w.y);
        return w;
    }
    return {d.x * m.sign_x(), d.y * m.sign_y()};
}

// --- numbers -------------------------------------------------------------------

bool to_double(const std::string& s, double& v) {
    if (s.empty()) return false;
    char* end = nullptr;
    v = std::strtod(s.c_str(), &end);
    return end && *end == '\0' && std::isfinite(v);
}

// Shortest decimal with up to five places, as LightBurn's own (557.27362).
std::string num(double v) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.5f", v);
    std::string s = buf;
    while (!s.empty() && s.back() == '0') s.pop_back();
    if (!s.empty() && s.back() == '.') s.pop_back();
    if (s == "-0" || s.empty()) s = "0";
    return s;
}

std::string at_line(const xml::Node& n, const std::string& what) {
    return "line " + std::to_string(n.line) + ": " + what;
}

// --- reading -------------------------------------------------------------------

struct Xf {
    double a{1}, b{0}, c{0}, d{1}, e{0}, f{0};
    // x' = a x + c y + e, y' = b x + d y + f. The samples only show a = d = -1
    // and b = c = 0; the placement of b and c is the usual a b c d e f order,
    // unverified (docs/lbrn2-format.md).
    Pt apply(double x, double y) const { return {a * x + c * y + e, b * x + d * y + f}; }
};

bool read_xform(const xml::Node& shape, Xf& xf, std::string& error) {
    const xml::Node* n = shape.child("XForm");
    if (!n) {
        error = at_line(shape, "<Shape> has no <XForm>");
        return false;
    }
    double v[6];
    const char* p = n->text.c_str();
    for (double& x : v) {
        char* end = nullptr;
        x = std::strtod(p, &end);
        if (end == p || !std::isfinite(x)) {
            error = at_line(*n, "<XForm> must hold six numbers");
            return false;
        }
        p = end;
    }
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') ++p;
    if (*p) {
        error = at_line(*n, "<XForm> must hold exactly six numbers");
        return false;
    }
    xf = {v[0], v[1], v[2], v[3], v[4], v[5]};
    return true;
}

struct Vert {
    double x{0}, y{0};
    bool has[4]{false, false, false, false};  // c0x c0y c1x c1y
    double c[4]{0, 0, 0, 0};
};

// "V0 0c0x1c1x1V40 0..." : V x y, then optional c0x/c0y/c1x/c1y values.
bool parse_verts(const xml::Node& n, std::vector<Vert>& out, std::string& error) {
    const std::string& s = n.text;
    std::size_t i = 0;
    auto number = [&](double& v) {
        const std::size_t b = i;
        while (i < s.size() && (std::isdigit(static_cast<unsigned char>(s[i])) || s[i] == '-' || s[i] == '+' ||
                                s[i] == '.' || s[i] == 'e' || s[i] == 'E')) {
            ++i;
        }
        return to_double(s.substr(b, i - b), v);
    };
    while (i < s.size()) {
        if (s[i] == ' ' || s[i] == '\n' || s[i] == '\r' || s[i] == '\t') {
            ++i;
            continue;
        }
        if (s[i] != 'V') {
            error = at_line(n, "<VertList> expected 'V' at offset " + std::to_string(i));
            return false;
        }
        ++i;
        Vert v;
        if (!number(v.x) || i >= s.size() || s[i] != ' ') {
            error = at_line(n, "<VertList> vertex without two coordinates at offset " + std::to_string(i));
            return false;
        }
        ++i;
        if (!number(v.y)) {
            error = at_line(n, "<VertList> vertex without two coordinates at offset " + std::to_string(i));
            return false;
        }
        while (i < s.size() && s[i] == 'c') {
            if (i + 2 >= s.size() || (s[i + 1] != '0' && s[i + 1] != '1') || (s[i + 2] != 'x' && s[i + 2] != 'y')) {
                error = at_line(n, "<VertList> unknown control field at offset " + std::to_string(i));
                return false;
            }
            const int k = (s[i + 1] - '0') * 2 + (s[i + 2] == 'y' ? 1 : 0);
            i += 3;
            if (!number(v.c[k])) {
                error = at_line(n, "<VertList> control field without a number");
                return false;
            }
            v.has[k] = true;
        }
        out.push_back(v);
    }
    return true;
}

struct Prim {
    char kind{'L'};
    int a{0}, b{0};
};

// "LineClosed", or explicit "L<a> <b>" / "B<a> <b>" primitives. Only
// LineClosed is in the samples; L and B are LightBurn's spelling as we know
// it and are unverified. `closed_all` is set for LineClosed.
bool parse_prims(const xml::Node& n, std::size_t nverts, std::vector<Prim>& out, bool& closed_all,
                 std::string& error) {
    const std::string& s = n.text;
    closed_all = false;
    if (s == "LineClosed") {
        closed_all = true;
        return true;
    }
    std::size_t i = 0;
    auto integer = [&](int& v) {
        const std::size_t b = i;
        while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i]))) ++i;
        if (i == b || i - b > 9) return false;
        v = std::atoi(s.substr(b, i - b).c_str());
        return true;
    };
    while (i < s.size()) {
        if (s[i] == ' ' || s[i] == '\n' || s[i] == '\r' || s[i] == '\t') {
            ++i;
            continue;
        }
        if (s[i] != 'L' && s[i] != 'B') {
            error = at_line(n, "<PrimList> primitive '" + s.substr(i, 12) + "' is not supported (only LineClosed, L and B)");
            return false;
        }
        Prim p;
        p.kind = s[i++];
        if (!integer(p.a) || i >= s.size() || s[i] != ' ') {
            error = at_line(n, "<PrimList> primitive without two vertex indices");
            return false;
        }
        ++i;
        if (!integer(p.b)) {
            error = at_line(n, "<PrimList> primitive without two vertex indices");
            return false;
        }
        if (static_cast<std::size_t>(p.a) >= nverts || static_cast<std::size_t>(p.b) >= nverts) {
            error = at_line(n, "<PrimList> refers to a vertex that is not in <VertList>");
            return false;
        }
        out.push_back(p);
    }
    return true;
}

void flatten_cubic(Pt p0, Pt c0, Pt c1, Pt p1, double tol, int depth, std::vector<Pt>& out) {
    // Distance of a control point from the chord bounds the curve's deviation.
    auto dist = [&](Pt q) {
        const double dx = p1.x - p0.x, dy = p1.y - p0.y;
        const double len = std::hypot(dx, dy);
        if (len < 1e-12) return std::hypot(q.x - p0.x, q.y - p0.y);
        return std::fabs((q.x - p0.x) * dy - (q.y - p0.y) * dx) / len;
    };
    if (depth >= 18 || std::max(dist(c0), dist(c1)) <= tol) {
        out.push_back(p1);
        return;
    }
    auto mid = [](Pt a, Pt b) { return Pt{(a.x + b.x) / 2, (a.y + b.y) / 2}; };
    const Pt a = mid(p0, c0), b = mid(c0, c1), c = mid(c1, p1);
    const Pt ab = mid(a, b), bc = mid(b, c);
    const Pt m = mid(ab, bc);
    flatten_cubic(p0, a, ab, m, tol, depth + 1, out);
    flatten_cubic(m, bc, c, p1, tol, depth + 1, out);
}

struct Reader {
    const Lbrn2Options& opts;
    LaserJob job;
    Lbrn2Info info;
    std::string error;
    bool have_ws{false};

    explicit Reader(const Lbrn2Options& o) : opts(o) {}

    bool fail(const xml::Node& n, const std::string& what) {
        error = at_line(n, what);
        return false;
    }

    void note_ws(Pt w) {
        if (!have_ws) {
            info.workspace_min = info.workspace_max = w;
            have_ws = true;
        }
        info.workspace_min.x = std::min(info.workspace_min.x, w.x);
        info.workspace_min.y = std::min(info.workspace_min.y, w.y);
        info.workspace_max.x = std::max(info.workspace_max.x, w.x);
        info.workspace_max.y = std::max(info.workspace_max.y, w.y);
    }

    bool value(const xml::Node& cut, const char* tag, double& v, bool& present) {
        present = false;
        const xml::Node* n = cut.child(tag);
        if (!n) return true;
        const std::string* s = n->attr("Value");
        if (!s || !to_double(*s, v)) return fail(*n, std::string("<") + tag + "> needs a numeric Value");
        present = true;
        return true;
    }

    bool cut_setting(const xml::Node& cut) {
        const std::string* type = cut.attr("type");
        if (!type || *type != "Cut") {
            return fail(cut, "<CutSetting type=\"" + (type ? *type : std::string()) +
                                 "\"> is not supported (only Cut; no sample shows another mode)");
        }
        LaserLayer l;
        l.mode = LayerMode::Vector;
        double v = 0.0;
        bool has = false;
        if (!value(cut, "index", v, has)) return false;
        if (!has || v < 0 || v != std::floor(v) || v > 1000) return fail(cut, "<CutSetting> needs an integer <index>");
        l.pen = static_cast<int>(v);
        l.color = lbrn2_index_to_aci(l.pen);
        if (job.layer(l.pen)) return fail(cut, "layer index " + std::to_string(l.pen) + " is defined twice");
        if (!value(cut, "speed", l.speed, has)) return false;
        if (!has || !(l.speed > 0)) return fail(cut, "<speed> must be given and positive");
        double p1min = 0, p1max = 0, p2min = 0, p2max = 0;
        bool h1, h2, h3, h4;
        if (!value(cut, "minPower", p1min, h1) || !value(cut, "maxPower", p1max, h2) ||
            !value(cut, "minPower2", p2min, h3) || !value(cut, "maxPower2", p2max, h4)) {
            return false;
        }
        if (!h1 || !h2) return fail(cut, "<minPower> and <maxPower> must be given");
        for (double p : {p1min, p1max, p2min, p2max}) {
            if (p < 0 || p > 100) return fail(cut, "a power outside 0..100");
        }
        l.min_power = p1min;
        l.max_power = p1max;
        // Tube 2 is always written; it is "as tube 1" when it equals it.
        if (h3 && h4 && (p2min != p1min || p2max != p1max)) {
            l.has_power2 = true;
            l.min_power2 = p2min;
            l.max_power2 = p2max;
        }
        bool has_passes = false;
        double passes = 1;
        if (!value(cut, "numPasses", passes, has_passes)) return false;
        if (has_passes) {
            if (passes < 1 || passes != std::floor(passes) || passes > 1000) {
                return fail(cut, "<numPasses> must be a whole number from 1");
            }
            l.passes = static_cast<int>(passes);
        }
        double prio = 0;
        bool has_prio = false;
        if (!value(cut, "priority", prio, has_prio)) return false;
        if (has_prio) l.order = static_cast<int>(prio);
        // Air: no element in any sample, and the .rd says "Air Assist On", so
        // absent means on. How LightBurn spells off is not known.
        l.air = true;
        // Output off: not in the samples. LightBurn's name for it is believed to
        // be doOutput (unverified). If that is wrong the layer is simply cut as
        // an ordinary one, so a project with a disabled layer must be checked by
        // eye until a sample confirms the name.
        if (const xml::Node* o = cut.child("doOutput")) {
            const std::string* s = o->attr("Value");
            if (s && *s == "0") l.mode = LayerMode::Skip;
        }
        static const char* const kIgnored[] = {"index",   "name",    "minPower",  "maxPower", "minPower2",
                                               "maxPower2", "speed", "PPI",       "numPasses", "perfLen",
                                               "perfSkip", "dotTime", "scanOpt", "overscan", "interval",
                                               "priority", "tabSize", "tabCount", "tabCountMax", "tabSpacing",
                                               "doOutput"};
        for (const xml::Node& c : cut.children) {
            if (std::find_if(std::begin(kIgnored), std::end(kIgnored), [&](const char* k) { return c.name == k; }) ==
                std::end(kIgnored)) {
                info.warnings.push_back(at_line(c, "layer " + std::to_string(l.pen) + ": <" + c.name + "> is not understood and was ignored"));
            }
        }
        std::string name;
        if (const xml::Node* nn = cut.child("name")) {
            if (const std::string* s = nn->attr("Value")) name = *s;
        }
        info.layer_names.emplace_back(l.pen, name);
        job.layers.push_back(l);
        return true;
    }

    // `ws` are workspace points; the path goes to the design frame.
    void add_path(int pen, bool closed, const std::vector<Pt>& ws) {
        LaserPath p;
        p.pen = pen;
        p.closed = closed;
        for (const Pt& w : ws) {
            note_ws(w);
            p.pts.push_back(to_design(opts.machine, w));
        }
        job.paths.push_back(std::move(p));
    }

    bool path_shape(const xml::Node& shape, int pen, const Xf& xf) {
        const xml::Node* vl = shape.child("VertList");
        const xml::Node* pl = shape.child("PrimList");
        if (!vl || !pl) return fail(shape, "a Path needs <VertList> and <PrimList>");
        std::vector<Vert> verts;
        if (!parse_verts(*vl, verts, error)) return false;
        std::vector<Prim> prims;
        bool closed_all = false;
        if (!parse_prims(*pl, verts.size(), prims, closed_all, error)) return false;
        if (verts.size() < 2) return fail(*vl, "a Path needs at least two vertices");
        auto ws = [&](const Vert& v) { return xf.apply(v.x, v.y); };
        if (closed_all) {
            std::vector<Pt> pts;
            for (const Vert& v : verts) pts.push_back(ws(v));
            add_path(pen, true, pts);
            return true;
        }
        // Chains of primitives that meet end to start become one path each.
        std::size_t k = 0;
        while (k < prims.size()) {
            const int first = prims[k].a;
            std::vector<Pt> pts{ws(verts[static_cast<std::size_t>(first)])};
            int cur = first;
            while (k < prims.size() && prims[k].a == cur) {
                const Prim& p = prims[k];
                const Vert& va = verts[static_cast<std::size_t>(p.a)];
                const Vert& vb = verts[static_cast<std::size_t>(p.b)];
                if (p.kind == 'B') {
                    if (!va.has[0] || !va.has[1] || !vb.has[2] || !vb.has[3]) {
                        return fail(*pl, "a Bézier primitive needs c0x/c0y on its start and c1x/c1y on its end vertex");
                    }
                    flatten_cubic(ws(va), xf.apply(va.c[0], va.c[1]), xf.apply(vb.c[2], vb.c[3]), ws(vb), opts.tolerance, 0, pts);
                } else {
                    pts.push_back(ws(vb));
                }
                cur = p.b;
                ++k;
            }
            const bool closed = cur == first && pts.size() > 2;
            if (closed) pts.pop_back();
            add_path(pen, closed, pts);
        }
        return true;
    }

    bool ellipse_shape(const xml::Node& shape, int pen, const Xf& xf) {
        double rx = 0, ry = 0;
        const std::string* a = shape.attr("Rx");
        const std::string* b = shape.attr("Ry");
        if (!a || !b || !to_double(*a, rx) || !to_double(*b, ry) || !(rx > 0) || !(ry > 0)) {
            return fail(shape, "an Ellipse needs positive Rx and Ry");
        }
        // The smallest power of two whose chord sagitta is within the tolerance:
        // f07's circles measure 32/64/128/256 segments by exactly this rule.
        const double r = std::max(rx * std::hypot(xf.a, xf.b), ry * std::hypot(xf.c, xf.d));
        int n = 8;
        while (n < 65536 && r * (1.0 - std::cos(kPi / n)) > opts.tolerance) n *= 2;
        std::vector<Pt> pts;
        pts.reserve(static_cast<std::size_t>(n));
        for (int i = 0; i < n; ++i) {
            const double t = 2.0 * kPi * i / n;
            pts.push_back(xf.apply(rx * std::cos(t), ry * std::sin(t)));
        }
        add_path(pen, true, pts);
        return true;
    }

    bool shape(const xml::Node& s) {
        const std::string* type = s.attr("Type");
        if (!type) return fail(s, "<Shape> has no Type");
        const std::string* ci = s.attr("CutIndex");
        double idx = 0;
        if (!ci || !to_double(*ci, idx)) return fail(s, "<Shape> has no numeric CutIndex");
        const int pen = static_cast<int>(idx);
        if (!job.layer(pen)) return fail(s, "<Shape> uses layer " + std::to_string(pen) + ", which has no <CutSetting>");
        if (*type != "Path" && *type != "Ellipse") {
            return fail(s, "Shape Type=\"" + *type + "\" is not supported (Path and Ellipse only)");
        }
        Xf xf;
        if (!read_xform(s, xf, error)) return false;
        ++info.shapes;
        return *type == "Path" ? path_shape(s, pen, xf) : ellipse_shape(s, pen, xf);
    }

    bool run(const std::string& text) {
        xml::Node root;
        if (!xml::parse(text, root, error)) return false;
        if (root.name != "LightBurnProject") return fail(root, "the root element is <" + root.name + ">, not <LightBurnProject>");
        if (const std::string* v = root.attr("AppVersion")) info.app_version = *v;
        const std::string* fv = root.attr("FormatVersion");
        double fvn = 0;
        if (!fv || !to_double(*fv, fvn)) return fail(root, "the project has no FormatVersion");
        if (fvn != 1) return fail(root, "FormatVersion " + *fv + " is not supported (the samples are all 1)");
        info.format_version = 1;
        if (const std::string* m = root.attr("MirrorX")) info.mirror_x = *m == "True";
        if (const std::string* m = root.attr("MirrorY")) info.mirror_y = *m == "True";
        if (info.mirror_x != (opts.machine.sign_x() < 0) || info.mirror_y != (opts.machine.sign_y() < 0)) {
            info.warnings.push_back("the project's MirrorX/MirrorY do not match the machine origin given; "
                                    "check --machine-origin");
        }
        // Two passes: all layers first, so a shape may appear before its CutSetting.
        for (const xml::Node& c : root.children) {
            if (c.name == "CutSetting" && !cut_setting(c)) return false;
        }
        for (const xml::Node& c : root.children) {
            if (c.name == "Shape") {
                if (!shape(c)) return false;
            } else if (c.name != "CutSetting" && c.name != "Thumbnail" && c.name != "VariableText" &&
                       c.name != "UIPrefs" && c.name != "Notes") {
                info.warnings.push_back(at_line(c, "<" + c.name + "> is not understood and was ignored"));
            }
        }
        job.start = opts.start;
        info.start = opts.start;
        info.job_origin = opts.job_origin;
        Pt lo, hi;
        bool any = false;
        for (const LaserPath& p : job.paths) {
            for (const Pt& q : p.pts) {
                if (!any) {
                    lo = hi = q;
                    any = true;
                }
                lo.x = std::min(lo.x, q.x);
                lo.y = std::min(lo.y, q.y);
                hi.x = std::max(hi.x, q.x);
                hi.y = std::max(hi.y, q.y);
            }
        }
        if (any) {
            switch (opts.job_origin) {
                case JobOrigin::BottomLeft: job.origin = {lo.x, lo.y}; break;
                case JobOrigin::BottomRight: job.origin = {hi.x, lo.y}; break;
                case JobOrigin::TopLeft: job.origin = {lo.x, hi.y}; break;
                case JobOrigin::TopRight: job.origin = {hi.x, hi.y}; break;
                case JobOrigin::Centre: job.origin = {(lo.x + hi.x) / 2, (lo.y + hi.y) / 2}; break;
            }
        }
        // Absolute: the head travels from machine zero, so that is where the
        // planner should assume it is (it picks the first start by nearness;
        // f03's file starts at the corner nearest machine zero).
        if (opts.start == StartMode::Absolute && opts.machine.has_bed()) {
            opts.machine.machine_to_bed(0.0, 0.0, job.origin.x, job.origin.y);
        }
        if (opts.machine.has_bed()) {
            job.bed_w = opts.machine.bed_w_mm;
            job.bed_h = opts.machine.bed_h_mm;
        }
        return true;
    }
};

bool read_file(const std::string& path, std::string& out, std::string& error) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        error = "cannot read '" + path + "'";
        return false;
    }
    out.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    return true;
}

}  // namespace

bool read_lbrn2(const std::string& text, LaserJob& job, Lbrn2Info& info, const Lbrn2Options& opts,
                std::string& error) {
    Reader r(opts);
    if (!r.run(text)) {
        error = r.error;
        return false;
    }
    job = std::move(r.job);
    info = std::move(r.info);
    return true;
}

bool read_lbrn2_file(const std::string& path, LaserJob& job, Lbrn2Info& info, const Lbrn2Options& opts,
                     std::string& error) {
    std::string text;
    if (!read_file(path, text, error)) return false;
    if (!read_lbrn2(text, job, info, opts, error)) {
        error = path + ": " + error;
        return false;
    }
    return true;
}

// --- writing -------------------------------------------------------------------

namespace {

// A 1x1 transparent PNG. The samples all carry a <Thumbnail>; whether
// LightBurn needs one is unknown, a blank one is harmless, and it redraws it.
const char kBlankThumbnail[] =
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mNkYPhfDwAChwGA60e6kgAAAABJRU5ErkJggg==";

// Copied from the samples (identical in all seven): LightBurn's own optimiser
// preferences, which the job does not own.
const char kPreamble[] =
    "    <VariableText>\n"
    "        <Start Value=\"0\"/>\n"
    "        <End Value=\"999\"/>\n"
    "        <Current Value=\"0\"/>\n"
    "        <Increment Value=\"1\"/>\n"
    "        <AutoAdvance Value=\"0\"/>\n"
    "    </VariableText>\n"
    "    <UIPrefs>\n"
    "        <Optimize_ByLayer Value=\"0\"/>\n"
    "        <Optimize_ByGroup Value=\"-1\"/>\n"
    "        <Optimize_ByPriority Value=\"1\"/>\n"
    "        <Optimize_WhichDirection Value=\"0\"/>\n"
    "        <Optimize_InnerToOuter Value=\"1\"/>\n"
    "        <Optimize_ByDirection Value=\"0\"/>\n"
    "        <Optimize_ReduceTravel Value=\"1\"/>\n"
    "        <Optimize_HideBacklash Value=\"0\"/>\n"
    "        <Optimize_ReduceDirChanges Value=\"0\"/>\n"
    "        <Optimize_ChooseCorners Value=\"0\"/>\n"
    "        <Optimize_AllowReverse Value=\"1\"/>\n"
    "        <Optimize_RemoveOverlaps Value=\"1\"/>\n"
    "        <Optimize_OptimalEntryPoint Value=\"1\"/>\n"
    "        <Optimize_OverlapDist Value=\"0.025\"/>\n"
    "    </UIPrefs>\n";

void val(std::string& out, const char* tag, const std::string& v) {
    out += "        <";
    out += tag;
    out += " Value=\"" + xml::escape(v) + "\"/>\n";
}

}  // namespace

bool write_lbrn2(const LaserJob& job, const Lbrn2Options& opts, std::string& out, std::string& error,
                 const Lbrn2Info* names) {
    for (const LaserLayer& l : job.layers) {
        if (l.mode == LayerMode::Skip || l.mode == LayerMode::Raster) {
            error = "layer " + std::to_string(l.pen) + " is " + layer_mode_name(l.mode) +
                    ": no sample shows how LightBurn spells that, so it is not written";
            return false;
        }
        if (!l.air) {
            error = "layer " + std::to_string(l.pen) +
                    " has air off: no sample shows how LightBurn spells that, so it is not written";
            return false;
        }
        if (l.pen < 0) {
            error = "layer pen " + std::to_string(l.pen) + " is not a LightBurn layer index";
            return false;
        }
    }
    for (const LaserPath& p : job.paths) {
        if (!job.layer(p.pen)) {
            error = "a path uses pen " + std::to_string(p.pen) + ", which has no layer record";
            return false;
        }
        if (p.pts.size() < 2) {
            error = "a path has fewer than two points";
            return false;
        }
    }
    std::string s;
    s += "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    s += std::string("<LightBurnProject AppVersion=\"1.4.03\" FormatVersion=\"1\" MaterialHeight=\"0\" MirrorX=\"") +
         (opts.machine.sign_x() < 0 ? "True" : "False") + "\" MirrorY=\"" +
         (opts.machine.sign_y() < 0 ? "True" : "False") + "\">\n";
    s += std::string("    <Thumbnail Source=\"") + kBlankThumbnail + "\"/>\n";
    s += kPreamble;
    for (const LaserLayer& l : job.layers) {
        std::string name = "Pen " + std::to_string(l.pen);
        if (names) {
            for (const auto& n : names->layer_names) {
                if (n.first == l.pen && !n.second.empty()) name = n.second;
            }
        }
        s += "    <CutSetting type=\"Cut\">\n";
        val(s, "index", std::to_string(l.pen));
        val(s, "name", name);
        val(s, "minPower", num(l.min_power));
        val(s, "maxPower", num(l.max_power));
        val(s, "minPower2", num(l.has_power2 ? l.min_power2 : l.min_power));
        val(s, "maxPower2", num(l.has_power2 ? l.max_power2 : l.max_power));
        val(s, "speed", num(l.speed));
        val(s, "PPI", "0");
        if (l.passes != 1) val(s, "numPasses", std::to_string(l.passes));
        val(s, "perfLen", "0.01");
        val(s, "perfSkip", "0.01");
        val(s, "dotTime", "1");
        val(s, "scanOpt", "byGroup");
        val(s, "overscan", "0");
        val(s, "interval", "0.088");
        val(s, "priority", std::to_string(l.order));
        val(s, "tabSize", "0.50038");
        val(s, "tabCount", "1");
        val(s, "tabCountMax", "1");
        val(s, "tabSpacing", "50.038");
        s += "    </CutSetting>\n";
    }
    int vert_id = 0;
    for (const LaserPath& p : job.paths) {
        s += "    <Shape Type=\"Path\" CutIndex=\"" + std::to_string(p.pen) + "\" VertID=\"" +
             std::to_string(vert_id++) + "\">\n";
        // Vertices carry workspace coordinates directly, so the transform is
        // the identity; the samples' -1 0 0 -1 only reflects where LightBurn
        // keeps the shape's own origin.
        s += "        <XForm>1 0 0 1 0 0</XForm>\n";
        std::string verts, prims;
        for (const Pt& q : p.pts) {
            const Pt w = to_workspace(opts.machine, q);
            verts += "V" + num(w.x) + " " + num(w.y) + "c0x1c1x1";
        }
        if (p.closed) {
            prims = "LineClosed";
        } else {
            for (std::size_t i = 0; i + 1 < p.pts.size(); ++i) {
                prims += "L" + std::to_string(i) + " " + std::to_string(i + 1);
            }
        }
        s += "        <VertList>" + verts + "</VertList>\n";
        s += "        <PrimList>" + prims + "</PrimList>\n";
        s += "    </Shape>\n";
    }
    s += "    <Notes ShowOnLoad=\"0\" Notes=\"\"/>\n";
    s += "</LightBurnProject>\n";
    // LightBurn's samples use CRLF line ends (saved on Windows); match them.
    out.clear();
    out.reserve(s.size() + s.size() / 40);
    for (char c : s) {
        if (c == '\n') out.push_back('\r');
        out.push_back(c);
    }
    return true;
}

bool write_lbrn2_file(const std::string& path, const LaserJob& job, const Lbrn2Options& opts,
                      std::string& error, const Lbrn2Info* names) {
    std::string text;
    if (!write_lbrn2(job, opts, text, error, names)) return false;
    std::ofstream f(path, std::ios::binary);
    if (!f || !f.write(text.data(), static_cast<std::streamsize>(text.size()))) {
        error = "cannot write '" + path + "'";
        return false;
    }
    return true;
}

}  // namespace ncad::laser
