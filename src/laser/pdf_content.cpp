// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// The in-tree content-stream interpreter: tokenizer, graphics state, paths.
// It never sees a file, only the decompressed stream, which is what lets it be
// tested without qpdf.

#include "ncad/laser/pdf_import.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cctype>
#include <set>

namespace ncad::laser {

namespace {

constexpr double kMmPerUnit = 25.4 / 72.0;
constexpr double kZeroLen = 1e-6;
constexpr double kCloseEps = 1e-4;
constexpr std::size_t kMaxQDepth = 512;  // a runaway q would otherwise eat memory

// --- tokens -----------------------------------------------------------------

struct Operand {
    enum Kind { Number, Name, Other } kind{Other};
    double num{0.0};
    std::string text;  // a name without its slash; empty otherwise
};

bool is_space(char c) { return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\f' || c == '\0'; }
bool is_delim(char c) {
    return c == '(' || c == ')' || c == '<' || c == '>' || c == '[' || c == ']' || c == '{' || c == '}' ||
           c == '/' || c == '%';
}
bool is_regular(char c) { return !is_space(c) && !is_delim(c); }

int hex_val(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

class Lexer {
public:
    explicit Lexer(const std::string& s) : s_(s) {}

    enum class Tok { End, Operand, Op };

    // The next operand or operator. Strings, hex strings, arrays and
    // dictionaries come back as one opaque operand: an operator-looking word
    // inside one of them is data, not a command, so they are skipped whole.
    Tok next(Operand& operand, std::string& op) {
        skip_space();
        if (i_ >= s_.size()) return Tok::End;
        const char c = s_[i_];
        operand = Operand{};
        if (c == '(') {
            skip_string();
            return Tok::Operand;
        }
        if (c == '/') {
            ++i_;
            operand.kind = Operand::Name;
            while (i_ < s_.size() && is_regular(s_[i_])) {
                // #xx escapes a byte in a name.
                if (s_[i_] == '#' && i_ + 2 < s_.size() && hex_val(s_[i_ + 1]) >= 0 && hex_val(s_[i_ + 2]) >= 0) {
                    operand.text += static_cast<char>(hex_val(s_[i_ + 1]) * 16 + hex_val(s_[i_ + 2]));
                    i_ += 3;
                } else {
                    operand.text += s_[i_++];
                }
            }
            return Tok::Operand;
        }
        if (c == '<' && i_ + 1 < s_.size() && s_[i_ + 1] == '<') {
            skip_nested();
            return Tok::Operand;
        }
        if (c == '<') {
            while (i_ < s_.size() && s_[i_] != '>') ++i_;
            if (i_ < s_.size()) ++i_;
            return Tok::Operand;
        }
        if (c == '[') {
            skip_nested();
            return Tok::Operand;
        }
        if (c == ']' || c == '>' || c == ')' || c == '{' || c == '}') {
            // A stray closer: not a token anything can use. Drop it.
            ++i_;
            return next(operand, op);
        }
        std::size_t j = i_;
        while (j < s_.size() && is_regular(s_[j])) ++j;
        const std::string word = s_.substr(i_, j - i_);
        i_ = j;
        if (std::strchr("+-.0123456789", word[0]) != nullptr) {
            char* end = nullptr;
            const double v = std::strtod(word.c_str(), &end);
            // Producers write things like "-.5" and "0.-3"; strtod's prefix is
            // what a viewer would use too.
            if (end != word.c_str() && std::isfinite(v)) {
                operand.kind = Operand::Number;
                operand.num = v;
                return Tok::Operand;
            }
        }
        if (word == "true" || word == "false" || word == "null") return Tok::Operand;
        op = word;
        return Tok::Op;
    }

    // Inline image: after BI, skip the key/values to ID, then the data to a
    // whitespace-delimited EI. The data is binary and may contain "EI".
    void skip_inline_image() {
        Operand o;
        std::string w;
        for (;;) {
            const std::size_t before = i_;
            const Tok t = next(o, w);
            if (t == Tok::End) return;
            if (t == Tok::Op && w == "ID") break;
            if (i_ == before) return;
        }
        if (i_ < s_.size() && is_space(s_[i_])) ++i_;  // the single byte after ID
        for (std::size_t k = i_; k + 1 < s_.size(); ++k) {
            if (s_[k] == 'E' && s_[k + 1] == 'I' && (k == 0 || is_space(s_[k - 1])) &&
                (k + 2 >= s_.size() || is_space(s_[k + 2]))) {
                i_ = k + 2;
                return;
            }
        }
        i_ = s_.size();
    }

private:
    void skip_space() {
        while (i_ < s_.size()) {
            if (is_space(s_[i_])) {
                ++i_;
            } else if (s_[i_] == '%') {
                while (i_ < s_.size() && s_[i_] != '\n' && s_[i_] != '\r') ++i_;
            } else {
                break;
            }
        }
    }
    // Literal string: parentheses nest, a backslash protects the next byte.
    void skip_string() {
        int depth = 0;
        while (i_ < s_.size()) {
            const char c = s_[i_++];
            if (c == '\\') {
                if (i_ < s_.size()) ++i_;
            } else if (c == '(') {
                ++depth;
            } else if (c == ')' && --depth == 0) {
                return;
            }
        }
    }
    // An array or dictionary, strings inside included.
    void skip_nested() {
        int depth = 0;
        while (i_ < s_.size()) {
            const char c = s_[i_];
            if (c == '(') {
                skip_string();
                continue;
            }
            if (c == '%') {
                while (i_ < s_.size() && s_[i_] != '\n' && s_[i_] != '\r') ++i_;
                continue;
            }
            ++i_;
            if (c == '[' || (c == '<' && i_ < s_.size() && s_[i_] == '<')) {
                if (c == '<') ++i_;
                ++depth;
            } else if (c == ']' || (c == '>' && i_ < s_.size() && s_[i_] == '>')) {
                if (c == '>') ++i_;
                if (--depth <= 0) return;
            }
        }
    }

    const std::string& s_;
    std::size_t i_{0};
};

// --- geometry ---------------------------------------------------------------

struct Matrix {
    double a{1}, b{0}, c{0}, d{1}, e{0}, f{0};
    Pt apply(double x, double y) const { return {a * x + c * y + e, b * x + d * y + f}; }
    // this applied after m: PDF's `cm` is  m x CTM  in row-vector terms.
    Matrix after(const Matrix& m) const {
        return {m.a * a + m.b * c, m.a * b + m.b * d, m.c * a + m.d * c, m.c * b + m.d * d,
                m.e * a + m.f * c + e, m.e * b + m.f * d + f};
    }
    double scale() const { return std::sqrt(std::fabs(a * d - b * c)); }
};

double dist(const Pt& a, const Pt& b) { return std::hypot(a.x - b.x, a.y - b.y); }

// --- colour -----------------------------------------------------------------

struct Colour {
    std::uint32_t rgb{0};  // black, the initial colour in every device space
    bool known{true};      // false: a pattern or something with no RGB
};

std::uint32_t pack(double r, double g, double b) {
    auto q = [](double v) { return static_cast<std::uint32_t>(std::lround(std::clamp(v, 0.0, 1.0) * 255.0)); };
    return q(r) << 16 | q(g) << 8 | q(b);
}

struct Space {
    std::string name{"DeviceGray"};
    bool device{true};  // one of the three Device spaces or their Cal/Lab cousins
};

struct GState {
    Matrix ctm;
    Colour stroke, fill;
    Space stroke_space, fill_space;
    double width{1.0};  // user units, as `w` set it
};

struct Seg {
    bool line{true};
    Pt c1, c2, end;
};

struct Subpath {
    Pt start;
    std::vector<Seg> segs;
    bool closed{false};
};

std::string count_of(std::size_t n, const char* one, const char* many) {
    return std::to_string(n) + " " + (n == 1 ? one : many);
}

class Interpreter {
public:
    Interpreter(const PdfPageData& page, SvgImport& out) : page_(page), res_(out) {}

    bool run(std::string& error) {
        const double x0 = std::min(page_.box[0], page_.box[2]), x1 = std::max(page_.box[0], page_.box[2]);
        const double y0 = std::min(page_.box[1], page_.box[3]), y1 = std::max(page_.box[1], page_.box[3]);
        if (!(x1 > x0) || !(y1 > y0) || !std::isfinite(x1 - x0) || !std::isfinite(y1 - y0)) {
            error = "the MediaBox is empty";
            return false;
        }
        res_ = SvgImport{};
        res_.width_mm = (x1 - x0) * kMmPerUnit;
        res_.height_mm = (y1 - y0) * kMmPerUnit;
        // Initial CTM: user space to millimetres with the box's lower-left
        // corner at the origin. PDF is Y up already, so there is no flip.
        gs_.ctm = {kMmPerUnit, 0, 0, kMmPerUnit, -x0 * kMmPerUnit, -y0 * kMmPerUnit};

        Lexer lex(page_.content);
        std::vector<Operand> args;
        Operand o;
        std::string op;
        for (;;) {
            const Lexer::Tok t = lex.next(o, op);
            if (t == Lexer::Tok::End) break;
            if (t == Lexer::Tok::Operand) {
                // Nothing real has dozens of operands; a runaway list is junk
                // and must not grow without bound.
                if (args.size() < 64) args.push_back(std::move(o));
                continue;
            }
            if (op == "BI") {
                lex.skip_inline_image();
                ++images_;
            } else {
                exec(op, args);
            }
            args.clear();
        }
        warn();
        return true;
    }

private:
    // The last `n` operands as numbers; false when there are not that many.
    bool nums(const std::vector<Operand>& a, std::size_t n, double* v) const {
        if (a.size() < n) return false;
        for (std::size_t i = 0; i < n; ++i) {
            const Operand& x = a[a.size() - n + i];
            if (x.kind != Operand::Number) return false;
            v[i] = x.num;
        }
        return true;
    }

    void exec(const std::string& op, const std::vector<Operand>& a) {
        double v[6];
        // --- graphics state
        if (op == "q") {
            if (stack_.size() < kMaxQDepth) stack_.push_back(gs_);
            else ++q_overflow_;
        } else if (op == "Q") {
            if (stack_.empty()) ++unbalanced_q_;
            else {
                gs_ = stack_.back();
                stack_.pop_back();
            }
        } else if (op == "cm") {
            if (nums(a, 6, v)) gs_.ctm = gs_.ctm.after({v[0], v[1], v[2], v[3], v[4], v[5]});
            else ++malformed_;
        } else if (op == "w") {
            if (nums(a, 1, v)) gs_.width = std::fabs(v[0]);
            else ++malformed_;
        // --- path construction
        } else if (op == "m") {
            if (nums(a, 2, v)) move_to(v[0], v[1]);
            else ++malformed_;
        } else if (op == "l") {
            if (nums(a, 2, v)) line_to(v[0], v[1]);
            else ++malformed_;
        } else if (op == "c") {
            if (nums(a, 6, v)) curve_to(v[0], v[1], v[2], v[3], v[4], v[5]);
            else ++malformed_;
        } else if (op == "v") {  // first control point is the current point
            if (nums(a, 4, v)) {
                const Pt cur = current();
                const Pt p = gs_.ctm.apply(v[0], v[1]), e = gs_.ctm.apply(v[2], v[3]);
                add_seg(false, cur, p, e);
            } else ++malformed_;
        } else if (op == "y") {  // second control point is the end point
            if (nums(a, 4, v)) {
                const Pt p = gs_.ctm.apply(v[0], v[1]), e = gs_.ctm.apply(v[2], v[3]);
                add_seg(false, p, e, e);
            } else ++malformed_;
        } else if (op == "h") {
            close_subpath();
        } else if (op == "re") {
            if (nums(a, 4, v)) rect(v[0], v[1], v[2], v[3]);
            else ++malformed_;
        // --- painting
        } else if (op == "S") {
            paint(true, false, false);
        } else if (op == "s") {
            paint(true, false, true);
        } else if (op == "f" || op == "F" || op == "f*") {
            paint(false, true, false);
        } else if (op == "B" || op == "B*") {
            paint(true, true, false);
        } else if (op == "b" || op == "b*") {
            paint(true, true, true);
        } else if (op == "n") {
            paint(false, false, false);
        } else if (op == "W" || op == "W*") {
            ++clips_;  // the path stays for the painting operator that follows
        // --- colour
        } else if (op == "G" || op == "g") {
            if (nums(a, 1, v)) set_colour(op == "G", {"DeviceGray", true}, {v[0]});
            else ++malformed_;
        } else if (op == "RG" || op == "rg") {
            if (nums(a, 3, v)) set_colour(op == "RG", {"DeviceRGB", true}, {v[0], v[1], v[2]});
            else ++malformed_;
        } else if (op == "K" || op == "k") {
            if (nums(a, 4, v)) set_colour(op == "K", {"DeviceCMYK", true}, {v[0], v[1], v[2], v[3]});
            else ++malformed_;
        } else if (op == "CS" || op == "cs") {
            const bool stroke = op == "CS";
            if (!a.empty() && a.back().kind == Operand::Name) {
                Space sp;
                sp.name = a.back().text;
                sp.device = sp.name == "DeviceGray" || sp.name == "DeviceRGB" || sp.name == "DeviceCMYK" ||
                            sp.name == "CalGray" || sp.name == "CalRGB" || sp.name == "G" || sp.name == "RGB" ||
                            sp.name == "CMYK";
                (stroke ? gs_.stroke_space : gs_.fill_space) = sp;
                // Every space starts at its own idea of black; ours is black.
                (stroke ? gs_.stroke : gs_.fill) = Colour{};
                if (!sp.device) named_spaces_.insert(sp.name);
            } else ++malformed_;
        } else if (op == "SC" || op == "SCN" || op == "sc" || op == "scn") {
            named_colour(op[0] == 'S', a);
        // --- marked content
        } else if (op == "BMC") {
            mc_.push_back({});
        } else if (op == "BDC") {
            begin_dc(a);
        } else if (op == "EMC") {
            if (mc_.empty()) ++unbalanced_emc_;
            else mc_.pop_back();
        // --- known, deliberately not imported
        } else if (op == "Tj" || op == "TJ" || op == "'" || op == "\"") {
            ++text_;
        } else if (op == "Do") {
            ++xobjects_;
        } else if (op == "sh") {
            ++shadings_;
        } else if (op == "gs" || op == "d" || op == "J" || op == "j" || op == "M" || op == "i" || op == "ri" ||
                   op == "BT" || op == "ET" || op == "Tc" || op == "Tw" || op == "Tz" || op == "TL" ||
                   op == "Tf" || op == "Tr" || op == "Ts" || op == "Td" || op == "TD" || op == "Tm" ||
                   op == "T*" || op == "MP" || op == "DP" || op == "BX" || op == "EX" || op == "d0" ||
                   op == "d1" || op == "ID" || op == "EI") {
            // Line caps, dashes, text positioning and the like change nothing
            // about the centre line the laser follows.
        } else {
            unknown_.insert(op);
        }
    }

    // --- path building

    Pt current() const {
        if (!sub_.empty()) return sub_.back().segs.empty() ? sub_.back().start : sub_.back().segs.back().end;
        return {0, 0};
    }

    void move_to(double x, double y) {
        Subpath s;
        s.start = gs_.ctm.apply(x, y);
        sub_.push_back(std::move(s));
    }

    // A segment with no current point (or after `h`) starts where the last
    // subpath started, as the spec has it; with nothing at all it is malformed.
    bool ensure_open() {
        if (sub_.empty()) {
            ++malformed_;
            return false;
        }
        if (sub_.back().closed) {
            Subpath s;
            s.start = sub_.back().start;
            sub_.push_back(std::move(s));
        }
        return true;
    }

    void add_seg(bool line, const Pt& c1, const Pt& c2, const Pt& end) {
        if (!ensure_open()) return;
        sub_.back().segs.push_back({line, c1, c2, end});
    }

    void line_to(double x, double y) {
        if (!ensure_open()) return;
        const Pt p = gs_.ctm.apply(x, y);
        sub_.back().segs.push_back({true, {}, {}, p});
    }

    void curve_to(double x1, double y1, double x2, double y2, double x3, double y3) {
        add_seg(false, gs_.ctm.apply(x1, y1), gs_.ctm.apply(x2, y2), gs_.ctm.apply(x3, y3));
    }

    void close_subpath() {
        if (!sub_.empty()) sub_.back().closed = true;
    }

    void rect(double x, double y, double w, double h) {
        move_to(x, y);
        line_to(x + w, y);
        line_to(x + w, y + h);
        line_to(x, y + h);
        close_subpath();
    }

    // --- colour

    // Device spaces by component count, which is what the operator names.
    static Colour from_components(const std::vector<double>& c) {
        Colour r;
        if (c.size() == 1) r.rgb = pack(c[0], c[0], c[0]);
        else if (c.size() == 3) r.rgb = pack(c[0], c[1], c[2]);
        else if (c.size() == 4) {
            const double k = 1.0 - std::clamp(c[3], 0.0, 1.0);
            r.rgb = pack((1.0 - std::clamp(c[0], 0.0, 1.0)) * k, (1.0 - std::clamp(c[1], 0.0, 1.0)) * k,
                         (1.0 - std::clamp(c[2], 0.0, 1.0)) * k);
        } else {
            r.known = false;
        }
        return r;
    }

    void set_colour(bool stroke, const Space& sp, const std::vector<double>& c) {
        (stroke ? gs_.stroke_space : gs_.fill_space) = sp;
        (stroke ? gs_.stroke : gs_.fill) = from_components(c);
    }

    // SC/SCN: the components mean what the current space says. For the Device
    // and Cal spaces that is the count; for anything else (ICCBased, Separation,
    // DeviceN, Indexed, Pattern) the file's space dictionary is not read, so
    // the count is the only guide -- three is RGB, four CMYK -- and a single
    // value is a tint, which is darker the higher it is. A guess, and warned.
    void named_colour(bool stroke, const std::vector<Operand>& a) {
        std::vector<double> c;
        bool pattern = false;
        for (const Operand& x : a) {
            if (x.kind == Operand::Number) c.push_back(x.num);
            else if (x.kind == Operand::Name) pattern = true;
        }
        const Space& sp = stroke ? gs_.stroke_space : gs_.fill_space;
        Colour col;
        if (pattern || c.empty() || (c.size() != 1 && c.size() != 3 && c.size() != 4)) {
            col.known = false;
        } else if (c.size() == 1 && !sp.device) {
            col = from_components({1.0 - c[0]});
        } else {
            col = from_components(c);
        }
        if (!sp.device || !col.known) named_spaces_.insert(sp.name);
        (stroke ? gs_.stroke : gs_.fill) = col;
    }

    // --- marked content

    struct Mc {
        std::string layer;  // empty: this level names no layer
    };

    void begin_dc(const std::vector<Operand>& a) {
        Mc m;
        // `/OC /MC0 BDC`: the tag, then a name that /Properties resolves.
        if (a.size() >= 2 && a[a.size() - 2].kind == Operand::Name && a[a.size() - 2].text == "OC" &&
            a.back().kind == Operand::Name) {
            const auto it = page_.properties.find(a.back().text);
            if (it != page_.properties.end()) m.layer = it->second;
        }
        mc_.push_back(std::move(m));
    }

    std::string layer() const {
        for (auto it = mc_.rbegin(); it != mc_.rend(); ++it) {
            if (!it->layer.empty()) return it->layer;
        }
        return "0";
    }

    // --- painting

    void paint(bool stroke, bool fill, bool close) {
        std::vector<Subpath> paths = std::move(sub_);
        sub_.clear();
        if (!stroke && !fill) return;
        const Colour& c = stroke ? gs_.stroke : gs_.fill;
        if (fill) ++res_.filled;
        int aci = 7;
        if (c.known) {
            bool exact = false;
            aci = rgb_to_aci(c.rgb, &exact);
            if (!exact) ++inexact_;
        } else {
            ++no_colour_;
        }
        // Width in mm: `w` in user units, scaled by the CTM, which already
        // carries the 1/72 in conversion.
        const double width_mm = stroke ? gs_.width * gs_.ctm.scale() : 0.0;
        const double width = width_mm >= kSvgWideStroke - 1e-4 ? width_mm : 0.0;

        const std::string lay = layer();
        bool any = false;
        for (Subpath& sp : paths) {
            if (close || !stroke) sp.closed = true;  // a fill closes its subpaths
            any |= emit(sp, lay, aci, width);
        }
        if (any && std::none_of(res_.layers.begin(), res_.layers.end(),
                                [&](const SvgLayer& l) { return l.name == lay; })) {
            res_.layers.push_back({lay, aci});
        }
    }

    // Runs of lines and runs of curves, as the SVG importer makes them.
    bool emit(const Subpath& sp, const std::string& lay, int aci, double width) {
        std::vector<Seg> segs;
        Pt cur = sp.start;
        for (const Seg& g : sp.segs) {
            if (g.line && dist(cur, g.end) < kZeroLen) continue;
            segs.push_back(g);
            cur = g.end;
        }
        // Closing draws the straight edge back to the start, unless the path
        // already got there.
        if (sp.closed && !segs.empty() && dist(cur, sp.start) >= kZeroLen) segs.push_back({true, {}, {}, sp.start});
        if (segs.empty()) return false;

        std::vector<SvgShape> runs;
        Pt run_start = sp.start;
        for (std::size_t i = 0; i < segs.size();) {
            const bool line = segs[i].line;
            std::size_t j = i;
            while (j < segs.size() && segs[j].line == line) ++j;
            SvgShape sh;
            sh.layer = lay;
            sh.aci = aci;
            sh.curve = !line;
            sh.pts.push_back(run_start);
            for (std::size_t k = i; k < j; ++k) {
                if (!line) {
                    sh.pts.push_back(segs[k].c1);
                    sh.pts.push_back(segs[k].c2);
                }
                sh.pts.push_back(segs[k].end);
            }
            run_start = segs[j - 1].end;
            runs.push_back(std::move(sh));
            i = j;
        }
        if (sp.closed && runs.size() == 1 && dist(runs[0].pts.front(), runs[0].pts.back()) < kCloseEps) {
            SvgShape& r = runs[0];
            if (r.curve) {
                r.closed = true;
            } else if (r.pts.size() > 3) {
                r.pts.pop_back();
                r.closed = true;
            }
        }
        for (SvgShape& r : runs) {
            if (r.curve) {
                if (width > 0.0) ++wide_curves_;
            } else {
                r.width = width;
            }
            res_.shapes.push_back(std::move(r));
        }
        return true;
    }

    // --- diagnostics

    void warn() {
        std::vector<std::string>& w = res_.warnings;
        if (res_.filled) {
            w.push_back(count_of(res_.filled, "shape has a fill", "shapes have fills") +
                        "; filled regions are raster, which is not supported yet, so " +
                        (res_.filled == 1 ? "its outline is" : "their outlines are") + " imported");
        }
        if (inexact_) {
            w.push_back(count_of(inexact_, "colour is", "colours are") +
                        " not in the AutoCAD palette and went to the nearest entry");
        }
        if (!named_spaces_.empty()) {
            std::string names;
            for (const std::string& n : named_spaces_) names += (names.empty() ? "" : ", ") + n;
            w.push_back("colour space " + names + " is not a device space and its definition is not read; "
                        "colours were guessed from their component counts");
        }
        if (no_colour_) {
            w.push_back(count_of(no_colour_, "paint has", "paints have") +
                        " no plain colour (a pattern or unknown space): imported in colour 7");
        }
        if (wide_curves_) {
            w.push_back(count_of(wide_curves_, "curve has", "curves have") +
                        " a stroke of 0.5 mm or more; splines carry no width, so " +
                        (wide_curves_ == 1 ? "it is" : "they are") + " cut on the centre line");
        }
        if (text_) w.push_back("text is not imported; convert it to outlines first");
        if (xobjects_) {
            w.push_back(count_of(xobjects_, "form or image XObject was", "form or image XObjects were") +
                        " skipped (Do)");
        }
        if (shadings_) w.push_back(count_of(shadings_, "shading was", "shadings were") + " skipped (sh)");
        if (images_) w.push_back(count_of(images_, "inline image was", "inline images were") + " skipped");
        if (clips_) w.push_back(count_of(clips_, "clipping path was", "clipping paths were") + " ignored");
        if (unbalanced_q_) w.push_back(count_of(unbalanced_q_, "Q", "Qs") + " with no matching q ignored");
        if (unbalanced_emc_) w.push_back(count_of(unbalanced_emc_, "EMC", "EMCs") + " with no matching BMC/BDC ignored");
        if (q_overflow_) w.push_back("q nested deeper than " + std::to_string(kMaxQDepth) + "; the excess was ignored");
        if (malformed_) w.push_back(count_of(malformed_, "operator", "operators") + " had missing or bad operands and " +
                                    (malformed_ == 1 ? "was" : "were") + " skipped");
        if (!unknown_.empty()) {
            std::string names;
            for (const std::string& n : unknown_) names += (names.empty() ? "" : " ") + n;
            w.push_back(std::string("unknown operator") + (unknown_.size() == 1 ? "" : "s") + " skipped: " + names);
        }
        if (res_.shapes.empty()) w.push_back("the page has no paths to import");
    }

    const PdfPageData& page_;
    SvgImport& res_;
    GState gs_;
    std::vector<GState> stack_;
    std::vector<Subpath> sub_;
    std::vector<Mc> mc_;
    std::set<std::string> unknown_, named_spaces_;
    std::size_t unbalanced_q_{0}, unbalanced_emc_{0}, q_overflow_{0}, malformed_{0}, clips_{0}, text_{0},
        xobjects_{0}, shadings_{0}, images_{0}, inexact_{0}, no_colour_{0}, wide_curves_{0};
};

}  // namespace

bool import_pdf_content(const PdfPageData& page, SvgImport& out, std::string& error) {
    SvgImport res;
    Interpreter in(page, res);
    if (!in.run(error)) return false;
    out = std::move(res);
    return true;
}

}  // namespace ncad::laser
