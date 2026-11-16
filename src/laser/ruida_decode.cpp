// SPDX-License-Identifier: BSD-3-Clause AND MIT
// Copyright (c) 2026, Sadie Forbes
// Copyright (c) 2021 meerk40t
//
// The command table is ported from meerk40t/ruida/rdjob.py (the command
// constants and RDJob.process). See third_party/meerk40t/NOTICE.

#include "ncad/laser/ruida_decode.hpp"

#include "ncad/laser/ruida_codec.hpp"

#include <cmath>
#include <cstdio>

namespace ncad::laser::ruida {

namespace {

#define K1(a) {a, 0, 0}, 1
#define K2(a, b) {a, b, 0}, 2
#define K3(a, b, c) {a, b, c}, 3

// Where MeerK40t's constants and its decoder disagree, the decoder's reading is
// taken when it is self-consistent and the writer's when the decoder slices at
// a wrong offset; each such row says so.
const RdSpec kTable[] = {
    // --- motion ---
    {K2(0x80, 0x00), "Axis X Move", "a"},
    {K2(0x80, 0x08), "Axis Z Move", "a"},  // constants say 80 01, with a TODO
    {K1(0x88), "Move Abs", "aa"},
    {K1(0x89), "Move Rel", "rr"},
    {K1(0x8A), "Move Rel X", "r"},
    {K1(0x8B), "Move Rel Y", "r"},
    {K2(0xA0, 0x00), "Axis Y Move", "a"},
    {K2(0xA0, 0x08), "Axis U Move", "a"},
    {K1(0xA8), "Cut Abs", "aa"},
    {K1(0xA9), "Cut Rel", "rr"},
    {K1(0xAA), "Cut Rel X", "r"},
    {K1(0xAB), "Cut Rel Y", "r"},

    // --- power ---
    {K1(0xC7), "Imd Power 1", "p"},
    {K1(0xC0), "Imd Power 2", "p"},
    {K1(0xC2), "Imd Power 3", "p"},
    {K1(0xC3), "Imd Power 4", "p"},
    {K1(0xC8), "End Power 1", "p"},
    {K1(0xC1), "End Power 2", "p"},
    {K1(0xC4), "End Power 3", "p"},
    {K1(0xC5), "End Power 4", "p"},
    {K2(0xC6, 0x01), "Min Power 1", "p"},
    {K2(0xC6, 0x02), "Max Power 1", "p"},
    {K2(0xC6, 0x21), "Min Power 2", "p"},
    {K2(0xC6, 0x22), "Max Power 2", "p"},
    {K2(0xC6, 0x05), "Min Power 3", "p"},
    {K2(0xC6, 0x06), "Max Power 3", "p"},
    {K2(0xC6, 0x07), "Min Power 4", "p"},
    {K2(0xC6, 0x08), "Max Power 4", "p"},
    {K2(0xC6, 0x10), "Laser Interval", "t"},
    {K2(0xC6, 0x11), "Add Delay", "t"},
    {K2(0xC6, 0x12), "Laser On Delay", "t"},
    {K2(0xC6, 0x13), "Laser Off Delay", "t"},
    {K2(0xC6, 0x15), "Laser On Delay 2", "t"},
    {K2(0xC6, 0x16), "Laser Off Delay 2", "t"},
    {K2(0xC6, 0x31), "Min Power 1 Part", "bp"},
    {K2(0xC6, 0x32), "Max Power 1 Part", "bp"},
    {K2(0xC6, 0x41), "Min Power 2 Part", "bp"},
    {K2(0xC6, 0x42), "Max Power 2 Part", "bp"},
    {K2(0xC6, 0x35), "Min Power 3 Part", "bp"},
    {K2(0xC6, 0x36), "Max Power 3 Part", "bp"},
    {K2(0xC6, 0x37), "Min Power 4 Part", "bp"},
    {K2(0xC6, 0x38), "Max Power 4 Part", "bp"},
    {K2(0xC6, 0x50), "Through Power 1", "p"},
    {K2(0xC6, 0x51), "Through Power 2", "p"},
    {K2(0xC6, 0x55), "Through Power 3", "p"},
    {K2(0xC6, 0x56), "Through Power 4", "p"},
    {K2(0xC6, 0x60), "Frequency Part", "bbf"},  // laser, part, frequency

    // --- speed ---
    {K2(0xC9, 0x02), "Speed Laser 1", "s"},
    {K2(0xC9, 0x03), "Speed Axis", "s"},
    {K2(0xC9, 0x04), "Speed Laser 1 Part", "bs"},
    {K2(0xC9, 0x05), "Force Eng Speed", "s"},
    {K2(0xC9, 0x06), "Speed Axis Move", "s"},

    // --- layer ---
    {K3(0xCA, 0x01, 0x00), "Layer End", ""},
    {K3(0xCA, 0x01, 0x01), "Work Mode 1", ""},
    {K3(0xCA, 0x01, 0x02), "Work Mode 2", ""},
    {K3(0xCA, 0x01, 0x03), "Work Mode 3", ""},
    {K3(0xCA, 0x01, 0x04), "Work Mode 4", ""},
    {K3(0xCA, 0x01, 0x55), "Work Mode 5", ""},
    {K3(0xCA, 0x01, 0x05), "Work Mode 6", ""},
    {K3(0xCA, 0x01, 0x10), "Laser Device 0", ""},
    {K3(0xCA, 0x01, 0x11), "Laser Device 1", ""},
    {K3(0xCA, 0x01, 0x12), "Air Assist Off", ""},
    {K3(0xCA, 0x01, 0x13), "Air Assist On", ""},
    {K3(0xCA, 0x01, 0x14), "DB Head", ""},
    {K3(0xCA, 0x01, 0x30), "En Laser 2 Offset 0", ""},
    {K3(0xCA, 0x01, 0x31), "En Laser 2 Offset 1", ""},
    {K2(0xCA, 0x02), "Layer Number Part", "b"},
    {K2(0xCA, 0x03), "En Laser Tube Start", "b"},
    {K2(0xCA, 0x04), "X Sign Map", "b"},
    {K2(0xCA, 0x05), "Layer Color", "c"},
    {K2(0xCA, 0x06), "Layer Color Part", "bc"},
    {K2(0xCA, 0x10), "En Ex IO", "b"},
    {K2(0xCA, 0x22), "Max Layer Part", "b"},
    {K2(0xCA, 0x30), "U File ID", "U"},
    {K2(0xCA, 0x40), "ZU Map", "b"},
    {K2(0xCA, 0x41), "Work Mode Part", "bb"},

    // --- handshake ---
    {K1(0xCC), "ACK", ""},
    {K1(0xCD), "ERR", ""},
    {K1(0xCE), "ENQ", ""},
    {K1(0xCF), "NAK", ""},

    {K1(0xD0), "Set Inhale Zone", "b"},
    {K1(0xD7), "End Of File", ""},

    // --- process control ---
    {K2(0xD8, 0x00), "Start Process", ""},
    {K2(0xD8, 0x01), "Stop Process", ""},
    {K2(0xD8, 0x02), "Pause Process", ""},
    {K2(0xD8, 0x03), "Restore Process", ""},
    {K2(0xD8, 0x10), "Ref Point 2 (machine zero)", ""},
    {K2(0xD8, 0x11), "Ref Point 1 (anchor)", ""},
    {K2(0xD8, 0x12), "Ref Point 0 (current position)", ""},
    {K2(0xD8, 0x20), "Keydown X Left", ""},
    {K2(0xD8, 0x21), "Keydown X Right", ""},
    {K2(0xD8, 0x22), "Keydown Y Top", ""},
    {K2(0xD8, 0x23), "Keydown Y Bottom", ""},
    {K2(0xD8, 0x24), "Keydown Z Up", ""},
    {K2(0xD8, 0x25), "Keydown Z Down", ""},
    {K2(0xD8, 0x26), "Keydown U Forward", ""},
    {K2(0xD8, 0x27), "Keydown U Backwards", ""},
    {K2(0xD8, 0x2A), "Home XY", ""},
    {K2(0xD8, 0x2C), "Home Z", ""},
    {K2(0xD8, 0x2D), "Home U", ""},
    {K2(0xD8, 0x2E), "Focus Z", ""},
    {K2(0xD8, 0x30), "Keyup X Left", ""},
    {K2(0xD8, 0x31), "Keyup X Right", ""},
    {K2(0xD8, 0x32), "Keyup Y Top", ""},
    {K2(0xD8, 0x33), "Keyup Y Bottom", ""},
    {K2(0xD8, 0x34), "Keyup Z Up", ""},
    {K2(0xD8, 0x35), "Keyup Z Down", ""},
    {K2(0xD8, 0x36), "Keyup U Forward", ""},
    {K2(0xD8, 0x37), "Keyup U Backwards", ""},

    // --- rapid moves: options byte, then coordinates ---
    {K2(0xD9, 0x00), "Rapid Move X", "ba"},
    {K2(0xD9, 0x01), "Rapid Move Y", "ba"},
    {K2(0xD9, 0x02), "Rapid Move Z", "ba"},
    {K2(0xD9, 0x03), "Rapid Move U", "ba"},
    {K2(0xD9, 0x0F), "Rapid Feed Axis", "b"},  // writer: options only
    {K2(0xD9, 0x10), "Rapid Move XY", "baa"},
    {K2(0xD9, 0x30), "Rapid Move XYU", "baaa"},  // decoder slices x wrongly

    // --- memory ---
    {K2(0xDA, 0x00), "Get Setting", "m"},
    {K2(0xDA, 0x01), "Set Setting / Reply", "muu"},

    // --- documents ---
    {K2(0xE5, 0x00), "Document File Upload", "uu"},
    {K2(0xE5, 0x02), "Document File End", ""},
    {K2(0xE5, 0x05), "Set File Sum", "u"},
    {K2(0xE6, 0x01), "Set Absolute", ""},

    {K2(0xE7, 0x00), "Block End", ""},
    {K2(0xE7, 0x01), "Set Filename", "z"},
    {K2(0xE7, 0x03), "Process Top Left", "aa"},
    {K2(0xE7, 0x04), "Process Repeat", "UUaa"},  // same shape as E7 08; zeros in every file seen
    {K2(0xE7, 0x05), "Array Direction", "b"},
    {K2(0xE7, 0x06), "Feed Repeat", "uu"},
    {K2(0xE7, 0x07), "Process Bottom Right", "aa"},
    // MeerK40t reads E7 08 and F2 05 as seven signed 14-bit words, and its
    // writer emits a constant (1, 1, 0, 1123, -3328, 4, 3480) it cannot explain.
    // In LightBurn's files the 14 bytes are two counts and then the job's
    // width and height as coordinates: 00 01 00 01 | 00 00 0E 0C 6C | 00 00 09
    // 03 1E is 1, 1, 231.020, 147.870 mm (ex/BC_power_test.rd).
    {K2(0xE7, 0x08), "Array Repeat", "UUaa"},
    {K2(0xE7, 0x09), "Feed Length", "u"},
    {K2(0xE7, 0x0A), "Feed Info", "u"},
    {K2(0xE7, 0x0B), "Array En Mirror Cut", "b"},
    {K2(0xE7, 0x0C), "Array Mirror Cut Distance", "b"},
    {K2(0xE7, 0x13), "Array Min Point", "aa"},
    {K2(0xE7, 0x17), "Array Max Point", "aa"},
    {K2(0xE7, 0x23), "Array Add", "aa"},
    {K2(0xE7, 0x24), "Array Mirror", "b"},
    {K2(0xE7, 0x32), "Set Tick Count", "u"},
    {K2(0xE7, 0x35), "Block X Size", "uu"},
    {K2(0xE7, 0x37), "Array Even Distance", "aa"},
    {K2(0xE7, 0x38), "Set Feed Auto Pause", "b"},
    {K2(0xE7, 0x3A), "Union Block Property", ""},
    {K2(0xE7, 0x3B), "Set File Property", "b"},
    {K2(0xE7, 0x46), "BY Test", "u"},
    {K2(0xE7, 0x50), "Document Min Point", "aa"},  // decoder slices at [1:6]
    {K2(0xE7, 0x51), "Document Max Point", "aa"},
    {K2(0xE7, 0x52), "Part Min Point", "baa"},
    {K2(0xE7, 0x53), "Part Max Point", "baa"},
    {K2(0xE7, 0x54), "Pen Offset", "ba"},
    {K2(0xE7, 0x55), "Layer Offset", "ba"},
    {K2(0xE7, 0x57), "PList Feed", ""},
    {K2(0xE7, 0x60), "Set Current Element Index", "b"},
    {K2(0xE7, 0x61), "Part Min Point Ex", "baa"},
    {K2(0xE7, 0x62), "Part Max Point Ex", "baa"},

    {K1(0xEA), "Array Start", "b"},
    {K1(0xEB), "Array End", ""},
    {K1(0xF0), "Ref Point Set", ""},

    {K2(0xF1, 0x00), "Element Max Index", "b"},
    {K2(0xF1, 0x01), "Element Name Max Index", "b"},
    {K2(0xF1, 0x02), "Enable Block Cutting", "b"},
    {K2(0xF1, 0x03), "Display Offset", "aa"},
    {K2(0xF1, 0x04), "Feed Auto Calc", "b"},

    {K2(0xF2, 0x00), "Element Index", "b"},
    {K2(0xF2, 0x01), "Element Name Index", "b"},
    {K2(0xF2, 0x02), "Element Name", "n"},  // fixed 10 bytes, no terminator (LightBurn)
    {K2(0xF2, 0x03), "Element Array Min Point", "aa"},
    {K2(0xF2, 0x04), "Element Array Max Point", "aa"},
    {K2(0xF2, 0x05), "Element Array", "UUaa"},  // as E7 08
    {K2(0xF2, 0x06), "Element Array Add", "aa"},
    {K2(0xF2, 0x07), "Element Array Mirror", "b"},
};

#undef K1
#undef K2
#undef K3

constexpr std::size_t kTableSize = sizeof(kTable) / sizeof(kTable[0]);

std::size_t field_width(char kind) {
    switch (kind) {
        case 'n': return 10;
        case 'a': case 's': case 't': case 'f': case 'u': case 'c': return 5;
        case 'r': case 'p': case 'U': case 'w': case 'm': return 2;
        case 'b': return 1;
        default: return 0;  // 'z' is variable
    }
}

const RdSpec* lookup(std::string_view cmd) {
    // Longest key wins, so CA 01 00 is found before anything keyed CA 01.
    const RdSpec* best = nullptr;
    for (const RdSpec& s : kTable) {
        if (s.key_len > cmd.size()) continue;
        bool match = true;
        for (std::uint8_t k = 0; k < s.key_len; ++k) {
            if (static_cast<std::uint8_t>(cmd[k]) != s.key[k]) {
                match = false;
                break;
            }
        }
        if (match && (!best || s.key_len > best->key_len)) best = &s;
    }
    return best;
}

void decode_fields(RdCommand& c) {
    const auto* p = reinterpret_cast<const std::uint8_t*>(c.raw.data());
    std::size_t at = c.spec->key_len;
    const std::size_t n = c.raw.size();
    for (const char* a = c.spec->args; *a; ++a) {
        const char kind = *a;
        RdField f;
        f.kind = kind;
        if (kind == 'z') {
            while (at < n && p[at] != 0) f.s.push_back(static_cast<char>(p[at++]));
            if (at < n) {
                ++at;  // the terminator
            } else {
                c.length_ok = false;
            }
            c.fields.push_back(std::move(f));
            continue;
        }
        const std::size_t w = field_width(kind);
        if (at + w > n) {
            c.length_ok = false;
            return;
        }
        switch (kind) {
            case 'a': f.i = get_abs_coord(p + at); break;
            case 'r': f.i = get_s14(p + at); break;
            case 'p': f.i = get_u14(p + at); f.d = power_from_raw(static_cast<std::uint32_t>(f.i)); break;
            case 's': f.i = static_cast<std::int64_t>(get_u35(p + at)); f.d = static_cast<double>(f.i) / 1000.0; break;
            case 't': case 'f': case 'u': case 'c': f.i = static_cast<std::int64_t>(get_u35(p + at)); break;
            case 'U': f.i = get_u14(p + at); break;
            case 'w': f.i = get_s14(p + at); break;
            case 'b': f.i = p[at]; break;
            case 'n':
                for (std::size_t k = 0; k < w; ++k) f.s.push_back(static_cast<char>(p[at + k]));
                break;
            case 'm': {
                char buf[8];
                std::snprintf(buf, sizeof buf, "%02X %02X", p[at], p[at + 1]);
                f.s = buf;
                f.i = (p[at] << 8) | p[at + 1];
                break;
            }
            default: break;
        }
        at += w;
        c.fields.push_back(std::move(f));
    }
    if (at != n) c.length_ok = false;
}

void append(std::string& out, const char* fmt, double v) {
    char buf[64];
    std::snprintf(buf, sizeof buf, fmt, v);
    out += buf;
}

void append_mm(std::string& out, std::int64_t um, bool sign) {
    // Exact: integer micrometres printed as mm with three decimals.
    const bool neg = um < 0;
    const std::uint64_t a = neg ? static_cast<std::uint64_t>(-um) : static_cast<std::uint64_t>(um);
    char buf[48];
    std::snprintf(buf, sizeof buf, "%s%llu.%03llu", neg ? "-" : (sign ? "+" : ""),
                  static_cast<unsigned long long>(a / 1000), static_cast<unsigned long long>(a % 1000));
    out += buf;
}

std::string format_field(const RdField& f) {
    std::string s;
    char buf[64];
    switch (f.kind) {
        case 'a': append_mm(s, f.i, false); break;
        case 'r': append_mm(s, f.i, true); break;
        case 'p': append(s, "%.2f%%", f.d); break;
        case 's': append(s, "%.3fmm/s", f.d); break;
        case 'c':
            std::snprintf(buf, sizeof buf, "#%02X%02X%02X", static_cast<unsigned>(f.i & 0xFF),
                          static_cast<unsigned>((f.i >> 8) & 0xFF), static_cast<unsigned>((f.i >> 16) & 0xFF));
            s = buf;
            break;
        case 'm': s = "mem " + f.s; break;
        case 'z': s = "\"" + f.s + "\""; break;
        case 'n':
            for (char ch : f.s) {
                std::snprintf(buf, sizeof buf, s.empty() ? "%02X" : " %02X", static_cast<unsigned char>(ch));
                s += buf;
            }
            break;
        case 't': case 'f':
            std::snprintf(buf, sizeof buf, "raw=%lld", static_cast<long long>(f.i));
            s = buf;
            break;
        default:
            std::snprintf(buf, sizeof buf, "%lld", static_cast<long long>(f.i));
            s = buf;
            break;
    }
    return s;
}

bool key_is(const RdCommand& c, std::uint8_t a, std::uint8_t b) {
    return c.spec && c.spec->key_len == 2 && c.spec->key[0] == a && c.spec->key[1] == b;
}

}  // namespace

const RdSpec* rd_spec_table(std::size_t* count) {
    if (count) *count = kTableSize;
    return kTable;
}

std::vector<RdCommand> decode_rd(std::string_view plain) {
    std::vector<RdCommand> out;
    std::size_t offset = 0;
    for (std::string_view raw : split_commands(plain)) {
        RdCommand c;
        c.offset = offset;
        c.raw = raw;
        offset += raw.size();
        if (static_cast<std::uint8_t>(raw[0]) < 0x80) {
            c.stray = true;
            c.length_ok = false;
        } else {
            c.spec = lookup(raw);
            if (c.spec) decode_fields(c);
        }
        out.push_back(std::move(c));
    }
    return out;
}

RdMotion motion_of(const RdCommand& c, char* axis) {
    if (axis) *axis = 'b';
    if (!c.spec || !c.length_ok || c.spec->key_len != 1) return RdMotion::None;
    switch (c.spec->key[0]) {
        case 0x88: return RdMotion::MoveAbs;
        case 0x89: return RdMotion::MoveRel;
        case 0x8A: if (axis) *axis = 'x'; return RdMotion::MoveRel;
        case 0x8B: if (axis) *axis = 'y'; return RdMotion::MoveRel;
        case 0xA8: return RdMotion::CutAbs;
        case 0xA9: return RdMotion::CutRel;
        case 0xAA: if (axis) *axis = 'x'; return RdMotion::CutRel;
        case 0xAB: if (axis) *axis = 'y'; return RdMotion::CutRel;
        default: return RdMotion::None;
    }
}

namespace {

// Applies a motion to (x, y) in um. Returns false for a non-motion command.
bool apply_motion(const RdCommand& c, std::int64_t& x, std::int64_t& y, bool* cut, bool* absolute) {
    char axis = 'b';
    const RdMotion m = motion_of(c, &axis);
    if (m == RdMotion::None) return false;
    *cut = (m == RdMotion::CutAbs || m == RdMotion::CutRel);
    *absolute = (m == RdMotion::MoveAbs || m == RdMotion::CutAbs);
    if (*absolute) {
        x = c.fields[0].i;
        y = c.fields[1].i;
    } else if (axis == 'x') {
        x += c.fields[0].i;
    } else if (axis == 'y') {
        y += c.fields[0].i;
    } else {
        x += c.fields[0].i;
        y += c.fields[1].i;
    }
    return true;
}

}  // namespace

namespace {

const char* corner_name(std::int64_t min_x, std::int64_t min_y, std::int64_t max_x, std::int64_t max_y) {
    const bool left = min_x == 0, right = max_x == 0;
    const bool bottom = min_y == 0, top = max_y == 0;
    if (top && left) return "top-left";
    if (top && right) return "top-right";
    if (bottom && left) return "bottom-left";
    if (bottom && right) return "bottom-right";
    return "other";
}

}  // namespace

RdSummary summarize_rd(const std::vector<RdCommand>& cmds, const RdMachine& machine) {
    RdSummary s;
    bool have_tl = false, have_br = false, have_dmin = false;
    std::int64_t dmin_x = 0, dmin_y = 0;
    std::int64_t x = 0, y = 0;
    bool seen_abs = false;
    std::uint64_t running_sum = 0;
    for (const RdCommand& c : cmds) {
        ++s.commands;
        if (c.stray) ++s.stray;
        else if (!c.spec) ++s.unknown;
        if (!c.length_ok && !c.stray) ++s.bad_length;

        if (c.length_ok && key_is(c, 0xE7, 0x03) && !have_tl) {
            s.box_min_x = c.fields[0].i;
            s.box_min_y = c.fields[1].i;
            have_tl = true;
        }
        if (c.length_ok && key_is(c, 0xE7, 0x07) && !have_br) {
            s.box_max_x = c.fields[0].i;
            s.box_max_y = c.fields[1].i;
            have_br = true;
        }
        if (c.length_ok && key_is(c, 0xE7, 0x50) && !have_dmin) {
            dmin_x = c.fields[0].i;
            dmin_y = c.fields[1].i;
            have_dmin = true;
        }
        if (key_is(c, 0xE5, 0x05) && c.length_ok && !s.has_file_sum) {
            s.has_file_sum = true;
            s.file_sum_stated = static_cast<std::uint64_t>(c.fields[0].i);
            s.file_sum_computed = running_sum + 0xD7;
        }
        if (c.spec && c.spec->key_len == 1 && c.spec->key[0] == 0xD7) s.has_eof = true;
        running_sum += byte_sum(c.raw);

        const std::int64_t px = x, py = y;
        bool cut = false, absolute = false;
        if (!apply_motion(c, x, y, &cut, &absolute)) continue;
        if (!absolute && !seen_abs) s.rel_before_abs = true;
        if (absolute) seen_abs = true;
        const double len = std::hypot(static_cast<double>(x - px), static_cast<double>(y - py)) / 1000.0;
        if (cut) {
            ++s.cuts;
            s.cut_mm += len;
            const double xs[2] = {static_cast<double>(px) / 1000.0, static_cast<double>(x) / 1000.0};
            const double ys[2] = {static_cast<double>(py) / 1000.0, static_cast<double>(y) / 1000.0};
            for (int k = 0; k < 2; ++k) {
                if (!s.have_cut_bounds) {
                    s.cut_min_x = s.cut_max_x = xs[k];
                    s.cut_min_y = s.cut_max_y = ys[k];
                    s.have_cut_bounds = true;
                } else {
                    s.cut_min_x = std::fmin(s.cut_min_x, xs[k]);
                    s.cut_max_x = std::fmax(s.cut_max_x, xs[k]);
                    s.cut_min_y = std::fmin(s.cut_min_y, ys[k]);
                    s.cut_max_y = std::fmax(s.cut_max_y, ys[k]);
                }
            }
        } else {
            ++s.moves;
            s.travel_mm += len;
        }
    }
    if (have_tl && have_br && have_dmin) {
        s.has_origin = true;
        s.origin_x = dmin_x - s.box_min_x;
        s.origin_y = dmin_y - s.box_min_y;
        s.origin_corner_file = corner_name(s.box_min_x, s.box_min_y, s.box_max_x, s.box_max_y);
        // File displacement = sign * design displacement, so the design box
        // is the file box with the mirrored axes' ends swapped.
        const std::int64_t sx = machine.sign_x(), sy = machine.sign_y();
        const std::int64_t dx0 = sx > 0 ? s.box_min_x : -s.box_max_x, dx1 = sx > 0 ? s.box_max_x : -s.box_min_x;
        const std::int64_t dy0 = sy > 0 ? s.box_min_y : -s.box_max_y, dy1 = sy > 0 ? s.box_max_y : -s.box_min_y;
        s.origin_corner = corner_name(dx0, dy0, dx1, dy1);
    }
    return s;
}

std::string rd_to_text(std::string_view plain, const RdTextOptions& opts) {
    const std::vector<RdCommand> cmds = decode_rd(plain);
    std::string out;
    std::int64_t x = 0, y = 0;
    char buf[64];

    for (const RdCommand& c : cmds) {
        std::string line;
        if (opts.offsets) {
            std::snprintf(buf, sizeof buf, "%06zx  ", c.offset);
            line += buf;
        }
        if (opts.hex) {
            std::string hex;
            for (std::size_t i = 0; i < c.raw.size(); ++i) {
                std::snprintf(buf, sizeof buf, i ? " %02X" : "%02X", static_cast<std::uint8_t>(c.raw[i]));
                hex += buf;
            }
            if (hex.size() < 32) hex.append(32 - hex.size(), ' ');
            line += hex + "  ";
        }
        if (c.stray) {
            line += "?? stray argument bytes";
        } else if (!c.spec) {
            line += "?? unknown command";
        } else {
            line += c.spec->name;
            std::string args;
            for (std::size_t i = 0; i < c.fields.size(); ++i) {
                args += (i ? ", " : "") + format_field(c.fields[i]);
            }
            if (!args.empty()) line += " " + args;
            if (!c.length_ok) line += "  !! length does not match layout";
            bool cut = false, absolute = false;
            if (apply_motion(c, x, y, &cut, &absolute) && opts.positions) {
                line += "  -> ";
                append_mm(line, x, false);
                line += ",";
                append_mm(line, y, false);
            }
        }
        out += line;
        out += '\n';
    }

    if (opts.summary) {
        const RdSummary s = summarize_rd(cmds, opts.machine);
        out += "\n; ---- summary ----\n";
        std::snprintf(buf, sizeof buf, "; commands %zu", s.commands);
        out += buf;
        std::snprintf(buf, sizeof buf, ", unknown %zu, bad length %zu, stray %zu\n", s.unknown, s.bad_length, s.stray);
        out += buf;
        std::snprintf(buf, sizeof buf, "; cuts %zu, %.3f mm", s.cuts, s.cut_mm);
        out += buf;
        std::snprintf(buf, sizeof buf, "; moves %zu, %.3f mm\n", s.moves, s.travel_mm);
        out += buf;
        if (s.have_cut_bounds) {
            out += "; cut bounds ";
            append(out, "%.3f", s.cut_min_x);
            out += ",";
            append(out, "%.3f", s.cut_min_y);
            out += " .. ";
            append(out, "%.3f", s.cut_max_x);
            out += ",";
            append(out, "%.3f", s.cut_max_y);
            out += " mm\n";
        }
        if (s.has_origin) {
            out += "; origin ";
            append_mm(out, s.origin_x, false);
            out += ",";
            append_mm(out, s.origin_y, false);
            out += std::string(" (") + s.origin_corner + "), box ";
            append_mm(out, s.box_min_x, false);
            out += ",";
            append_mm(out, s.box_min_y, false);
            out += " .. ";
            append_mm(out, s.box_max_x, false);
            out += ",";
            append_mm(out, s.box_max_y, false);
            out += " mm relative to it\n";
        }
        if (s.rel_before_abs) out += "; WARNING relative motion before any absolute one\n";
        if (s.has_file_sum) {
            std::snprintf(buf, sizeof buf, "; file sum %llu, computed %llu: %s\n",
                          static_cast<unsigned long long>(s.file_sum_stated),
                          static_cast<unsigned long long>(s.file_sum_computed),
                          s.file_sum_stated == s.file_sum_computed ? "ok" : "MISMATCH");
            out += buf;
        } else {
            out += "; no file sum\n";
        }
        out += s.has_eof ? "; end of file present\n" : "; WARNING no end-of-file command\n";
    }
    return out;
}

}  // namespace ncad::laser::ruida
