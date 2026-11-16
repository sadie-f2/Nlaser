// SPDX-License-Identifier: BSD-3-Clause AND MIT
// Copyright (c) 2026, Sadie Forbes
// Copyright (c) 2021 meerk40t
//
// The command table is ported from meerk40t/ruida/rdjob.py (the command
// constants and RDJob.process). See third_party/meerk40t/NOTICE.

// Decoding a plain (unswizzled) Ruida stream into commands, and printing it.
//
// This is the debugging tool every later stage leans on: an .rd file from
// LightBurn and one from our encoder, both run through `rd_to_text`, can be
// diffed line by line. So the decoder is TABLE-DRIVEN and STRICT:
//
// - Each known command has an argument layout, and a command whose length does
//   not match its layout is flagged, not quietly half-read. MeerK40t's
//   decoder reads several commands at the wrong offsets (E7 50, D9 30); a
//   length check is what catches that class of mistake.
// - Unknown commands are printed with their bytes and counted, never dropped.
// - The file sum (E5 05) is recomputed and compared, which tests our reading
//   of how it is formed against any file that carries one.
//
// Units printed are the ones the table is sure of: coordinates in mm (exact,
// from integer um), power in %, speed in mm/s. Time and frequency fields are
// printed RAW, because MeerK40t encodes and decodes them inconsistently and
// LightBurn's files have not yet said which reading is right.
#pragma once

#include "ncad/laser/ruida_machine.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace ncad::laser::ruida {

// One entry in the command table.
//
// `key` is 1 to 3 leading bytes. `args` is the layout after the key, one
// character per field:
//   a  absolute coordinate, 5 bytes, signed 32-bit um
//   r  relative coordinate, 2 bytes, signed um
//   p  power, 2 bytes, %
//   s  speed, 5 bytes, mm/s (stored as um/s)
//   t  time, 5 bytes, raw
//   f  frequency, 5 bytes, raw
//   u  unsigned 35-bit, 5 bytes, raw
//   U  unsigned 14-bit, 2 bytes, raw
//   w  signed 14-bit, 2 bytes, raw
//   b  one raw byte (part number, flag, option)
//   c  colour, 5 bytes, 0xBBGGRR
//   m  memory address, 2 raw bytes
//   n  10 raw bytes (element name; LightBurn writes no terminator)
//   z  string, up to and including a 0x00 terminator
struct RdSpec {
    std::uint8_t key[3];
    std::uint8_t key_len;
    const char* name;
    const char* args;
};

// The kinds of motion, for position tracking.
enum class RdMotion : std::uint8_t { None, MoveAbs, MoveRel, CutAbs, CutRel };

struct RdField {
    char kind{'\0'};
    std::int64_t i{0};  // integer value (um for coordinates, raw otherwise)
    double d{0.0};      // power %, speed mm/s
    std::string s;      // strings and memory addresses
};

struct RdCommand {
    std::size_t offset{0};
    std::string_view raw;          // a view into the stream that was decoded
    const RdSpec* spec{nullptr};   // null for an unknown command
    std::vector<RdField> fields;
    bool length_ok{true};          // false: shorter or longer than the layout
    bool stray{false};             // argument bytes before the first command
};

// Decodes every command in `plain`. The returned commands hold views into
// `plain`, which must outlive them.
std::vector<RdCommand> decode_rd(std::string_view plain);

// Motion kind and the axes a motion command moves along: for MoveRel/CutRel
// with one field, `axis` says which ('x' or 'y'); otherwise 'b' for both.
RdMotion motion_of(const RdCommand& c, char* axis);

struct RdSummary {
    std::size_t commands{0};
    std::size_t unknown{0};
    std::size_t bad_length{0};
    std::size_t stray{0};

    std::size_t cuts{0};      // cut commands
    std::size_t moves{0};     // move (laser off) commands
    double cut_mm{0.0};       // total cut length
    double travel_mm{0.0};    // total move length
    bool have_cut_bounds{false};
    double cut_min_x{0.0}, cut_min_y{0.0}, cut_max_x{0.0}, cut_max_y{0.0};

    // A relative motion before any absolute one: the controller's start
    // position is whatever it happens to be. MeerK40t forces the first motion
    // to be absolute for exactly this reason.
    bool rel_before_abs{false};

    // The job's origin point, from the header. LightBurn writes the process
    // box (E7 03 / E7 07) RELATIVE to the origin its 9-dot setting picks, and
    // Document Min Point (E7 50) in the same frame as the moves, so
    //     origin = Document Min Point - Process Top Left
    // in every file seen. Which process-box coordinates are zero says which
    // corner the origin is: e.g. top-left is x in 0..W, y in -H..0.
    bool has_origin{false};
    std::int64_t origin_x{0}, origin_y{0};          // um, in the moves' frame
    std::int64_t box_min_x{0}, box_min_y{0};        // process box, um,
    std::int64_t box_max_x{0}, box_max_y{0};        // relative to the origin
    // The origin's corner in DESIGN terms (X right, Y up) for the machine given
    // to summarize_rd, e.g. "top-left"; "other" when it is not on a corner.
    const char* origin_corner{"?"};
    // The same, read straight off the file's box with no machine mapping: what
    // a bottom-left machine would call it. Equal to origin_corner by default.
    const char* origin_corner_file{"?"};

    bool has_file_sum{false};
    std::uint64_t file_sum_stated{0};
    std::uint64_t file_sum_computed{0};  // bytes before E5 05, plus 0xD7 (EOF)
    bool has_eof{false};
};

// `machine` says how the file's axes relate to the design's, so the origin
// corner can be named in design terms. The default is the identity, which is
// right for files from a bottom-left-origin machine.
RdSummary summarize_rd(const std::vector<RdCommand>& cmds, const RdMachine& machine = RdMachine{});

struct RdTextOptions {
    bool offsets{true};    // leading file offset
    bool hex{true};        // the command's bytes
    bool positions{true};  // head position after each motion
    bool summary{true};    // the summary block at the end
    RdMachine machine;     // for naming the origin corner in the summary
};

std::string rd_to_text(std::string_view plain, const RdTextOptions& opts);

// The table itself, for tests and for anyone extending it.
const RdSpec* rd_spec_table(std::size_t* count);

}  // namespace ncad::laser::ruida
