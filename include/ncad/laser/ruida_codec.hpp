// SPDX-License-Identifier: BSD-3-Clause AND MIT
// Copyright (c) 2026, Sadie Forbes
// Copyright (c) 2021 meerk40t
//
// Ported from meerk40t/ruida/rdjob.py (value encoders, swizzle) and
// meerk40t/ruida/udp_connection.py (packet framing). See
// third_party/meerk40t/NOTICE.

// The Ruida wire primitives: byte swizzling, 7-bit number packing, and UDP
// packet framing. Pure functions over bytes; no I/O.
//
// **Numbers are 7 bits per byte, most significant first.** The high bit of
// every byte is reserved: a byte >= 0x80 begins a command and a byte < 0x80 is
// an argument, which is what lets a stream be split into commands with no
// length fields (`split_commands`). A 14-bit value takes two bytes and a
// 35-bit value five.
//
// **Swizzling is a per-controller-model byte permutation**, keyed by a "magic"
// byte (0x88 on most 644x boards). Everything sent to the controller and
// everything in an .rd file is swizzled; replies come back swizzled too. It is
// a bijection, so it is two 256-entry tables.
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace ncad::laser::ruida {

// --- swizzling ---------------------------------------------------------------

inline constexpr std::uint8_t kDefaultMagic = 0x88;

std::uint8_t swizzle_byte(std::uint8_t b, std::uint8_t magic);
std::uint8_t unswizzle_byte(std::uint8_t b, std::uint8_t magic);

class Swizzle {
public:
    explicit Swizzle(std::uint8_t magic = kDefaultMagic);

    std::uint8_t magic() const { return magic_; }

    std::string encode(std::string_view plain) const;
    std::string decode(std::string_view swizzled) const;

private:
    std::uint8_t magic_;
    std::array<std::uint8_t, 256> fwd_{};
    std::array<std::uint8_t, 256> inv_{};
};

// Guesses the magic of a swizzled .rd file. In real files about half of all
// plain bytes are 0x00, and swizzle(0x00) == magic + 1 for every magic -- so the
// most frequent byte, minus one, is almost certainly the magic. Runs of the
// same byte count extra, as MeerK40t's does. A header that decodes to
// `D8 10|11|12 F0` (Ref Point, Ref Point Set) under 0x88 -- or under exactly
// one magic -- is taken first: long raster streams defeat the histogram (r01 to
// r03 guess 0xA3 and are 0x88). Returns false on empty input.
bool guess_magic(std::string_view swizzled, std::uint8_t& magic);

// --- 7-bit numbers -----------------------------------------------------------

// Range limits of the signed forms.
inline constexpr std::int32_t kS14Min = -8192;
inline constexpr std::int32_t kS14Max = 8191;

// Unsigned encoders mask to their width; callers range-check first where a
// silent wrap would matter (see the value encoders below, which do).
void put_u14(std::string& out, std::uint32_t v);
void put_u35(std::string& out, std::uint64_t v);

// Two's complement in the field's width.
void put_s14(std::string& out, std::int32_t v);
void put_s35(std::string& out, std::int64_t v);

// Decoders read exactly 2 or 5 bytes from `p`; the caller guarantees length.
std::uint32_t get_u14(const std::uint8_t* p);
std::int32_t get_s14(const std::uint8_t* p);
std::uint64_t get_u35(const std::uint8_t* p);
std::int64_t get_s35(const std::uint8_t* p);

// --- value encoders ----------------------------------------------------------
//
// Each returns false, leaving `out` untouched, when the value cannot be
// represented. Nothing is clamped: a coordinate silently wrapped by a field
// width is a head that goes somewhere nobody drew.

// Absolute coordinate, micrometres: signed 32-bit two's complement carried in
// the 35-bit field with its top three bits zero, as LightBurn writes it.
bool put_abs_coord(std::string& out, std::int64_t um);
std::int32_t get_abs_coord(const std::uint8_t* p);

// Relative coordinate, micrometres, 14-bit signed field: -8192..8191 um.
// (MeerK40t's `jump`/`mark` test `abs(d) > 8192`, which lets +8192 through to
// wrap to -8192. The range here is the field's real one.)
bool put_rel_coord(std::string& out, std::int32_t um);
bool fits_rel(std::int64_t um);

// Power, percent 0..100, as a 14-bit fraction of 16383, rounded to nearest.
// Confirmed against LightBurn: 8% is 0A 1F (1311) and 12% is 0F 2E (1966);
// MeerK40t truncates and would write 1310 and 1965.
bool put_power(std::string& out, double percent);
double power_from_raw(std::uint32_t raw);

// Speed, mm/s, sent as um/s in a 35-bit field.
bool put_speed(std::string& out, double mm_per_s);

// --- commands and packets ----------------------------------------------------

// Splits a plain (unswizzled) stream into commands at every byte >= 0x80.
std::vector<std::string_view> split_commands(std::string_view plain);

// Sum of plain bytes, as the .rd file-sum command wants it.
std::uint64_t byte_sum(std::string_view bytes);

// One UDP datagram: a big-endian 16-bit sum of the SWIZZLED payload bytes,
// then the swizzled payload.
std::string package_udp(const Swizzle& sw, std::string_view plain);

// The inverse, for a datagram SENT TO a controller -- an emulator receiving
// ours, or a captured packet. Replies FROM the controller carry no checksum and
// are just swizzled (see udp_connection.py), so they go through
// Swizzle::decode instead. False when too short or the sum does not match;
// `plain` is then untouched.
bool unpackage_udp(const Swizzle& sw, std::string_view datagram, std::string& plain);

}  // namespace ncad::laser::ruida
