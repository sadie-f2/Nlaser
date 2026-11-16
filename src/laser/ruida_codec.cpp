// SPDX-License-Identifier: BSD-3-Clause AND MIT
// Copyright (c) 2026, Sadie Forbes
// Copyright (c) 2021 meerk40t
//
// Ported from meerk40t/ruida/rdjob.py and meerk40t/ruida/udp_connection.py.
// See third_party/meerk40t/NOTICE.

#include "ncad/laser/ruida_codec.hpp"

#include <cmath>
#include <limits>

namespace ncad::laser::ruida {

// --- swizzling ---------------------------------------------------------------

std::uint8_t swizzle_byte(std::uint8_t b, std::uint8_t magic) {
    unsigned v = b;
    // Swap bit 7 and bit 0 (three XORs), then XOR the magic and add one.
    v ^= (v >> 7) & 0xFFu;
    v ^= (v << 7) & 0xFFu;
    v ^= (v >> 7) & 0xFFu;
    v ^= magic;
    v = (v + 1u) & 0xFFu;
    return static_cast<std::uint8_t>(v);
}

std::uint8_t unswizzle_byte(std::uint8_t b, std::uint8_t magic) {
    unsigned v = b;
    v = (v - 1u) & 0xFFu;
    v ^= magic;
    v ^= (v >> 7) & 0xFFu;
    v ^= (v << 7) & 0xFFu;
    v ^= (v >> 7) & 0xFFu;
    return static_cast<std::uint8_t>(v);
}

Swizzle::Swizzle(std::uint8_t magic) : magic_(magic) {
    for (unsigned i = 0; i < 256; ++i) {
        const auto b = static_cast<std::uint8_t>(i);
        fwd_[i] = swizzle_byte(b, magic);
        inv_[i] = unswizzle_byte(b, magic);
    }
}

std::string Swizzle::encode(std::string_view plain) const {
    std::string out(plain.size(), '\0');
    for (std::size_t i = 0; i < plain.size(); ++i) {
        out[i] = static_cast<char>(fwd_[static_cast<std::uint8_t>(plain[i])]);
    }
    return out;
}

std::string Swizzle::decode(std::string_view swizzled) const {
    std::string out(swizzled.size(), '\0');
    for (std::size_t i = 0; i < swizzled.size(); ++i) {
        out[i] = static_cast<char>(inv_[static_cast<std::uint8_t>(swizzled[i])]);
    }
    return out;
}

bool guess_magic(std::string_view swizzled, std::uint8_t& magic) {
    if (swizzled.empty()) return false;
    std::array<std::uint64_t, 256> hist{};
    int prev = -1;
    for (char c : swizzled) {
        const auto b = static_cast<std::uint8_t>(c);
        hist[b] += (prev == b) ? 5u : 1u;
        prev = b;
    }
    unsigned best = 0;
    for (unsigned i = 1; i < 256; ++i) {
        if (hist[i] > hist[best]) best = i;
    }
    magic = static_cast<std::uint8_t>((best - 1u) & 0xFFu);
    return true;
}

// --- 7-bit numbers -----------------------------------------------------------

void put_u14(std::string& out, std::uint32_t v) {
    out.push_back(static_cast<char>((v >> 7) & 0x7Fu));
    out.push_back(static_cast<char>(v & 0x7Fu));
}

void put_u35(std::string& out, std::uint64_t v) {
    for (int shift = 28; shift >= 0; shift -= 7) {
        out.push_back(static_cast<char>((v >> shift) & 0x7Fu));
    }
}

void put_s14(std::string& out, std::int32_t v) {
    put_u14(out, static_cast<std::uint32_t>(v) & 0x3FFFu);
}

void put_s35(std::string& out, std::int64_t v) {
    put_u35(out, static_cast<std::uint64_t>(v) & 0x7FFFFFFFFull);
}

std::uint32_t get_u14(const std::uint8_t* p) {
    return (static_cast<std::uint32_t>(p[0] & 0x7Fu) << 7) | (p[1] & 0x7Fu);
}

std::int32_t get_s14(const std::uint8_t* p) {
    const std::uint32_t u = get_u14(p);
    return (u & 0x2000u) ? static_cast<std::int32_t>(u) - 0x4000 : static_cast<std::int32_t>(u);
}

std::uint64_t get_u35(const std::uint8_t* p) {
    std::uint64_t v = 0;
    for (int i = 0; i < 5; ++i) v = (v << 7) | (p[i] & 0x7Fu);
    return v;
}

std::int64_t get_s35(const std::uint8_t* p) {
    const std::uint64_t u = get_u35(p);
    return (u & 0x400000000ull) ? static_cast<std::int64_t>(u) - 0x800000000ll
                                : static_cast<std::int64_t>(u);
}

// --- value encoders ----------------------------------------------------------

bool put_abs_coord(std::string& out, std::int64_t um) {
    // Signed 32-bit two's complement in the 35-bit field, top three bits
    // zero. That is what LightBurn writes: -3.204 mm is 0F 7F 7F 7C 0C, not
    // the 35-bit form 7F 7F 7F 7C 0C (ex/BC_power_test.rd, Document Min Point).
    if (um < std::numeric_limits<std::int32_t>::min() ||
        um > std::numeric_limits<std::int32_t>::max()) {
        return false;
    }
    put_u35(out, static_cast<std::uint32_t>(static_cast<std::int32_t>(um)));
    return true;
}

std::int32_t get_abs_coord(const std::uint8_t* p) {
    return static_cast<std::int32_t>(static_cast<std::uint32_t>(get_u35(p) & 0xFFFFFFFFull));
}

bool fits_rel(std::int64_t um) { return um >= kS14Min && um <= kS14Max; }

bool put_rel_coord(std::string& out, std::int32_t um) {
    if (!fits_rel(um)) return false;
    put_s14(out, um);
    return true;
}

bool put_power(std::string& out, double percent) {
    if (!(percent >= 0.0 && percent <= 100.0)) return false;  // also rejects NaN
    const auto raw = static_cast<std::uint32_t>(std::lround(percent * 16383.0 / 100.0));
    put_u14(out, raw);
    return true;
}

double power_from_raw(std::uint32_t raw) { return static_cast<double>(raw) * 100.0 / 16383.0; }

bool put_speed(std::string& out, double mm_per_s) {
    if (!(mm_per_s >= 0.0)) return false;
    const double um = std::round(mm_per_s * 1000.0);
    if (um > static_cast<double>(0x7FFFFFFFFull)) return false;
    put_u35(out, static_cast<std::uint64_t>(um));
    return true;
}

// --- commands and packets ----------------------------------------------------

std::vector<std::string_view> split_commands(std::string_view plain) {
    std::vector<std::string_view> cmds;
    std::size_t mark = 0;
    for (std::size_t i = 0; i < plain.size(); ++i) {
        if (static_cast<std::uint8_t>(plain[i]) >= 0x80u && i != mark) {
            cmds.push_back(plain.substr(mark, i - mark));
            mark = i;
        }
    }
    if (mark != plain.size()) cmds.push_back(plain.substr(mark));
    return cmds;
}

std::uint64_t byte_sum(std::string_view bytes) {
    std::uint64_t s = 0;
    for (char c : bytes) s += static_cast<std::uint8_t>(c);
    return s;
}

std::string package_udp(const Swizzle& sw, std::string_view plain) {
    const std::string body = sw.encode(plain);
    const auto sum = static_cast<std::uint16_t>(byte_sum(body) & 0xFFFFu);
    std::string out;
    out.reserve(body.size() + 2);
    out.push_back(static_cast<char>(sum >> 8));
    out.push_back(static_cast<char>(sum & 0xFFu));
    out += body;
    return out;
}

bool unpackage_udp(const Swizzle& sw, std::string_view datagram, std::string& plain) {
    if (datagram.size() < 2) return false;
    const auto hi = static_cast<std::uint8_t>(datagram[0]);
    const auto lo = static_cast<std::uint8_t>(datagram[1]);
    const auto want = static_cast<std::uint16_t>((hi << 8) | lo);
    const std::string_view body = datagram.substr(2);
    if (static_cast<std::uint16_t>(byte_sum(body) & 0xFFFFu) != want) return false;
    plain = sw.decode(body);
    return true;
}

}  // namespace ncad::laser::ruida
