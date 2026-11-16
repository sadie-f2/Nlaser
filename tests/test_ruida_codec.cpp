// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// Golden bytes for the Ruida wire primitives.
//
// Every expected value below was produced by MeerK40t's own functions
// (meerk40t/ruida/rdjob.py at d511944), run on the same inputs. Where this port
// deliberately differs, the case says so and gives MeerK40t's bytes too.

#include "test.hpp"

#include "ncad/laser/ruida_codec.hpp"

#include <string>

using namespace ncad::laser::ruida;

namespace {

std::string bytes(std::initializer_list<unsigned> list) {
    std::string s;
    for (unsigned b : list) s.push_back(static_cast<char>(b));
    return s;
}

const std::uint8_t* u8(const std::string& s) {
    return reinterpret_cast<const std::uint8_t*>(s.data());
}

}  // namespace

TEST_CASE("ruida: swizzle bytes, magic 0x88") {
    const unsigned in[] = {0x00, 0x01, 0x7F, 0x80, 0xCC, 0xDA, 0xFF};
    const unsigned out[] = {0x89, 0x09, 0x77, 0x8A, 0xC6, 0xD4, 0x78};
    for (int i = 0; i < 7; ++i) {
        CHECK(swizzle_byte(static_cast<std::uint8_t>(in[i]), 0x88) == out[i]);
    }
}

TEST_CASE("ruida: swizzle bytes, magic 0x11") {
    const unsigned in[] = {0x00, 0x01, 0x7F, 0x80, 0xCC, 0xDA, 0xFF};
    const unsigned out[] = {0x12, 0x92, 0xF0, 0x11, 0x5D, 0x4B, 0xEF};
    for (int i = 0; i < 7; ++i) {
        CHECK(swizzle_byte(static_cast<std::uint8_t>(in[i]), 0x11) == out[i]);
    }
}

TEST_CASE("ruida: swizzle is a bijection for every magic") {
    for (unsigned m = 0; m < 256; ++m) {
        const Swizzle sw(static_cast<std::uint8_t>(m));
        bool seen[256] = {};
        for (unsigned b = 0; b < 256; ++b) {
            const std::string one(1, static_cast<char>(b));
            const std::string enc = sw.encode(one);
            const auto e = static_cast<std::uint8_t>(enc[0]);
            CHECK(!seen[e]);
            seen[e] = true;
            CHECK(sw.decode(enc) == one);
        }
    }
}

TEST_CASE("ruida: guess_magic finds the magic from a zero-heavy stream") {
    // Shaped like a real file: a command byte then mostly-zero arguments.
    std::string plain;
    for (int i = 0; i < 200; ++i) {
        plain += bytes({0x88, 0x00, 0x00, 0x00, 0x12, 0x34, 0x00, 0x00, 0x05, 0x10, 0x20});
    }
    for (unsigned m : {0x88u, 0x11u, 0x38u}) {
        std::uint8_t got = 0;
        REQUIRE(guess_magic(Swizzle(static_cast<std::uint8_t>(m)).encode(plain), got));
        CHECK(got == m);
    }
    std::uint8_t unused = 0;
    CHECK(!guess_magic("", unused));
}

TEST_CASE("ruida: 35-bit numbers") {
    std::string s;
    put_u35(s, 12345678);
    CHECK(s == bytes({0x00, 0x05, 0x71, 0x42, 0x4E}));

    s.clear();
    put_s35(s, -1);
    CHECK(s == bytes({0x7F, 0x7F, 0x7F, 0x7F, 0x7F}));
    CHECK(get_s35(u8(s)) == -1);

    s.clear();
    put_s35(s, -50000);
    CHECK(s == bytes({0x7F, 0x7F, 0x7C, 0x79, 0x30}));
    CHECK(get_s35(u8(s)) == -50000);

    CHECK(get_u35(u8(bytes({0x00, 0x05, 0x71, 0x42, 0x4E}))) == 12345678u);
}

TEST_CASE("ruida: 14-bit numbers and their limits") {
    std::string s;
    put_s14(s, -1);
    CHECK(s == bytes({0x7F, 0x7F}));
    CHECK(get_s14(u8(s)) == -1);

    s.clear();
    put_s14(s, kS14Max);
    CHECK(s == bytes({0x3F, 0x7F}));
    CHECK(get_s14(u8(s)) == 8191);

    s.clear();
    put_s14(s, kS14Min);
    CHECK(s == bytes({0x40, 0x00}));
    CHECK(get_s14(u8(s)) == -8192);
}

TEST_CASE("ruida: relative coordinate refuses +8192, which MeerK40t lets wrap") {
    // MeerK40t: abs(8192) > 8192 is false, so it emits a relative move of
    // +8192 um, which encode14 turns into bytes that decode as -8192 -- a
    // 16 mm error. Here the field's true range is enforced.
    std::string s;
    CHECK(put_rel_coord(s, 8191));
    CHECK(put_rel_coord(s, -8192));
    const std::size_t before = s.size();
    CHECK(!put_rel_coord(s, 8192));
    CHECK(!put_rel_coord(s, -8193));
    CHECK(s.size() == before);
    CHECK(fits_rel(8191) && !fits_rel(8192));
}

TEST_CASE("ruida: absolute coordinate is 32-bit two's complement, as LightBurn writes it") {
    // ex/BC_power_test.rd, Document Min Point X: 0F 7F 7F 7C 0C is -500 um.
    // The top three bits of the 35-bit field are zero; MeerK40t's encode32
    // would write 7F 7F 7F 7C 0C for the same value.
    std::string s;
    CHECK(put_abs_coord(s, -500));
    CHECK(s == bytes({0x0F, 0x7F, 0x7F, 0x7C, 0x0C}));
    CHECK(get_abs_coord(u8(s)) == -500);

    s.clear();
    CHECK(put_abs_coord(s, 231020));  // Process Bottom Right X in the same file
    CHECK(s == bytes({0x00, 0x00, 0x0E, 0x0C, 0x6C}));
    CHECK(get_abs_coord(u8(s)) == 231020);

    // MeerK40t's 35-bit form decodes to the same value, so either reads back.
    CHECK(get_abs_coord(u8(bytes({0x7F, 0x7F, 0x7F, 0x7C, 0x0C}))) == -500);

    s.clear();
    CHECK(!put_abs_coord(s, 0x80000000ll));
    CHECK(s.empty());
}

TEST_CASE("ruida: power rounds to nearest, as LightBurn does (MeerK40t truncates)") {
    // LightBurn (ex/BC_power_test.rd): 8% -> 0A 1F (1311), 12% -> 0F 2E (1966).
    // MeerK40t would write 1310 and 1965.
    std::string s;
    CHECK(put_power(s, 8.0));
    CHECK(s == bytes({0x0A, 0x1F}));
    s.clear();
    CHECK(put_power(s, 12.0));
    CHECK(s == bytes({0x0F, 0x2E}));
    s.clear();
    CHECK(put_power(s, 100.0));
    CHECK(s == bytes({0x7F, 0x7F}));
    s.clear();
    CHECK(put_power(s, 50.0));
    CHECK(s == bytes({0x40, 0x00}));
    s.clear();
    CHECK(put_power(s, 18.0));
    CHECK(s == bytes({0x17, 0x05}));
    s.clear();
    CHECK(put_power(s, 0.0));
    CHECK(s == bytes({0x00, 0x00}));

    s.clear();
    CHECK(!put_power(s, 100.1));
    CHECK(!put_power(s, -0.1));
    CHECK(s.empty());

    CHECK_NEAR(power_from_raw(16383), 100.0, 1e-12);
    CHECK_NEAR(power_from_raw(2949), 18.0, 0.01);
}

TEST_CASE("ruida: speed is um/s in 35 bits") {
    std::string s;
    CHECK(put_speed(s, 150.0));
    CHECK(s == bytes({0x00, 0x00, 0x09, 0x13, 0x70}));
    CHECK(get_u35(u8(s)) == 150000u);
    s.clear();
    CHECK(!put_speed(s, -1.0));
    CHECK(s.empty());
}

TEST_CASE("ruida: split_commands at every byte >= 0x80") {
    const std::string plain = bytes({0x88, 0x00, 0x01, 0xA9, 0x02, 0xD7});
    const auto cmds = split_commands(plain);
    REQUIRE(cmds.size() == 3);
    CHECK(cmds[0] == bytes({0x88, 0x00, 0x01}));
    CHECK(cmds[1] == bytes({0xA9, 0x02}));
    CHECK(cmds[2] == bytes({0xD7}));
    CHECK(split_commands("").empty());
}

TEST_CASE("ruida: UDP packet for the card-ID query") {
    // DA 00 05 7E: read memory 05 7E (card ID). Bytes from MeerK40t's _package.
    const Swizzle sw(0x88);
    const std::string pkt = package_udp(sw, bytes({0xDA, 0x00, 0x05, 0x7E}));
    CHECK(pkt == bytes({0x02, 0x61, 0xD4, 0x89, 0x0D, 0xF7}));

    std::string plain;
    CHECK(unpackage_udp(sw, pkt, plain));
    CHECK(plain == bytes({0xDA, 0x00, 0x05, 0x7E}));

    std::string corrupt = pkt;
    corrupt[3] = static_cast<char>(corrupt[3] ^ 0x01);
    std::string untouched = "x";
    CHECK(!unpackage_udp(sw, corrupt, untouched));
    CHECK(untouched == "x");
    CHECK(!unpackage_udp(sw, "\x01", untouched));
}

TEST_CASE("ruida: ACK and ENQ as swizzled single bytes") {
    const Swizzle sw(0x88);
    CHECK(sw.encode(bytes({0xCC})) == bytes({0xC6}));  // ACK
    CHECK(sw.encode(bytes({0xCE})) == bytes({0xC8}));  // ENQ
}
