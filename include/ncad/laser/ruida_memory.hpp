// SPDX-License-Identifier: BSD-3-Clause AND MIT
// Copyright (c) 2026, Sadie Forbes
// Copyright (c) 2021 meerk40t
//
// The names MeerK40t's emulator gives to controller memory addresses
// (meerk40t/ruida/emulator.py, RuidaEmulator.mem_lookup, commit d511944), as a
// data table, and a read-only dump of them from a controller. See
// third_party/meerk40t/NOTICE.
//
// What this is, and is not: MeerK40t's names come from reverse engineering the
// vendor's software and a reference controller; the VALUES in its table are
// stubs for its emulator and are NOT ported. An address a real controller
// answers is not thereby confirmed to mean what the name says.
#pragma once

#include "ncad/laser/ruida_transport.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace ncad::laser::ruida {

// The name MeerK40t gives `mem` (a 14-bit address), "" if it names none. An
// address in a range is numbered: "PC Lock 3", "Document Number +5".
std::string memory_name(std::uint16_t mem);

// Every address with a name, ascending (ranges expanded).
std::vector<std::uint16_t> named_addresses();

// Names that mention red, light, pointer or offset -- where the red-dot
// pointer offset would be. "red" must be a word of its own ("Preferred" is
// not red).
bool memory_name_interesting(const std::string& name);

// "0x0221" (14-bit) or "04:21" (the two 7-bit wire bytes, hex); false if
// malformed or above 0x3FFF.
bool parse_memory_address(const std::string& text, std::uint16_t& out);

// "0x0221 [04 21]": the address as 14-bit hex and as the two wire bytes.
std::string format_memory_address(std::uint16_t mem);

struct SettingRow {
    std::uint16_t addr{0};
    std::string name;       // "" = not in MeerK40t's table
    bool interesting{false};
    bool replied{false};    // false: the controller did not answer this read
    std::string raw;        // the value bytes as received
    bool has_number{false};
    std::uint64_t number{0};
    std::string text;       // the value as text, when it looks like text
    std::string error;      // why there was no reply
};

// Reads `addresses` in order, one query each. A missing reply is recorded and
// the dump goes on. `stop` (may be empty) is polled between reads; `progress`
// (may be empty) is told how many are done. Read-only by construction: only
// DA 00 memory reads are issued.
bool read_settings(RuidaTransport& t, const std::vector<std::uint16_t>& addresses, std::vector<SettingRow>& rows,
                   const std::function<bool()>& stop, const std::function<void(std::size_t, std::size_t)>& progress,
                   std::string& error);

// `raw` as text if it looks like a string: printable ASCII up to a NUL, at
// least 3 characters, and more than the 5 bytes of a plain number.
std::string value_as_text(const std::string& raw);

}  // namespace ncad::laser::ruida
