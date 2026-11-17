// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

#include "ncad/laser/status_words.hpp"

namespace ncad::laser {

const char* state_seen(std::uint64_t v) {
    switch (v) {
    case 0x8000000: return "idle, just after power-on";
    case 0x8010600: return "idle";
    case 0x8010401: return "job starting";
    case 0x8010405: return "job running";
    case 0x8010403: return "job held (lid open)";
    case 0xD030600: return "moving (goto/jog)";
    default: return nullptr;
    }
}

bool state_is_idle(std::uint64_t v) { return v == 0x8000000 || v == 0x8010600; }

std::string reading(std::uint64_t v) { return v == 0xF2390000u ? std::string("n/a") : std::to_string(v); }

}  // namespace ncad::laser
