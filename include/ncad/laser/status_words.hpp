// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// What the status words of a Ruida controller are known to mean, and how a
// reading is printed. Shared by `ncad-laser` and the LASER command so the two
// never disagree about a state.
#pragma once

#include <cstdint>
#include <string>

namespace ncad::laser {

// State words seen on the Black Cat (RDC6442S, RDLC-V8.01.50), 2026-11-17.
// Observations, not a decoding: null for a word not listed.
const char* state_seen(std::uint64_t state);

// True for the words seen while the controller waits for work (idle).
bool state_is_idle(std::uint64_t state);

// Every read the controller has no value for answers 0xF2390000 (unused
// settings, file progress while idle); printed as "n/a", not as a number.
std::string reading(std::uint64_t v);

}  // namespace ncad::laser
