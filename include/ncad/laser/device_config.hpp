// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// lasers.conf: named devices, so a front end says `--device bench` rather
// than repeating an address and a dozen tuning flags.
//
// INI, one [name] section per device. `#` and `;` start a comment, on a line
// of their own or after a value. Keys: driver (only `ruida`), host, magic,
// port, local_port, keypad_port, max_packet, max_power, power_limit (hard limit, file-only), ack_timeout_ms, bed (`W H` in mm),
// origin_corner (bl br tl tr), flip_x, flip_y,
// pointer_offset (`X Y` mm: the red-dot offset, see ruida_encode.hpp). Unknown keys and malformed
// values are errors, because a typo that silently fell back to a default
// (say max_power) is how a laser runs hotter than its owner meant.
//
// Pure text in, structs out: no file or socket access except the file
// wrapper, so the headless core stays testable.
#pragma once

#include "ncad/laser/transport.hpp"

#include <string>
#include <vector>

namespace ncad::laser {

// `out` is replaced. Errors read "<config>:LINE: what".
bool read_device_configs(const std::string& text, std::vector<DeviceConfig>& out, std::string& error);

// As above, reading `path`; errors name the path instead of "<config>".
bool read_device_config_file(const std::string& path, std::vector<DeviceConfig>& out, std::string& error);

// $XDG_CONFIG_HOME/ncad/lasers.conf, else ~/.config/ncad/lasers.conf. Empty
// when neither variable is set.
std::string default_device_config_path();

// Sets `pointer_offset = X Y` in device `name`'s section of lasers.conf text,
// editing only that key: an existing `pointer_offset` line is replaced (its
// indentation and trailing comment kept), else a line is added after the last
// setting of the section. Every other byte of `text` is preserved. This is the
// ONLY key a front end may write back; power_limit stays file-only (see
// docs/design-notes.md). False with `error` when the section does not exist or
// the numbers are not finite.
bool set_pointer_offset_text(const std::string& text, const std::string& name, double x, double y,
                             std::string& out, std::string& error);

// As above, on the file `path`: written to a temporary file beside it and
// renamed over it, so a failure leaves the original untouched.
bool set_pointer_offset_file(const std::string& path, const std::string& name, double x, double y,
                             std::string& error);

// False when no device has that name.
bool find_device(const std::vector<DeviceConfig>& devices, const std::string& name, DeviceConfig& out);

// What a front end starts from before applying flags. A non-empty `name`
// must exist. An empty one picks the device only when there is exactly one:
// with several, guessing which laser to fire is not acceptable.
bool select_device(const std::vector<DeviceConfig>& devices, const std::string& name, DeviceConfig& out,
                   std::string& error);

}  // namespace ncad::laser
