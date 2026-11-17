// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

#include "ncad/laser/device_config.hpp"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>

namespace ncad::laser {

namespace {

std::string trim(const std::string& s) {
    const char* ws = " \t\r\n";
    const std::size_t a = s.find_first_not_of(ws);
    if (a == std::string::npos) return {};
    return s.substr(a, s.find_last_not_of(ws) - a + 1);
}

bool parse_int(const std::string& v, long lo, long hi, long& out) {
    if (v.empty()) return false;
    char* end = nullptr;
    out = std::strtol(v.c_str(), &end, 0);  // base 0: 0x88 and 136 both work
    return *end == '\0' && out >= lo && out <= hi;
}

bool parse_double(const std::string& v, double& out) {
    if (v.empty()) return false;
    char* end = nullptr;
    out = std::strtod(v.c_str(), &end);
    return *end == '\0' && std::isfinite(out);
}

bool parse_bool(const std::string& v, bool& out) {
    if (v == "true" || v == "yes" || v == "on" || v == "1") { out = true; return true; }
    if (v == "false" || v == "no" || v == "off" || v == "0") { out = false; return true; }
    return false;
}

// Applies one key. Returns an empty string on success, else what is wrong.
std::string apply(DeviceConfig& d, const std::string& key, const std::string& v) {
    long n = 0;
    if (key == "driver") {
        if (v != "ruida") return "unknown driver '" + v + "' (only 'ruida' is supported)";
    } else if (key == "host") {
        if (v.empty()) return "host is empty";
        d.host = v;
    } else if (key == "serial") {
        // The controller's USB port, e.g. /dev/cu.usbserial-A904TTIM. A device
        // gives host or serial; if both, the command line decides (see ncad-laser).
        if (v.empty()) return "serial is empty";
        d.serial = v;
    } else if (key == "baud") {
        if (!parse_int(v, 300, 4000000, n)) return "bad baud '" + v + "' (300..4000000)";
        d.baud = static_cast<int>(n);
    } else if (key == "hw_flow") {
        bool b = false;
        if (!parse_bool(v, b)) return "bad hw_flow '" + v + "' (true or false)";
        d.hw_flow = b;
    } else if (key == "magic") {
        if (v == "auto") return "magic=auto is not implemented yet";
        if (!parse_int(v, 0, 255, n)) return "bad magic '" + v + "' (0..255, e.g. 0x88)";
        d.magic = static_cast<std::uint8_t>(n);
    } else if (key == "port") {
        if (!parse_int(v, 1, 65535, n)) return "bad port '" + v + "' (1..65535)";
        d.port = static_cast<int>(n);
    } else if (key == "local_port") {
        if (!parse_int(v, 0, 65535, n)) return "bad local_port '" + v + "' (0..65535)";
        d.local_port = static_cast<int>(n);
    } else if (key == "keypad_port") {
        if (!parse_int(v, 1, 65535, n)) return "bad keypad_port '" + v + "' (1..65535)";
        d.keypad_port = static_cast<int>(n);
    } else if (key == "max_packet") {
        if (!parse_int(v, 16, 65000, n)) return "bad max_packet '" + v + "' (16..65000)";
        d.max_packet = static_cast<std::size_t>(n);
    } else if (key == "ack_timeout_ms") {
        if (!parse_int(v, 10, 60000, n)) return "bad ack_timeout_ms '" + v + "' (10..60000)";
        d.ack_timeout_ms = static_cast<int>(n);
    } else if (key == "max_power") {
        double p = 0.0;
        if (!parse_double(v, p) || p < 0.0 || p > 100.0) return "bad max_power '" + v + "' (0..100)";
        d.max_power = p;
    } else if (key == "bed") {
        // "W H": two numbers, so a stray "600" or "600x400" is caught, not guessed at.
        const std::size_t sp = v.find_first_of(" \t");
        double w = 0.0, h = 0.0;
        if (sp == std::string::npos || !parse_double(v.substr(0, sp), w) ||
            !parse_double(trim(v.substr(sp)), h) || !(w > 0.0) || !(h > 0.0)) {
            return "bad bed '" + v + "' (expected 'W H' in mm, both positive)";
        }
        d.bed_w_mm = w;
        d.bed_h_mm = h;
    } else if (key == "pointer_offset") {
        // "X Y" in mm, any sign: the red dot's offset from the beam, in the
        // machine frame as LightBurn's device settings write it.
        const std::size_t sp = v.find_first_of(" \t");
        double x = 0.0, y = 0.0;
        if (sp == std::string::npos || !parse_double(v.substr(0, sp), x) || !parse_double(trim(v.substr(sp)), y)) {
            return "bad pointer_offset '" + v + "' (expected 'X Y' in mm)";
        }
        d.pointer_offset_x = x;
        d.pointer_offset_y = y;
    } else if (key == "origin_corner") {
        if (v != "bl" && v != "br" && v != "tl" && v != "tr") {
            return "bad origin_corner '" + v + "' (bl, br, tl or tr)";
        }
        d.origin_corner = v;
    } else if (key == "flip_x" || key == "flip_y") {
        bool b = false;
        if (!parse_bool(v, b)) return "bad " + key + " '" + v + "' (true or false)";
        (key == "flip_x" ? d.flip_x : d.flip_y) = b;
    } else {
        return "unknown key '" + key + "'";
    }
    return {};
}

bool parse(const std::string& text, const std::string& source, std::vector<DeviceConfig>& out,
           std::string& error) {
    std::vector<DeviceConfig> devices;
    std::vector<std::string> seen;  // keys given in the current section
    int line_no = 0;
    std::size_t pos = 0;
    auto fail = [&](const std::string& what) {
        error = source + ":" + std::to_string(line_no) + ": " + what;
        return false;
    };
    while (pos <= text.size()) {
        std::size_t nl = text.find('\n', pos);
        if (nl == std::string::npos) nl = text.size();
        std::string line = text.substr(pos, nl - pos);
        pos = nl + 1;
        ++line_no;
        const std::size_t c = line.find_first_of("#;");
        if (c != std::string::npos) line.erase(c);
        line = trim(line);
        if (line.empty()) continue;

        if (line.front() == '[') {
            if (line.back() != ']') return fail("unterminated section header");
            const std::string name = trim(line.substr(1, line.size() - 2));
            if (name.empty()) return fail("empty device name");
            for (const DeviceConfig& d : devices) {
                if (d.name == name) return fail("duplicate device '" + name + "'");
            }
            devices.emplace_back();
            devices.back().name = name;
            seen.clear();
            continue;
        }
        const std::size_t eq = line.find('=');
        if (eq == std::string::npos) return fail("expected 'key = value' or a [section]");
        if (devices.empty()) return fail("setting before any [device] section");
        const std::string key = trim(line.substr(0, eq));
        const std::string value = trim(line.substr(eq + 1));
        for (const std::string& k : seen) {
            if (k == key) return fail("duplicate key '" + key + "'");
        }
        seen.push_back(key);
        const std::string problem = apply(devices.back(), key, value);
        if (!problem.empty()) return fail(problem);
    }
    out = std::move(devices);
    return true;
}

}  // namespace

bool read_device_configs(const std::string& text, std::vector<DeviceConfig>& out, std::string& error) {
    return parse(text, "<config>", out, error);
}

bool read_device_config_file(const std::string& path, std::vector<DeviceConfig>& out, std::string& error) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        error = "cannot read '" + path + "'";
        return false;
    }
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    return parse(text, path, out, error);
}

std::string default_device_config_path() {
    const char* xdg = std::getenv("XDG_CONFIG_HOME");
    if (xdg && *xdg) return std::string(xdg) + "/ncad/lasers.conf";
    const char* home = std::getenv("HOME");
    if (home && *home) return std::string(home) + "/.config/ncad/lasers.conf";
    return {};
}

bool find_device(const std::vector<DeviceConfig>& devices, const std::string& name, DeviceConfig& out) {
    for (const DeviceConfig& d : devices) {
        if (d.name == name) {
            out = d;
            return true;
        }
    }
    return false;
}

bool select_device(const std::vector<DeviceConfig>& devices, const std::string& name, DeviceConfig& out,
                   std::string& error) {
    if (!name.empty()) {
        if (find_device(devices, name, out)) return true;
        error = "no device '" + name + "' in the configuration";
        if (!devices.empty()) {
            error += " (have:";
            for (const DeviceConfig& d : devices) error += " " + d.name;
            error += ")";
        }
        return false;
    }
    if (devices.size() == 1) {
        out = devices.front();
        return true;
    }
    error = devices.empty() ? "no device given: use --host IP or --device NAME"
                            : "several devices configured: choose one with --device NAME";
    return false;
}

}  // namespace ncad::laser
