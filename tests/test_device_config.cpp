// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// lasers.conf parsing: values, comments, errors with line numbers, lookup.

#include "test.hpp"

#include "ncad/laser/device_config.hpp"

#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

using namespace ncad::laser;

namespace {

bool fails_with(const std::string& text, const std::string& expect) {
    std::vector<DeviceConfig> out;
    std::string error;
    if (read_device_configs(text, out, error)) return false;
    return error.find(expect) != std::string::npos;
}

}  // namespace

TEST_CASE("device config: every key") {
    const std::string text =
        "[bench]\n"
        "driver = ruida\n"
        "host = 192.168.1.100\n"
        "magic = 0x11\n"
        "port = 50201\n"
        "local_port = 40201\n"
        "max_packet = 512\n"
        "max_power = 60.5\n"
        "ack_timeout_ms = 500\n"
        "bed = 600 400\n"
        "origin_corner = tl\n"
        "flip_x = yes\n"
        "flip_y = false\n"
        "pointer_offset = 5.2 -31.2   # the red dot, as LightBurn writes it\n";
    std::vector<DeviceConfig> d;
    std::string error;
    REQUIRE(read_device_configs(text, d, error));
    REQUIRE(d.size() == 1u);
    CHECK(d[0].name == "bench");
    CHECK(d[0].host == "192.168.1.100");
    CHECK(d[0].magic == 0x11);
    CHECK(d[0].port == 50201);
    CHECK(d[0].local_port == 40201);
    CHECK(d[0].max_packet == 512u);
    CHECK_NEAR(d[0].max_power, 60.5, 1e-9);
    CHECK(d[0].ack_timeout_ms == 500);
    CHECK_NEAR(d[0].bed_w_mm, 600.0, 1e-9);
    CHECK_NEAR(d[0].bed_h_mm, 400.0, 1e-9);
    CHECK(d[0].origin_corner == "tl");
    CHECK(d[0].flip_x);
    CHECK(!d[0].flip_y);
    CHECK_NEAR(d[0].pointer_offset_x, 5.2, 1e-9);
    CHECK_NEAR(d[0].pointer_offset_y, -31.2, 1e-9);
}

TEST_CASE("device config: defaults, comments, several devices") {
    const std::string text =
        "# the lab\n"
        "; another style\n"
        "\n"
        "[a]  # trailing comment\n"
        "host = 10.0.0.1 ; the bench\n"
        "\n"
        "[ b ]\r\n"
        "host=10.0.0.2\r\n"
        "magic = 136";  // no final newline
    std::vector<DeviceConfig> d;
    std::string error;
    REQUIRE(read_device_configs(text, d, error));
    REQUIRE(d.size() == 2u);
    CHECK(d[0].name == "a");
    CHECK(d[0].host == "10.0.0.1");
    CHECK(d[0].magic == 0x88);
    CHECK(d[0].port == 50200);
    CHECK(d[0].bed_w_mm == 0.0);
    CHECK(d[0].origin_corner.empty());
    CHECK(d[0].pointer_offset_x == 0.0 && d[0].pointer_offset_y == 0.0);
    CHECK(d[1].name == "b");
    CHECK(d[1].host == "10.0.0.2");
    CHECK(d[1].magic == 136);

    std::vector<DeviceConfig> none;
    REQUIRE(read_device_configs("# nothing\n", none, error));
    CHECK(none.empty());
}

TEST_CASE("device config: errors name the line") {
    CHECK(fails_with("[a]\nhost = x\nbogus = 1\n", "<config>:3: unknown key 'bogus'"));
    CHECK(fails_with("host = x\n", "<config>:1: setting before any"));
    CHECK(fails_with("[a]\nmagic = 300\n", "<config>:2: bad magic"));
    CHECK(fails_with("[a]\nmagic = auto\n", "<config>:2: magic=auto is not implemented yet"));
    CHECK(fails_with("[a]\ndriver = grbl\n", "<config>:2: unknown driver 'grbl'"));
    CHECK(fails_with("[a]\nport = 0\n", "bad port"));
    CHECK(fails_with("[a]\nport = 12abc\n", "bad port"));
    CHECK(fails_with("[a]\nmax_power = 120\n", "bad max_power"));
    CHECK(fails_with("[a]\nbed = 600\n", "bad bed"));
    CHECK(fails_with("[a]\nbed = 600x400\n", "bad bed"));
    CHECK(fails_with("[a]\nbed = 600 -4\n", "bad bed"));
    CHECK(fails_with("[a]\norigin_corner = middle\n", "bad origin_corner"));
    CHECK(fails_with("[a]\nflip_x = maybe\n", "bad flip_x"));
    CHECK(fails_with("[a]\npointer_offset = 5.2\n", "bad pointer_offset"));
    CHECK(fails_with("[a]\npointer_offset = 5.2,31.2\n", "bad pointer_offset"));
    CHECK(fails_with("[a]\npointer_offset = a b\n", "bad pointer_offset"));
    CHECK(fails_with("[a\nhost = x\n", "<config>:1: unterminated"));
    CHECK(fails_with("[]\n", "empty device name"));
    CHECK(fails_with("[a]\njunk\n", "<config>:2: expected"));
    CHECK(fails_with("[a]\n[a]\n", "<config>:2: duplicate device 'a'"));
    CHECK(fails_with("[a]\nhost = x\nhost = y\n", "<config>:3: duplicate key"));
}

TEST_CASE("device config: file wrapper names the path") {
    std::vector<DeviceConfig> d;
    std::string error;
    CHECK(!read_device_config_file("/nonexistent/lasers.conf", d, error));
    CHECK(error.find("/nonexistent/lasers.conf") != std::string::npos);
    CHECK(!read_device_config_file(NLASER_FIXTURES "/bad_lasers.conf", d, error));
    CHECK(error.find("bad_lasers.conf:2: unknown key 'colour'") != std::string::npos);
}

TEST_CASE("device config: find_device and select_device") {
    std::vector<DeviceConfig> d;
    std::string error;
    REQUIRE(read_device_configs("[a]\nhost = 1.1.1.1\n[b]\nhost = 2.2.2.2\n", d, error));
    DeviceConfig got;
    REQUIRE(find_device(d, "b", got));
    CHECK(got.host == "2.2.2.2");
    CHECK(!find_device(d, "c", got));

    CHECK(select_device(d, "a", got, error));
    CHECK(got.host == "1.1.1.1");
    CHECK(!select_device(d, "zzz", got, error));
    CHECK(error.find("zzz") != std::string::npos);
    CHECK(error.find("a b") != std::string::npos);
    // Two devices and no name: never guess which laser to fire.
    CHECK(!select_device(d, "", got, error));
    CHECK(error.find("--device") != std::string::npos);
}

TEST_CASE("device config: the single device is the default") {
    std::vector<DeviceConfig> d;
    std::string error;
    REQUIRE(read_device_configs("[only]\nhost = 10.1.1.1\nmagic = 0x11\nport = 50201\n", d, error));
    DeviceConfig cfg;
    REQUIRE(select_device(d, "", cfg, error));
    CHECK(cfg.name == "only");
    CHECK(cfg.host == "10.1.1.1");
    // The front end applies flags to this struct, so an explicit flag wins
    // and every key it leaves alone keeps the file's value.
    cfg.host = "10.9.9.9";
    cfg.magic = 0x88;
    CHECK(cfg.port == 50201);

    std::vector<DeviceConfig> none;
    CHECK(!select_device(none, "", cfg, error));
    CHECK(error.find("--host") != std::string::npos);
}

TEST_CASE("device config: default path shape") {
    // The environment is the caller's, so only the shape is checked.
    const std::string p = default_device_config_path();
    CHECK(p.empty() || p.find("ncad/lasers.conf") != std::string::npos);
}

TEST_CASE("device config: a USB serial device") {
    std::vector<ncad::laser::DeviceConfig> devs;
    std::string error;
    REQUIRE(ncad::laser::read_device_configs(
        "[blackcat]\nserial = /dev/cu.usbserial-A904TTIM\nbaud = 115200\nhw_flow = off\norigin_corner = tr\n", devs,
        error));
    REQUIRE(devs.size() == 1);
    CHECK(devs[0].serial == "/dev/cu.usbserial-A904TTIM");
    CHECK(devs[0].host.empty());
    CHECK(devs[0].baud == 115200);
    CHECK(!devs[0].hw_flow);
    CHECK(!ncad::laser::read_device_configs("[x]\nbaud = 12\n", devs, error));
    CHECK(error.find("baud") != std::string::npos);
}

TEST_CASE("device config: keypad_port") {
    std::vector<ncad::laser::DeviceConfig> devs;
    std::string error;
    REQUIRE(ncad::laser::read_device_configs("[a]\nhost = 1.2.3.4\n[b]\nhost = 1.2.3.5\nkeypad_port = 50300\n", devs,
                                              error));
    CHECK(devs[0].keypad_port == 50207);
    CHECK(devs[1].keypad_port == 50300);
    CHECK(!ncad::laser::read_device_configs("[a]\nkeypad_port = 0\n", devs, error));
}

TEST_CASE("device config: power_limit defaults to 80 and is read from the file only") {
    CHECK_NEAR(ncad::laser::DeviceConfig{}.power_limit, 80.0, 1e-9);
    std::vector<ncad::laser::DeviceConfig> devs;
    std::string error;
    REQUIRE(ncad::laser::read_device_configs("[a]\nhost = x\n[b]\nhost = y\npower_limit = 100\n[c]\npower_limit = 0\n",
                                              devs, error));
    CHECK_NEAR(devs[0].power_limit, 80.0, 1e-9);
    CHECK_NEAR(devs[1].power_limit, 100.0, 1e-9);
    CHECK_NEAR(devs[2].power_limit, 0.0, 1e-9);
    for (const char* bad : {"101", "-1", "abc", "", "50%"}) {
        CHECK(!ncad::laser::read_device_configs(std::string("[a]\npower_limit = ") + bad + "\n", devs, error));
        CHECK(error.find("bad power_limit") != std::string::npos);
    }
}

TEST_CASE("device config: set_pointer_offset_text edits one key and nothing else") {
    using ncad::laser::set_pointer_offset_text;
    std::string out, error;
    // Replaces in place, keeping indentation and the trailing comment.
    const std::string a =
        "# top comment\n"
        "[bench]\n"
        "host = 1.2.3.4   # the bench\n"
        "  pointer_offset =  1 2   # LightBurn\n"
        "power_limit = 100\n"
        "\n"
        "[a2]\n"
        "pointer_offset = 9 9\n";
    REQUIRE(set_pointer_offset_text(a, "bench", 5.2, -31.25, out, error));
    CHECK(out ==
          "# top comment\n"
          "[bench]\n"
          "host = 1.2.3.4   # the bench\n"
          "  pointer_offset = 5.2 -31.25   # LightBurn\n"
          "power_limit = 100\n"
          "\n"
          "[a2]\n"
          "pointer_offset = 9 9\n");

    // Adds after the section's last setting, before the blank line and the next section.
    const std::string b = "[bench]\nhost = x   ; c\n\n# about a2\n[a2]\nhost = y\n";
    REQUIRE(set_pointer_offset_text(b, "bench", 0, 0.5, out, error));
    CHECK(out == "[bench]\nhost = x   ; c\npointer_offset = 0 0.5\n\n# about a2\n[a2]\nhost = y\n");

    // The last section of a file with no final newline, and CRLF.
    REQUIRE(set_pointer_offset_text("[a]\nhost = x", "a", 1, 2, out, error));
    CHECK(out == "[a]\nhost = x\npointer_offset = 1 2");
    REQUIRE(set_pointer_offset_text("[a]\r\nhost = x\r\n", "a", 1, 2, out, error));
    CHECK(out == "[a]\r\nhost = x\r\npointer_offset = 1 2\r\n");

    // An empty section, and the result still parses back to the same numbers.
    REQUIRE(set_pointer_offset_text("[a]\n[b]\nhost = z\n", "a", 3.5, 4.5, out, error));
    CHECK(out == "[a]\npointer_offset = 3.5 4.5\n[b]\nhost = z\n");
    std::vector<ncad::laser::DeviceConfig> devs;
    REQUIRE(ncad::laser::read_device_configs(out, devs, error));
    CHECK_NEAR(devs[0].pointer_offset_x, 3.5, 1e-12);
    CHECK_NEAR(devs[0].pointer_offset_y, 4.5, 1e-12);

    CHECK(!set_pointer_offset_text(a, "nonesuch", 1, 1, out, error));
    CHECK(error.find("nonesuch") != std::string::npos);
    CHECK(!set_pointer_offset_text(a, "bench", 1e6, 1, out, error));
}

TEST_CASE("device config: set_pointer_offset_file writes through a temporary file") {
    namespace fs = std::filesystem;
    const fs::path dir = fs::temp_directory_path() / ("nlaser_cfg_" + std::to_string(::getpid()));
    fs::create_directories(dir);
    const std::string path = (dir / "lasers.conf").string();
    const std::string text = "[bench]\nhost = 1.2.3.4\npower_limit = 60\n";
    std::ofstream(path, std::ios::binary) << text;
    std::string error;
    REQUIRE(ncad::laser::set_pointer_offset_file(path, "bench", 7, 8, error));
    std::ifstream in(path, std::ios::binary);
    const std::string got((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    CHECK(got == text + "pointer_offset = 7 8\n");
    CHECK(!fs::exists(path + ".tmp"));
    CHECK(!ncad::laser::set_pointer_offset_file((dir / "absent.conf").string(), "bench", 1, 1, error));
    std::error_code ec;
    fs::remove_all(dir, ec);
}
