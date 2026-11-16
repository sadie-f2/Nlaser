// SPDX-License-Identifier: BSD-3-Clause AND MIT
// Copyright (c) 2026, Sadie Forbes
// Copyright (c) 2021 meerk40t
//
// The line settings follow meerk40t/ruida/usb_transport.py. See
// third_party/meerk40t/NOTICE.

// A StreamPort over a POSIX serial device (termios). Lives in ncad_laser_net,
// the only target that includes OS I/O headers; this header does not.
//
// What MeerK40t opens the Ruida's USB port with (usb_transport.py:10-13,
// 49-55): 8 data bits, no parity, 1 stop bit, RTS/CTS flow control, 115200
// baud, and it asserts RTS and DTR. It says the controller is an FTDI USB
// serial chip and that "baud doesn't seem to matter much" -- which is what a
// USB-to-serial bridge that fakes the rate would show, but it is a
// reverse-engineered observation, not a spec (usb_transport.py:3-5). So the
// rate is a parameter, defaulting to MeerK40t's, and `hw_flow` can be turned
// off if a cable does not carry CTS.
//
// The device is opened exclusively (TIOCEXCL) so a second program cannot
// interleave bytes into a job; that is the serial counterpart of UdpPort
// finding 40200 busy.
#pragma once

#include "ncad/laser/transport.hpp"

#include <string>

namespace ncad::laser {

class SerialPort final : public StreamPort {
public:
    SerialPort() = default;
    ~SerialPort() override;
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // `path` is /dev/cu.* (macOS; cu, not tty, which waits for carrier) or
    // /dev/ttyUSB*, /dev/ttyACM* (Linux). `baud` must be a standard rate
    // (9600 .. 230400). Input left over from before the open is discarded.
    bool open(const std::string& path, int baud, bool hw_flow, std::string& error);
    void close();

    bool write(std::string_view bytes, std::string& error) override;
    int read(std::string& out, std::size_t max, int timeout_ms, std::string& error) override;

private:
    int fd_{-1};
};

}  // namespace ncad::laser
