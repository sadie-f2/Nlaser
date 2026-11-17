// SPDX-License-Identifier: BSD-3-Clause AND MIT
// Copyright (c) 2026, Sadie Forbes
// Copyright (c) 2021 meerk40t
//
// The line settings follow meerk40t/ruida/usb_transport.py. See
// third_party/meerk40t/NOTICE.

#include "ncad/laser/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace ncad::laser {

namespace {

bool baud_constant(int baud, speed_t& out) {
    switch (baud) {
        case 9600: out = B9600; return true;
        case 19200: out = B19200; return true;
        case 38400: out = B38400; return true;
        case 57600: out = B57600; return true;
        case 115200: out = B115200; return true;
        case 230400: out = B230400; return true;
        default: return false;
    }
}

std::string sys_error(const char* what) { return std::string(what) + ": " + std::strerror(errno); }

}  // namespace

SerialPort::~SerialPort() { close(); }

void SerialPort::close() {
    if (fd_ < 0) return;
    // Let queued job bytes leave before the descriptor goes; a close that
    // discards them would silently truncate an upload.
    ::tcdrain(fd_);
    ::close(fd_);
    fd_ = -1;
}

// The device went away (USB unplugged, controller off): drop the descriptor
// without tcdrain, which could wait on a device that no longer exists. The
// next write reopens the same path, so a replugged cable works again without
// restarting the program or leaving LASER.
void SerialPort::lost() {
    if (fd_ < 0) return;
    ::close(fd_);
    fd_ = -1;
}

bool SerialPort::open(const std::string& path, int baud, bool hw_flow, std::string& error) {
    close();
    speed_t speed = B115200;
    if (!baud_constant(baud, speed)) {
        error = "unsupported baud rate " + std::to_string(baud) + " (9600, 19200, 38400, 57600, 115200, 230400)";
        return false;
    }
    // Non-blocking, and no controlling terminal: opening a tty must not make
    // it ours, and a modem-control line must not stall the open.
    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        error = "cannot open " + path + ": " + std::strerror(errno);
        return false;
    }
    if (::ioctl(fd_, TIOCEXCL) != 0) {
        error = sys_error("cannot get exclusive use of the serial port");
        close();
        return false;
    }

    termios t{};
    if (::tcgetattr(fd_, &t) != 0) {
        error = path + " is not a serial device: " + std::strerror(errno);
        close();
        return false;
    }
    ::cfmakeraw(&t);  // no echo, no line editing, no CR/LF or XON/XOFF mangling of job bytes
    t.c_cflag &= ~static_cast<tcflag_t>(CSIZE | PARENB | CSTOPB | CRTSCTS);
    t.c_cflag |= CS8 | CLOCAL | CREAD;
    if (hw_flow) t.c_cflag |= CRTSCTS;
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 0;
    if (::cfsetispeed(&t, speed) != 0 || ::cfsetospeed(&t, speed) != 0 || ::tcsetattr(fd_, TCSANOW, &t) != 0) {
        error = sys_error("cannot configure the serial port");
        close();
        return false;
    }
    // Assert DTR and RTS, as MeerK40t does. Best effort: not every device (a
    // pseudo-terminal, say) has modem lines.
    int lines = TIOCM_DTR | TIOCM_RTS;
    (void)::ioctl(fd_, TIOCMBIS, &lines);
    ::tcflush(fd_, TCIOFLUSH);
    // Remembered only once it has worked, so a write after a failed first
    // open still says "port not open" rather than retrying a bad path.
    path_ = path;
    baud_ = baud;
    hw_flow_ = hw_flow;
    return true;
}

bool SerialPort::write(std::string_view bytes, std::string& error) {
    if (fd_ < 0) {
        if (path_.empty()) {
            error = "port not open";
            return false;
        }
        std::string why;
        if (!open(path_, baud_, hw_flow_, why)) {
            error = "the serial device is gone (cable unplugged?): " + why;
            return false;
        }
    }
    std::size_t done = 0;
    while (done < bytes.size()) {
        const ssize_t n = ::write(fd_, bytes.data() + done, bytes.size() - done);
        if (n > 0) {
            done += static_cast<std::size_t>(n);
            continue;
        }
        if (n < 0 && errno == EINTR) continue;
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            // The tty buffer is full: the controller is holding RTS/CTS low.
            // Wait for room, but not forever -- a wedged controller is an error.
            pollfd p{fd_, POLLOUT, 0};
            const int r = ::poll(&p, 1, 10000);
            if (r > 0) continue;
            if (r < 0 && errno == EINTR) continue;
            error = r == 0 ? "the controller stopped accepting data for 10 s" : sys_error("poll");
            return false;
        }
        error = sys_error("write");
        lost();
        return false;
    }
    return true;
}

int SerialPort::read(std::string& out, std::size_t max, int timeout_ms, std::string& error) {
    if (fd_ < 0) {
        error = "port not open";
        return -1;
    }
    for (;;) {
        pollfd p{fd_, POLLIN, 0};
        const int r = ::poll(&p, 1, timeout_ms);
        if (r == 0) return 0;
        if (r < 0) {
            if (errno == EINTR) continue;
            error = sys_error("poll");
            return -1;
        }
        char buf[512];
        const std::size_t want = max < sizeof buf ? max : sizeof buf;
        const ssize_t n = ::read(fd_, buf, want);
        if (n > 0) {
            out.append(buf, static_cast<std::size_t>(n));
            return 1;
        }
        if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) continue;
        // n == 0 after POLLIN, or POLLHUP/EIO: the device went away (USB
        // unplugged or the controller powered off).
        error = n == 0 ? "the serial device closed" : sys_error("read");
        lost();
        return -1;
    }
}

}  // namespace ncad::laser
