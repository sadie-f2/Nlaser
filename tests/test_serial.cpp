// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// SerialPort over a POSIX pseudo-terminal, with the fake controller on the
// master end: the real termios code and the real link, end to end, no laser.

#include "test.hpp"

#include "fake_controller.hpp"

#include "ncad/laser/job.hpp"
#include "ncad/laser/plan.hpp"
#include "ncad/laser/ruida_decode.hpp"
#include "ncad/laser/ruida_link.hpp"
#include "ncad/laser/ruida_transport.hpp"
#include "ncad/laser/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>

using namespace ncad::laser;
using namespace ncad::laser::ruida;

namespace {

// A pseudo-terminal pair; the fake controller serves the master end on a
// thread, as the controller's USB chip serves the cable.
class PtyController {
public:
    nlaser_test::FakeSerialController fake;  // read it only after wait_for_stop()

    bool start() {
        master_ = ::posix_openpt(O_RDWR | O_NOCTTY);
        if (master_ < 0 || ::grantpt(master_) != 0 || ::unlockpt(master_) != 0) return false;
        const char* name = ::ptsname(master_);
        if (!name) return false;
        slave_path_ = name;
        // Raw on the master too, so replies are not mangled on the way out.
        termios t{};
        if (::tcgetattr(master_, &t) == 0) {
            ::cfmakeraw(&t);
            ::tcsetattr(master_, TCSANOW, &t);
        }
        thread_ = std::thread([this] { run(); });
        return true;
    }

    const std::string& path() const { return slave_path_; }

    // The fake is owned by the serving thread until this returns true.
    bool wait_for_stop() {
        for (int i = 0; i < 300; ++i) {
            if (stops_seen_) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return false;
    }

    ~PtyController() {
        stop_ = true;
        if (thread_.joinable()) thread_.join();
        if (master_ >= 0) ::close(master_);
    }

private:
    void run() {
        while (!stop_) {
            pollfd p{master_, POLLIN, 0};
            if (::poll(&p, 1, 20) <= 0 || !(p.revents & POLLIN)) continue;
            char buf[1024];
            const ssize_t n = ::read(master_, buf, sizeof buf);
            if (n <= 0) continue;
            std::string error;
            fake.write(std::string_view(buf, static_cast<std::size_t>(n)), error);
            const std::string out = fake.take_output();
            std::size_t done = 0;
            while (done < out.size()) {
                const ssize_t w = ::write(master_, out.data() + done, out.size() - done);
                if (w <= 0) break;
                done += static_cast<std::size_t>(w);
            }
            if (fake.stops > 0) stops_seen_ = true;  // published after the fake's last write
        }
    }

    int master_{-1};
    std::string slave_path_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> stops_seen_{false};
    std::thread thread_;
};

DeviceConfig quick_cfg() {
    DeviceConfig cfg;
    cfg.ack_timeout_ms = 1000;
    return cfg;
}

}  // namespace

TEST_CASE("serial port: counters and status over a pseudo-terminal") {
    PtyController ctl;
    ctl.fake.set_number(kMemCardId, 0x65106510);
    ctl.fake.set_number(kMemJobCount, 7);
    ctl.fake.set_number(kMemMachineStatus, 22);
    ctl.fake.memory[kMemFirmware] = std::string("RDC8.01.50\0", 11);
    REQUIRE(ctl.start());

    SerialPort port;
    std::string error;
    REQUIRE(port.open(ctl.path(), 115200, false, error));
    RuidaTransport t(port, quick_cfg());
    REQUIRE(t.open(error));
    Counters k;
    REQUIRE(t.counters(k, error));
    CHECK(k.card_id == 0x65106510u);
    CHECK(k.jobs == 7);
    CHECK(k.firmware == "RDC8.01.50");
    MachineStatus s;
    REQUIRE(t.status(s, error));
    CHECK(s.has_state && s.state == 22);
}

TEST_CASE("serial port: a whole job uploads through termios, byte for byte") {
    PtyController ctl;
    REQUIRE(ctl.start());

    LaserJob raw;
    LaserLayer l;
    l.pen = 5;
    l.min_power = l.max_power = 10;
    raw.layers.push_back(l);
    LaserPath p;
    p.pen = 5;
    p.closed = true;
    for (int i = 0; i < 2000; ++i) p.pts.push_back({50.0 + 20.0 * std::cos(i * 0.00314), 50.0 + 20.0 * std::sin(i * 0.00314)});
    raw.paths.push_back(p);
    LaserJob planned;
    PlanReport r;
    std::string error;
    REQUIRE(plan_job(raw, planned, PlanOptions{}, r, error));

    SerialPort port;
    REQUIRE(port.open(ctl.path(), 115200, false, error));
    DeviceConfig cfg = quick_cfg();
    cfg.max_packet = 200;
    RuidaTransport t(port, cfg);
    REQUIRE(t.open(error));
    REQUIRE(t.send(planned, Progress{}, error));
    REQUIRE(t.stop(error));  // nothing acknowledges the job, so the stop marks its end

    std::string expected;
    REQUIRE(prepare_send(planned, cfg, expected, error));
    REQUIRE(ctl.wait_for_stop());
    CHECK(ctl.fake.stops == 1);
    CHECK(ctl.fake.job == expected);
}

TEST_CASE("serial port: silence is a clean failure") {
    PtyController ctl;
    ctl.fake.silent = true;
    REQUIRE(ctl.start());
    SerialPort port;
    std::string error;
    REQUIRE(port.open(ctl.path(), 115200, false, error));
    DeviceConfig cfg;
    cfg.ack_timeout_ms = 100;
    RuidaTransport t(port, cfg);
    CHECK(!t.open(error));
    CHECK(error.find("no answer") != std::string::npos);
}

TEST_CASE("serial port: hardware flow control is accepted on a pseudo-terminal, or reported") {
    PtyController ctl;
    REQUIRE(ctl.start());
    SerialPort port;
    std::string error;
    if (port.open(ctl.path(), 115200, true, error)) {
        RuidaTransport t(port, quick_cfg());
        CHECK(t.open(error));
    } else {
        CHECK(!error.empty());  // a refusal must say why; real ttys take CRTSCTS
    }
}

TEST_CASE("serial port: bad paths, non-ttys and bad rates are refused with words") {
    SerialPort port;
    std::string error;
    CHECK(!port.open("/dev/cu.does-not-exist", 115200, false, error));
    CHECK(error.find("cannot open") != std::string::npos);

    error.clear();
    CHECK(!port.open("/dev/null", 115200, false, error));  // opens, but is not a tty
    CHECK(!error.empty());

    error.clear();
    CHECK(!port.open("/dev/null", 12345, false, error));
    CHECK(error.find("baud") != std::string::npos);

    error.clear();
    CHECK(!port.write("x", error));  // never opened
    CHECK(error == "port not open");
}
