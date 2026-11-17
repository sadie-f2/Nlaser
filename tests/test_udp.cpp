// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// UdpPort over real sockets on loopback, with the fake controller behind a
// socket on a thread: counters, then a whole job upload, end to end.

#include "test.hpp"

#include "fake_controller.hpp"

#include "ncad/laser/job.hpp"
#include "ncad/laser/keypad.hpp"
#include "ncad/laser/plan.hpp"
#include "ncad/laser/ruida_decode.hpp"
#include "ncad/laser/ruida_link.hpp"
#include "ncad/laser/ruida_transport.hpp"
#include "ncad/laser/udp_port.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cmath>
#include <string>
#include <vector>
#include <thread>

using namespace ncad::laser;
using namespace ncad::laser::ruida;

namespace {

// The fake controller, listening on a loopback UDP port and answering the
// sender, as a controller answers 40200.
class LoopbackController {
public:
    nlaser_test::FakeController fake;

    bool start() {
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        a.sin_port = 0;
        if (fd_ < 0 || ::bind(fd_, reinterpret_cast<sockaddr*>(&a), sizeof a) != 0) return false;
        socklen_t len = sizeof a;
        ::getsockname(fd_, reinterpret_cast<sockaddr*>(&a), &len);
        port_ = ntohs(a.sin_port);
        thread_ = std::thread([this] { run(); });
        return true;
    }

    int port() const { return port_; }

    ~LoopbackController() {
        stop_ = true;
        if (thread_.joinable()) thread_.join();
        if (fd_ >= 0) ::close(fd_);
    }

private:
    void run() {
        while (!stop_) {
            pollfd p{fd_, POLLIN, 0};
            if (::poll(&p, 1, 20) <= 0) continue;
            static char buf[65536];  // a datagram can be this big; --chunk goes to 65000
            sockaddr_in from{};
            socklen_t flen = sizeof from;
            const ssize_t n = ::recvfrom(fd_, buf, sizeof buf, 0, reinterpret_cast<sockaddr*>(&from), &flen);
            if (n <= 0) continue;
            std::string error, out;
            fake.send(std::string_view(buf, static_cast<std::size_t>(n)), error);
            while (fake.recv(out, 0, error) == 1) {
                ::sendto(fd_, out.data(), out.size(), 0, reinterpret_cast<sockaddr*>(&from), flen);
            }
        }
    }

    int fd_{-1};
    int port_{0};
    std::atomic<bool> stop_{false};
    std::thread thread_;
};

}  // namespace

TEST_CASE("udp: counters over loopback") {
    LoopbackController ctl;
    ctl.fake.set_number(kMemCardId, 0x65106510);
    ctl.fake.set_number(kMemJobCount, 7);
    REQUIRE(ctl.start());

    UdpPort port;
    std::string error;
    REQUIRE(port.open("127.0.0.1", ctl.port(), 0, error));
    DeviceConfig cfg;
    cfg.ack_timeout_ms = 500;
    RuidaTransport t(port, cfg);
    REQUIRE(t.open(error));
    Counters k;
    REQUIRE(t.counters(k, error));
    CHECK(k.card_id == 0x65106510u);
    CHECK(k.jobs == 7);
}

TEST_CASE("udp: a whole job uploads over loopback, with a NAK on the way") {
    LoopbackController ctl;
    ctl.fake.nak_next = 1;
    REQUIRE(ctl.start());

    LaserJob raw;
    LaserLayer l;
    l.pen = 5;
    l.min_power = l.max_power = 10;
    raw.layers.push_back(l);
    LaserPath p;
    p.pen = 5;
    p.closed = true;
    for (int i = 0; i < 400; ++i) p.pts.push_back({50.0 + 20.0 * std::cos(i * 0.0157), 50.0 + 20.0 * std::sin(i * 0.0157)});
    raw.paths.push_back(p);
    LaserJob planned;
    PlanReport r;
    std::string error;
    REQUIRE(plan_job(raw, planned, PlanOptions{}, r, error));

    UdpPort port;
    REQUIRE(port.open("127.0.0.1", ctl.port(), 0, error));
    DeviceConfig cfg;
    cfg.ack_timeout_ms = 500;
    cfg.max_packet = 200;
    RuidaTransport t(port, cfg);
    REQUIRE(t.open(error));
    REQUIRE(t.send(planned, Progress{}, error));

    std::string expected;
    REQUIRE(prepare_send(planned, cfg, expected, error));
    CHECK(ctl.fake.job == expected);
    CHECK(ctl.fake.naks_sent == 1);
    CHECK(t.stats().naks == 1);
}

// Seen once as a crash of nlaser_fakectl while tools/laser_edge_cases.sh ran
// against it, and not reproduced since (the whole script, 20000 random
// datagrams and a decoder fuzz all ran clean under ASan/UBSan). What a
// controller sits in front of is the network, so pin the cases a run can
// produce: a datagram bigger than the old 2048-byte receive buffer, and
// malformed ones, none of which may bring it down or leave it deaf.
TEST_CASE("udp: oversize and malformed datagrams are survived, and the next read still answers") {
    LoopbackController ctl;
    ctl.fake.set_number(kMemCardId, 0x65106510);
    REQUIRE(ctl.start());
    UdpPort port;
    std::string error, in;
    REQUIRE(port.open("127.0.0.1", ctl.port(), 0, error));
    const Swizzle sw;

    // 8000 bytes of plain payload: whole, it passes its sum and is ACKed (0xCC).
    REQUIRE(port.send(package_udp(sw, std::string(8000, '\x01')), error));
    REQUIRE(port.recv(in, 1000, error) == 1);
    CHECK(sw.decode(in) == std::string(1, '\xCC'));

    // Empty, one byte, a bare checksum, and noise: each is answered or ignored.
    for (const std::string& d : {std::string(), std::string(1, '\x7F'), std::string(2, '\0'),
                                 std::string("\x12\x34\x56\x78\x9A", 5)}) {
        REQUIRE(port.send(d, error));
    }
    while (port.recv(in, 100, error) == 1) {
    }

    // Still alive: a memory read gets its DA 01 reply.
    const std::string read("\xDA\x00\x05\x7E", 4);
    REQUIRE(port.send(package_udp(sw, read), error));
    bool answered = false;
    for (int i = 0; i < 4 && port.recv(in, 500, error) == 1; ++i) {
        const std::string plain = sw.decode(in);
        if (plain.size() > 2 && static_cast<std::uint8_t>(plain[0]) == 0xDA && plain[1] == 0x01) answered = true;
        if (answered) break;
    }
    CHECK(answered);
}

TEST_CASE("udp: nothing listening is a clean failure") {
    // Bind a port and close it, so nothing answers there.
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    ::bind(fd, reinterpret_cast<sockaddr*>(&a), sizeof a);
    socklen_t len = sizeof a;
    ::getsockname(fd, reinterpret_cast<sockaddr*>(&a), &len);
    const int dead = ntohs(a.sin_port);
    ::close(fd);

    UdpPort port;
    std::string error;
    REQUIRE(port.open("127.0.0.1", dead, 0, error));
    DeviceConfig cfg;
    cfg.ack_timeout_ms = 100;
    RuidaTransport t(port, cfg);
    CHECK(!t.open(error));
    CHECK(!error.empty());
}

TEST_CASE("keypad: the key method sends exactly key-down then key-up, plain, to the keypad port") {
    // A fake keypad listener on loopback, standing in for the controller's 50207.
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    REQUIRE(fd >= 0);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    REQUIRE(::bind(fd, reinterpret_cast<sockaddr*>(&a), sizeof a) == 0);
    socklen_t len = sizeof a;
    ::getsockname(fd, reinterpret_cast<sockaddr*>(&a), &len);

    DeviceConfig cfg;
    cfg.host = "127.0.0.1";
    cfg.keypad_port = ntohs(a.sin_port);
    cfg.keypad_local_port = 0;
    cfg.power_limit = 100.0;  // the key method is refused under any lower limit
    std::string error;
    REQUIRE(pulse_key(cfg, 20, error));

    std::vector<std::string> got;
    for (;;) {
        pollfd p{fd, POLLIN, 0};
        if (::poll(&p, 1, 200) <= 0) break;
        char buf[64];
        const ssize_t n = ::recv(fd, buf, sizeof buf, 0);
        if (n <= 0) break;
        got.emplace_back(buf, static_cast<std::size_t>(n));
    }
    ::close(fd);
    REQUIRE(got.size() == 2);
    CHECK(got[0] == std::string("\xA5\x50\x05", 3));
    CHECK(got[1] == std::string("\xA5\x51\x05", 3));

    // Refusals: serial devices, and a duration out of range. Nothing is sent.
    DeviceConfig tty;
    tty.serial = "/dev/null";
    tty.power_limit = 100.0;
    CHECK(!pulse_key(tty, 20, error));
    CHECK(error.find("UDP") != std::string::npos);
    CHECK(!pulse_key(cfg, 5000, error));
    CHECK(error.find("10..2000") != std::string::npos);
}

TEST_CASE("keypad: the key method is refused under either power limit") {
    DeviceConfig cfg;  // the default hard limit is 80
    cfg.host = "127.0.0.1";
    std::string error;
    CHECK(!pulse_key(cfg, 20, error));
    CHECK(error.find("power_limit 80%") != std::string::npos);
    cfg.power_limit = 100.0;
    cfg.max_power = 50.0;
    CHECK(!key_pulse_allowed(cfg, error));
    CHECK(error.find("max_power 50%") != std::string::npos);
    cfg.max_power = 100.0;
    CHECK(key_pulse_allowed(cfg, error));
}
