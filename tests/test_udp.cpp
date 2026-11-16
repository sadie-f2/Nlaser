// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// UdpPort over real sockets on loopback, with the fake controller behind a
// socket on a thread: counters, then a whole job upload, end to end.

#include "test.hpp"

#include "fake_controller.hpp"

#include "ncad/laser/job.hpp"
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
            char buf[2048];
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
