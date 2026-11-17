// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// nlaser_fakectl: the test suite's fake controller on a UDP port, for trying
// ncad-laser end to end with no laser. Prints each job it receives as rd2txt
// would summarise it.
//
//   nlaser_fakectl [--port 50200] [--nak-every N]
//   ncad-laser counters --host 127.0.0.1 --local-port 0

#include "fake_controller.hpp"

#include "ncad/laser/ruida_decode.hpp"
#include "ncad/laser/ruida_link.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>

using namespace ncad::laser::ruida;

int main(int argc, char** argv) {
    int port = 50200;
    int nak_every = 0;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) port = std::atoi(argv[++i]);
        else if (std::strcmp(argv[i], "--nak-every") == 0 && i + 1 < argc) nak_every = std::atoi(argv[++i]);
        else {
            std::fprintf(stderr, "usage: nlaser_fakectl [--port N] [--nak-every N]\n");
            return 2;
        }
    }
    nlaser_test::FakeController fake;
    fake.set_number(kMemCardId, 0x65106510);
    fake.memory[kMemFirmware] = std::string("FAKE-8.01.50\0", 13);
    fake.set_number(kMemMachineStatus, 22);
    fake.set_number(kMemBedX, 900000);
    fake.set_number(kMemBedY, 600000);
    fake.set_number(kMemJobCount, 0);

    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a.sin_port = htons(static_cast<std::uint16_t>(port));
    if (fd < 0 || ::bind(fd, reinterpret_cast<sockaddr*>(&a), sizeof a) != 0) {
        std::perror("nlaser_fakectl: bind");
        return 1;
    }
    std::printf("fake Ruida controller on 127.0.0.1:%d\n", port);
    std::fflush(stdout);

    // A reader that goes away (fakectl piped to `head`, a closed terminal) must
    // not kill the controller halfway through a run.
    std::signal(SIGPIPE, SIG_IGN);

    std::size_t seen = 0;
    std::uint64_t jobs = 0;
    for (;;) {
        // The largest UDP datagram, not the largest a job normally sends: a
        // packet bigger than the buffer is cut short by recvfrom, and --chunk
        // goes up to 65000.
        static char buf[65536];
        sockaddr_in from{};
        socklen_t flen = sizeof from;
        const ssize_t n = ::recvfrom(fd, buf, sizeof buf, 0, reinterpret_cast<sockaddr*>(&from), &flen);
        if (n <= 0) continue;
        if (nak_every > 0 && ++seen % static_cast<std::size_t>(nak_every) == 0) fake.nak_next = 1;
        std::string error, out;
        fake.send(std::string_view(buf, static_cast<std::size_t>(n)), error);
        // Nothing here reads the history, and a long run (watch, settings
        // dumps) would only grow it.
        const bool starts_file = !fake.packets.empty() && fake.packets.back().size() >= 2 &&
                                 static_cast<std::uint8_t>(fake.packets.back()[0]) == 0xD8 &&
                                 static_cast<std::uint8_t>(fake.packets.back()[1]) == 0x12;
        // A file begins with Ref Point 0 (D8 12). Moves from goto and jog that
        // arrived before it are not part of it, and would make its sum
        // "MISMATCH" in the report below.
        if (starts_file) fake.job = fake.packets.back();
        fake.packets.clear();
        constexpr std::size_t kMaxJobBytes = 64u << 20;  // an upload that never ends
        if (fake.job.size() > kMaxJobBytes) fake.job.clear();
        while (fake.recv(out, 0, error) == 1) {
            ::sendto(fd, out.data(), out.size(), 0, reinterpret_cast<sockaddr*>(&from), flen);
        }
        if (fake.stops) {
            std::printf("STOP\n");
            fake.stops = 0;
        }
        if (fake.pauses) {
            std::printf("PAUSE\n");
            fake.pauses = 0;
        }
        if (fake.resumes) {
            std::printf("RESUME\n");
            fake.resumes = 0;
        }
        if (!fake.job.empty() && static_cast<std::uint8_t>(fake.job.back()) == 0xD7) {
            const RdSummary s = summarize_rd(decode_rd(fake.job));
            std::printf("job received: %zu bytes, %zu cuts (%.1f mm), %zu moves, file sum %s, origin %s\n",
                        fake.job.size(), s.cuts, s.cut_mm, s.moves,
                        s.file_sum_stated == s.file_sum_computed ? "ok" : "MISMATCH", s.origin_corner);
            fake.set_number(kMemJobCount, ++jobs);
            fake.job.clear();
        }
        std::fflush(stdout);
    }
}
