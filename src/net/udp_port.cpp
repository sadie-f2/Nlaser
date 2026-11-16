// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

#include "ncad/laser/udp_port.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace ncad::laser {

static_assert(sizeof(sockaddr_storage) <= 128, "peer buffer too small");

UdpPort::~UdpPort() { close(); }

void UdpPort::close() {
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
}

bool UdpPort::open(const std::string& host, int port, int local_port, std::string& error) {
    close();
    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    addrinfo* res = nullptr;
    const std::string service = std::to_string(port);
    const int rc = ::getaddrinfo(host.c_str(), service.c_str(), &hints, &res);
    if (rc != 0 || !res) {
        error = "cannot resolve '" + host + "': " + ::gai_strerror(rc);
        return false;
    }
    std::memcpy(peer_, res->ai_addr, res->ai_addrlen);
    peer_len_ = static_cast<unsigned>(res->ai_addrlen);
    ::freeaddrinfo(res);

    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
        error = std::string("socket: ") + std::strerror(errno);
        return false;
    }
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(static_cast<std::uint16_t>(local_port));
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&local), sizeof local) != 0) {
        error = "cannot bind local UDP port " + std::to_string(local_port) + ": " + std::strerror(errno) +
                (errno == EADDRINUSE ? " (is LightBurn or another sender running?)" : "");
        close();
        return false;
    }
    socklen_t len = sizeof local;
    ::getsockname(fd_, reinterpret_cast<sockaddr*>(&local), &len);
    local_port_ = ntohs(local.sin_port);
    return true;
}

bool UdpPort::send(std::string_view bytes, std::string& error) {
    if (fd_ < 0) {
        error = "port not open";
        return false;
    }
    const ssize_t n = ::sendto(fd_, bytes.data(), bytes.size(), 0, reinterpret_cast<const sockaddr*>(peer_),
                               static_cast<socklen_t>(peer_len_));
    if (n != static_cast<ssize_t>(bytes.size())) {
        error = std::string("sendto: ") + std::strerror(errno);
        return false;
    }
    return true;
}

int UdpPort::recv(std::string& out, int timeout_ms, std::string& error) {
    if (fd_ < 0) {
        error = "port not open";
        return -1;
    }
    const auto* want = reinterpret_cast<const sockaddr_in*>(peer_);
    for (;;) {
        pollfd p{fd_, POLLIN, 0};
        const int r = ::poll(&p, 1, timeout_ms);
        if (r == 0) return 0;
        if (r < 0) {
            if (errno == EINTR) continue;
            error = std::string("poll: ") + std::strerror(errno);
            return -1;
        }
        char buf[2048];
        sockaddr_in from{};
        socklen_t flen = sizeof from;
        const ssize_t n = ::recvfrom(fd_, buf, sizeof buf, 0, reinterpret_cast<sockaddr*>(&from), &flen);
        if (n < 0) {
            if (errno == EINTR) continue;
            error = std::string("recvfrom: ") + std::strerror(errno);
            return -1;
        }
        // Only the device we opened for; anything else on this port is not ours.
        if (from.sin_addr.s_addr != want->sin_addr.s_addr) continue;
        out.assign(buf, static_cast<std::size_t>(n));
        return 1;
    }
}

}  // namespace ncad::laser
