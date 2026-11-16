// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// A DatagramPort over UDP (POSIX sockets). Lives in ncad_laser_net, the only
// target that includes a socket header; this header itself does not.
//
// The Ruida controller listens on 50200 and answers to 40200, so by default
// the port binds 40200 locally -- which means one program at a time per
// machine can talk to a controller this way (LightBurn included).
#pragma once

#include "ncad/laser/transport.hpp"

#include <string>

namespace ncad::laser {

class UdpPort final : public DatagramPort {
public:
    UdpPort() = default;
    ~UdpPort() override;
    UdpPort(const UdpPort&) = delete;
    UdpPort& operator=(const UdpPort&) = delete;

    // `local_port` 0 binds any free port (tests).
    bool open(const std::string& host, int port, int local_port, std::string& error);
    void close();

    bool send(std::string_view bytes, std::string& error) override;
    int recv(std::string& out, int timeout_ms, std::string& error) override;

    // The port actually bound, after open().
    int local_port() const { return local_port_; }

private:
    int fd_{-1};
    int local_port_{0};
    unsigned char peer_[128]{};  // a sockaddr_storage, kept opaque here
    unsigned peer_len_{0};
};

}  // namespace ncad::laser
