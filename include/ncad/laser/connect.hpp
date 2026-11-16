// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// Opening a configured device: the one place that turns a DeviceConfig into a
// LaserTransport with a real port under it. It lives in the net target
// because that is the only one allowed to include a socket or termios header;
// a front end that wants a transport it can test (the LASER command) takes a
// LaserConnector instead, and the tests hand it one over a fake controller.
#pragma once

#include "ncad/laser/transport.hpp"

#include <functional>
#include <memory>
#include <string>

namespace ncad::laser {

// Builds a transport for `cfg`, ready for open(). False with `error` when the
// port cannot be opened. The transport owns its port.
using LaserConnector =
    std::function<bool(const DeviceConfig& cfg, std::unique_ptr<LaserTransport>& out, std::string& error)>;

// UDP when cfg.host is set, USB serial when cfg.serial is; exactly one.
bool connect_ruida(const DeviceConfig& cfg, std::unique_ptr<LaserTransport>& out, std::string& error);

}  // namespace ncad::laser
