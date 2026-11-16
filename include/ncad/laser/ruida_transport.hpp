// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// LaserTransport for a Ruida controller, over any DatagramPort.
//
// Send uploads the encoded job; the controller runs it once its end-of-file
// arrives. Frame uploads a travel-only job -- the job's box as laser-off
// moves, with every power at 0 and no cut command in the file -- so the frame
// is placed by exactly the rules that will place the job. Stop is D8 01.
#pragma once

#include "ncad/laser/ruida_link.hpp"
#include "ncad/laser/transport.hpp"

namespace ncad::laser::ruida {

class RuidaTransport final : public LaserTransport {
public:
    RuidaTransport(DatagramPort& port, const DeviceConfig& cfg);

    bool open(std::string& error) override;
    bool send(const LaserJob& planned, const Progress& progress, std::string& error) override;
    bool frame(const LaserJob& planned, std::string& error) override;
    bool stop(std::string& error) override;
    bool status(MachineStatus& out, std::string& error) override;
    bool counters(Counters& out, std::string& error) override;

    const LinkStats& stats() const { return link_.stats(); }

private:
    DeviceConfig cfg_;
    RuidaLink link_;
};

// The bytes Send would upload, and the checks it makes first. Public so
// `ncad-laser send` can show what it is about to do, and so tests can see
// exactly what went on the wire.
bool prepare_send(const LaserJob& planned, const DeviceConfig& cfg, std::string& plain, std::string& error);
bool prepare_frame(const LaserJob& planned, std::string& plain, std::string& error);

}  // namespace ncad::laser::ruida
