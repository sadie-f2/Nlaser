// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

#include "ncad/laser/connect.hpp"

#include "ncad/laser/ruida_transport.hpp"
#include "ncad/laser/serial_port.hpp"
#include "ncad/laser/udp_port.hpp"

namespace ncad::laser {

namespace {

// A RuidaTransport borrows its port; this keeps the port alive exactly as long
// as the transport, so a caller holds one object. Members are declared port
// first so the transport is destroyed before the port it points at.
class OwningRuida final : public LaserTransport {
public:
    OwningRuida(const DeviceConfig& cfg) : transport_(cfg.serial.empty() ? make_udp(cfg) : make_tty(cfg)) {}

    bool open_port(const DeviceConfig& cfg, std::string& error) {
        return cfg.serial.empty() ? udp_.open(cfg.host, cfg.port, cfg.local_port, error)
                                  : tty_.open(cfg.serial, cfg.baud, cfg.hw_flow, error);
    }

    bool open(std::string& error) override { return transport_.open(error); }
    bool send(const LaserJob& j, const Progress& p, std::string& e) override { return transport_.send(j, p, e); }
    bool frame(const LaserJob& j, std::string& e) override { return transport_.frame(j, e); }
    bool dryrun(const LaserJob& j, std::string& e) override { return transport_.dryrun(j, e); }
    bool pulse(double pw, int ms, std::string& e) override { return transport_.pulse(pw, ms, e); }
    bool poll(MachineStatus& o, std::string& e) override { return transport_.poll(o, e); }
    bool stop(std::string& e) override { return transport_.stop(e); }
    bool move_head(MoveFrame f, double x, double y, double sp, MoveReport& r, std::string& e) override {
        return transport_.move_head(f, x, y, sp, r, e);
    }
    bool status(MachineStatus& o, std::string& e) override { return transport_.status(o, e); }
    bool counters(Counters& o, std::string& e) override { return transport_.counters(o, e); }
    std::string note() const override { return transport_.note(); }

private:
    ruida::RuidaTransport make_udp(const DeviceConfig& cfg) { return ruida::RuidaTransport(udp_, cfg); }
    ruida::RuidaTransport make_tty(const DeviceConfig& cfg) { return ruida::RuidaTransport(tty_, cfg); }

    UdpPort udp_;
    SerialPort tty_;
    ruida::RuidaTransport transport_;
};

}  // namespace

bool connect_ruida(const DeviceConfig& cfg, std::unique_ptr<LaserTransport>& out, std::string& error) {
    if (cfg.host.empty() == cfg.serial.empty()) {
        error = "the device needs exactly one of host and serial";
        return false;
    }
    auto t = std::make_unique<OwningRuida>(cfg);
    if (!t->open_port(cfg, error)) return false;
    out = std::move(t);
    return true;
}

}  // namespace ncad::laser
