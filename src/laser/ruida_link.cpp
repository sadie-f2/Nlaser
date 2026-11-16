// SPDX-License-Identifier: BSD-3-Clause AND MIT
// Copyright (c) 2026, Sadie Forbes
// Copyright (c) 2021 meerk40t
//
// The handshake follows meerk40t/ruida/udp_connection.py; the serial path
// follows usb_transport.py and ruidasession.py. See third_party/meerk40t/NOTICE.

#include "ncad/laser/ruida_link.hpp"

#include <chrono>

namespace ncad::laser::ruida {

namespace {

constexpr char kAck = static_cast<char>(0xCC);
constexpr char kErr = static_cast<char>(0xCD);
constexpr char kEnq = static_cast<char>(0xCE);
constexpr char kNak = static_cast<char>(0xCF);

using Clock = std::chrono::steady_clock;

int remaining_ms(Clock::time_point deadline) {
    const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
    return left > 0 ? static_cast<int>(left) : 0;
}

}  // namespace

RuidaLink::RuidaLink(DatagramPort& port, std::uint8_t magic, const LinkOptions& opts)
    : dgram_(&port), sw_(magic), opts_(opts) {}

RuidaLink::RuidaLink(StreamPort& port, std::uint8_t magic, const LinkOptions& opts)
    : stream_(&port), sw_(magic), opts_(opts) {}

RuidaLink::Got RuidaLink::receive(std::string& data, int timeout_ms, std::string& error) {
    std::string raw;
    const int r = dgram_->recv(raw, timeout_ms, error);
    if (r < 0) return Got::Failed;
    if (r == 0) return Got::Timeout;
    data = sw_.decode(raw);
    if (data.size() == 1) {
        switch (data[0]) {
            case kAck: ++stats_.acks; return Got::Ack;
            case kNak: ++stats_.naks; return Got::Nak;
            case kErr: return Got::Err;
            case kEnq: ++stats_.keepalives; return Got::Enq;
            default: break;
        }
    }
    ++stats_.replies;
    return Got::Data;
}

bool RuidaLink::connect(std::string& error) {
    if (stream_) return connect_serial(error);
    const std::string enq = package_udp(sw_, std::string(1, kEnq));
    for (int attempt = 0; attempt < 3; ++attempt) {
        if (!dgram_->send(enq, error)) return false;
        ++stats_.packets;
        const auto deadline = Clock::now() + std::chrono::milliseconds(opts_.ack_timeout_ms);
        for (;;) {
            std::string data;
            const Got g = receive(data, remaining_ms(deadline), error);
            if (g == Got::Failed) return false;
            if (g == Got::Timeout) break;
            if (g == Got::Ack) {
                // Drop whatever the controller had been saving up.
                std::string junk, ignore;
                while (dgram_->recv(junk, 50, ignore) == 1) {
                }
                have_early_reply_ = false;
                return true;
            }
        }
    }
    error = "no answer from the controller (3 ENQs, " + std::to_string(opts_.ack_timeout_ms) + " ms each)";
    return false;
}

bool RuidaLink::command(std::string_view plain, std::string& error) {
    if (stream_) {
        // No checksum, no ACK: swizzled bytes onto the wire and done. The OS
        // port blocks (and RTS/CTS throttles) when the controller is behind.
        if (!stream_->write(sw_.encode(plain), error)) return false;
        ++stats_.packets;
        return true;
    }
    const std::string pkt = package_udp(sw_, plain);
    if (!dgram_->send(pkt, error)) return false;
    ++stats_.packets;
    int naks = 0;
    auto deadline = Clock::now() + std::chrono::milliseconds(opts_.ack_timeout_ms);
    for (;;) {
        std::string data;
        const Got g = receive(data, remaining_ms(deadline), error);
        switch (g) {
            case Got::Failed:
                return false;
            case Got::Timeout:
                error = "no acknowledgement within " + std::to_string(opts_.ack_timeout_ms) +
                        " ms (not resent: it may have arrived)";
                return false;
            case Got::Ack:
                return true;
            case Got::Nak:
                if (++naks > opts_.max_naks) {
                    error = "the controller refused a packet " + std::to_string(naks) + " times";
                    return false;
                }
                if (!dgram_->send(pkt, error)) return false;
                ++stats_.packets;
                deadline = Clock::now() + std::chrono::milliseconds(opts_.ack_timeout_ms);
                break;
            case Got::Err:
                error = "the controller reported an error (ERR)";
                return false;
            case Got::Enq:
                break;
            case Got::Data:
                // A reply before its ACK, which MeerK40t also sees. It answers
                // this command, so it stands in for the ACK.
                have_early_reply_ = true;
                early_reply_ = data;
                return true;
        }
    }
}

bool RuidaLink::query(std::uint16_t mem, MemReply& out, std::string& error) {
    if (stream_) return query_serial(mem, out, error);
    have_early_reply_ = false;
    std::string cmd;
    cmd.push_back(static_cast<char>(0xDA));
    cmd.push_back(0x00);
    cmd.push_back(static_cast<char>((mem >> 7) & 0x7F));
    cmd.push_back(static_cast<char>(mem & 0x7F));
    if (!command(cmd, error)) return false;

    auto parse = [&](const std::string& d) {
        if (d.size() < 4 || static_cast<std::uint8_t>(d[0]) != 0xDA || d[1] != 0x01 || d[2] != cmd[2] ||
            d[3] != cmd[3]) {
            return false;
        }
        MemReply r;
        r.mem = mem;
        r.raw = d.substr(4);
        if (r.raw.size() >= 5) {
            r.has_number = true;
            r.number = get_u35(reinterpret_cast<const std::uint8_t*>(r.raw.data()));
        }
        out = std::move(r);
        return true;
    };

    if (have_early_reply_) {
        have_early_reply_ = false;
        if (parse(early_reply_)) return true;
    }
    const auto deadline = Clock::now() + std::chrono::milliseconds(opts_.ack_timeout_ms);
    for (;;) {
        std::string data;
        const Got g = receive(data, remaining_ms(deadline), error);
        if (g == Got::Failed) return false;
        if (g == Got::Timeout) {
            error = "no reply to memory read " + std::to_string(mem);
            return false;
        }
        if (g == Got::Data && parse(data)) return true;
        // ACKs, keep-alives and replies to something else: keep waiting.
    }
}

// --- serial ------------------------------------------------------------------

void RuidaLink::drain_stream() {
    std::string junk, ignore;
    // Bounded, so a controller that never stops talking cannot hang us.
    for (int i = 0; i < 64; ++i) {
        junk.clear();
        if (stream_->read(junk, 4096, 0, ignore) != 1) break;
    }
}

bool RuidaLink::query_serial(std::uint16_t mem, MemReply& out, std::string& error) {
    drain_stream();
    std::string cmd;
    cmd.push_back(static_cast<char>(0xDA));
    cmd.push_back(0x00);
    cmd.push_back(static_cast<char>((mem >> 7) & 0x7F));
    cmd.push_back(static_cast<char>(mem & 0x7F));
    if (!stream_->write(sw_.encode(cmd), error)) return false;
    ++stats_.packets;

    // DA 01 <mem> then 5 value bytes: the 9 bytes MeerK40t reads. Anything
    // before the header is stale and is skipped.
    const std::string header = std::string("\xDA\x01", 2) + cmd.substr(2, 2);
    constexpr std::size_t kMaxBuffered = 4096;
    constexpr std::size_t kNumber = 5;
    std::string plain;  // decoded; the swizzle is per byte, so chunks decode alone
    std::size_t at = std::string::npos;
    const auto deadline = Clock::now() + std::chrono::milliseconds(opts_.ack_timeout_ms);
    for (;;) {
        if (at != std::string::npos && plain.size() >= at + header.size() + kNumber) break;
        if (plain.size() > kMaxBuffered) {
            error = "no reply to memory read " + std::to_string(mem) + " (the stream is not Ruida replies)";
            return false;
        }
        std::string chunk;
        const int r = stream_->read(chunk, 256, remaining_ms(deadline), error);
        if (r < 0) return false;
        if (r == 0) {
            error = "no reply to memory read " + std::to_string(mem);
            return false;
        }
        plain += sw_.decode(chunk);
        if (at == std::string::npos) at = plain.find(header);
    }
    // The tail: nothing more for `stream_quiet_ms` ends the reply. Costs that
    // much per read; fine for a link that is not time-critical.
    for (;;) {
        std::string chunk;
        const int r = stream_->read(chunk, 256, opts_.stream_quiet_ms, error);
        if (r < 0) return false;
        if (r == 0) break;
        plain += sw_.decode(chunk);
        if (plain.size() > kMaxBuffered) break;
    }
    ++stats_.replies;
    MemReply rep;
    rep.mem = mem;
    rep.raw = plain.substr(at + header.size());
    rep.has_number = true;
    rep.number = get_u35(reinterpret_cast<const std::uint8_t*>(rep.raw.data()));
    out = std::move(rep);
    return true;
}

bool RuidaLink::connect_serial(std::string& error) {
    // ENQ gets no answer over USB, so ask for machine status instead, as
    // MeerK40t does, and take any reply as proof the controller is there.
    MemReply r;
    std::string last;
    for (int attempt = 0; attempt < 3; ++attempt) {
        if (query_serial(kMemMachineStatus, r, last)) {
            drain_stream();
            return true;
        }
    }
    error = "no answer from the controller on the serial port (3 status reads, " +
            std::to_string(opts_.ack_timeout_ms) + " ms each): " + last;
    return false;
}

bool RuidaLink::upload(std::string_view plain, const Progress& progress, std::string& error) {
    const std::vector<std::string_view> cmds = split_commands(plain);
    std::size_t done = 0;
    std::size_t i = 0;
    while (i < cmds.size()) {
        // Gather whole commands up to max_packet; one longer than that goes
        // alone, never split.
        const char* start = cmds[i].data();
        std::size_t len = cmds[i].size();
        ++i;
        while (i < cmds.size() && len + cmds[i].size() <= opts_.max_packet) {
            len += cmds[i].size();
            ++i;
        }
        if (!command(std::string_view(start, len), error)) {
            error = "upload failed at byte " + std::to_string(done) + " of " + std::to_string(plain.size()) +
                    ": " + error;
            return false;
        }
        done += len;
        if (progress) progress(done, plain.size());
    }
    return true;
}

}  // namespace ncad::laser::ruida
