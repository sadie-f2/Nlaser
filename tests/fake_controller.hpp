// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// In-memory Ruida controllers. What a controller holds and records is shared
// (ControllerState); how it frames the wire is not:
//
//  - FakeController is seen as a DatagramPort. It does what udp_connection.py
//    expects of a real one: checks each packet's checksum (NAK when wrong),
//    ACKs, answers DA 00 reads with DA 01 <mem> <value>, and keeps every
//    accepted payload so a test can decode exactly what went over the wire.
//  - FakeSerialController is seen as a StreamPort and frames as
//    usb_transport.py describes: swizzled bytes, no checksum, no ACK, replies
//    to reads only. It parses commands out of a byte stream the way a
//    controller must, since writes arrive in arbitrary pieces over a tty.
//
// Faults are injected by setting the public knobs.
#pragma once

#include "ncad/laser/ruida_codec.hpp"
#include "ncad/laser/transport.hpp"

#include <cstdint>
#include <deque>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace nlaser_test {

struct ControllerState {
    // --- what it holds ---
    std::map<std::uint16_t, std::string> memory;  // raw value bytes per address
    std::set<std::uint16_t> mute;                 // reads of these get no reply

    void set_number(std::uint16_t mem, std::uint64_t v) {
        std::string raw;
        ncad::laser::ruida::put_u35(raw, v);
        memory[mem] = raw;
    }

    // --- what it saw ---
    std::vector<std::string> packets;  // accepted plain payloads, in order
    std::string job;                   // payloads that were not ENQ, reads or STOP
    int enqs{0};
    int stops{0};

    // Takes one accepted plain command; returns the plain reply it owes
    // (empty for none).
    std::string accept(const std::string& plain) {
        packets.push_back(plain);
        const auto b0 = static_cast<std::uint8_t>(plain[0]);
        std::string answer;
        if (plain.size() == 1 && b0 == 0xCE) {
            ++enqs;
        } else if (plain.size() == 4 && b0 == 0xDA && plain[1] == 0x00) {
            const auto mem = static_cast<std::uint16_t>((plain[2] << 7) | plain[3]);
            if (mute.count(mem)) return answer;  // the read is accepted, nothing comes back
            const auto it = memory.find(mem);
            answer = std::string("\xDA\x01", 2) + plain.substr(2, 2);
            answer += (it != memory.end()) ? it->second : std::string(5, '\0');
        } else if (plain.size() == 2 && b0 == 0xD8 && plain[1] == 0x01) {
            ++stops;
        } else {
            job += plain;
        }
        return answer;
    }
};

class FakeController final : public ncad::laser::DatagramPort, public ControllerState {
public:
    explicit FakeController(std::uint8_t magic = 0x88) : sw_(magic) {}

    // --- faults ---
    int nak_next{0};             // NAK this many good packets before accepting
    bool silent{false};          // answer nothing at all
    bool err_next{false};        // answer the next packet with ERR
    bool corrupt_next{false};    // the next packet is damaged in flight
    bool reply_before_ack{false};
    int drop_acks_after{-1};     // after this many accepted packets, stop ACKing

    int naks_sent{0};
    int bad_checksums{0};

    bool send(std::string_view bytes, std::string& error) override {
        (void)error;
        std::string datagram(bytes);
        if (corrupt_next) {
            corrupt_next = false;
            if (datagram.size() > 2) datagram[2] = static_cast<char>(datagram[2] ^ 0x5A);
        }
        if (silent) return true;
        std::string plain;
        if (!ncad::laser::ruida::unpackage_udp(sw_, datagram, plain)) {
            ++bad_checksums;
            ++naks_sent;
            reply(0xCF);
            return true;
        }
        if (err_next) {
            err_next = false;
            reply(0xCD);
            return true;
        }
        if (nak_next > 0) {
            --nak_next;
            ++naks_sent;
            reply(0xCF);
            return true;
        }
        const std::string answer = accept(plain);
        const bool ack = drop_acks_after < 0 || static_cast<int>(packets.size()) <= drop_acks_after;
        if (!answer.empty() && reply_before_ack) inbox_.push_back(sw_.encode(answer));
        if (ack) reply(0xCC);
        if (!answer.empty() && !reply_before_ack) inbox_.push_back(sw_.encode(answer));
        return true;
    }

    int recv(std::string& out, int timeout_ms, std::string& error) override {
        (void)timeout_ms;
        (void)error;
        if (inbox_.empty()) return 0;  // a timeout, instantly: the tests do not wait
        out = inbox_.front();
        inbox_.pop_front();
        return 1;
    }

    // Something the controller says unprompted, e.g. a keep-alive.
    void say(std::uint8_t b) { reply(b); }
    // A whole datagram of plain bytes it pushes by itself (e.g. a status reply nobody asked for).
    void push(const std::string& plain) { inbox_.push_back(sw_.encode(plain)); }

private:
    void reply(std::uint8_t b) { inbox_.push_back(sw_.encode(std::string(1, static_cast<char>(b)))); }

    ncad::laser::ruida::Swizzle sw_;
    std::deque<std::string> inbox_;
};

class FakeSerialController final : public ncad::laser::StreamPort, public ControllerState {
public:
    explicit FakeSerialController(std::uint8_t magic = 0x88) : sw_(magic) {}

    // --- faults ---
    bool silent{false};          // answer nothing at all
    std::size_t read_chunk{0};   // deliver at most this many bytes per read (0 = all)
    std::string stale;           // swizzled bytes already waiting before the first reply
    std::string tail;            // plain bytes sent after every reply (e.g. a longer string)

    // --- what it saw ---
    std::string wire;            // every byte written to it, exactly as written

    bool write(std::string_view bytes, std::string& error) override {
        (void)error;
        wire.append(bytes);
        if (silent) return true;
        pending_ += sw_.decode(bytes);
        parse();
        return true;
    }

    int read(std::string& out, std::size_t max, int timeout_ms, std::string& error) override {
        (void)timeout_ms;
        (void)error;
        if (!stale.empty()) {
            outbox_ = stale + outbox_;
            stale.clear();
        }
        if (outbox_.empty()) return 0;  // a timeout, instantly
        std::size_t n = outbox_.size() < max ? outbox_.size() : max;
        if (read_chunk != 0 && n > read_chunk) n = read_chunk;
        out.append(outbox_, 0, n);
        outbox_.erase(0, n);
        return 1;
    }

    // Plain bytes it says unprompted, e.g. status pushed while cutting.
    void push(const std::string& plain) { outbox_ += sw_.encode(plain); }

    // Everything it has queued to say, swizzled, for a test that carries the
    // bytes elsewhere (a pseudo-terminal).
    std::string take_output() {
        std::string o;
        o.swap(outbox_);
        return o;
    }

private:
    // Commands begin at a byte >= 0x80, so a command is whole once the next
    // one has begun. A few are complete on their own, since nothing
    // follows them: a memory read (DA 00 + 2 address bytes), STOP (D8 01),
    // and the end-of-file (D7) that ends a job.
    void parse() {
        for (;;) {
            if (pending_.empty()) return;
            std::size_t len = 0;
            const auto b0 = static_cast<std::uint8_t>(pending_[0]);
            if (b0 < 0x80) {  // stray argument bytes: not ours to execute
                pending_.erase(0, 1);
                continue;
            }
            if (b0 == 0xDA && pending_.size() >= 2 && pending_[1] == 0x00) {
                if (pending_.size() < 4) return;
                len = 4;
            } else if (b0 == 0xD8 && pending_.size() >= 2 && pending_[1] == 0x01) {
                len = 2;  // STOP
            } else if (b0 == 0xD7 || b0 == 0xCE) {
                len = 1;
            } else {
                for (std::size_t i = 1; i < pending_.size(); ++i) {
                    if (static_cast<std::uint8_t>(pending_[i]) >= 0x80) {
                        len = i;
                        break;
                    }
                }
                if (len == 0) return;  // not complete yet
            }
            const std::string answer = accept(pending_.substr(0, len));
            pending_.erase(0, len);
            if (!answer.empty()) outbox_ += sw_.encode(answer + tail);
        }
    }

    ncad::laser::ruida::Swizzle sw_;
    std::string pending_;  // decoded bytes not yet a whole command
    std::string outbox_;   // swizzled bytes waiting to be read
};

}  // namespace nlaser_test
