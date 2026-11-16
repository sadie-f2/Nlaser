// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// An in-memory Ruida controller, seen from our side as a DatagramPort.
//
// It does what udp_connection.py expects of a real one: checks each packet's
// checksum (NAK when wrong), ACKs, answers DA 00 reads with DA 01 <mem>
// <value>, and keeps every accepted payload so a test can decode exactly what
// went over the wire. Faults are injected by setting the public knobs.
#pragma once

#include "ncad/laser/ruida_codec.hpp"
#include "ncad/laser/transport.hpp"

#include <cstdint>
#include <deque>
#include <map>
#include <string>
#include <vector>

namespace nlaser_test {

class FakeController final : public ncad::laser::DatagramPort {
public:
    explicit FakeController(std::uint8_t magic = 0x88) : sw_(magic) {}

    // --- faults ---
    int nak_next{0};             // NAK this many good packets before accepting
    bool silent{false};          // answer nothing at all
    bool err_next{false};        // answer the next packet with ERR
    bool corrupt_next{false};    // the next packet is damaged in flight
    bool reply_before_ack{false};
    int drop_acks_after{-1};     // after this many accepted packets, stop ACKing

    // --- what it holds ---
    std::map<std::uint16_t, std::string> memory;  // raw value bytes per address

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
        packets.push_back(plain);
        const auto b0 = static_cast<std::uint8_t>(plain[0]);
        std::string answer;
        if (plain.size() == 1 && b0 == 0xCE) {
            ++enqs;
        } else if (plain.size() == 4 && b0 == 0xDA && plain[1] == 0x00) {
            const auto mem = static_cast<std::uint16_t>((plain[2] << 7) | plain[3]);
            const auto it = memory.find(mem);
            answer = std::string("\xDA\x01", 2) + plain.substr(2, 2);
            answer += (it != memory.end()) ? it->second : std::string(5, '\0');
        } else if (plain.size() == 2 && b0 == 0xD8 && plain[1] == 0x01) {
            ++stops;
        } else {
            job += plain;
        }
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

private:
    void reply(std::uint8_t b) { inbox_.push_back(sw_.encode(std::string(1, static_cast<char>(b)))); }

    ncad::laser::ruida::Swizzle sw_;
    std::deque<std::string> inbox_;
};

}  // namespace nlaser_test
