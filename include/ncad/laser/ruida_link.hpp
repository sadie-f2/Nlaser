// SPDX-License-Identifier: BSD-3-Clause AND MIT
// Copyright (c) 2026, Sadie Forbes
// Copyright (c) 2021 meerk40t
//
// The handshake follows meerk40t/ruida/udp_connection.py (connect,
// _ruida_handshaker). See third_party/meerk40t/NOTICE.

// The Ruida protocol over a DatagramPort: handshake, acknowledged commands,
// memory queries, and chunked upload. Headless; no socket in sight.
//
// The protocol speaks only when spoken to. Each datagram we send is a 16-bit
// checksum and a swizzled payload; the controller answers with a swizzled
// single byte -- ACK (CC), NAK (CF: resend that packet), ERR (CD) or ENQ
// (CE, keep-alive) -- and a memory query (DA 00) is answered with a second
// datagram, DA 01 <mem> <value>, which may arrive before the ACK.
//
// **A timeout is a failure, never a resend.** A NAK says the controller
// rejected the packet, so resending it is right. A missing ACK may mean the
// packet arrived and the ACK was lost; resending would then insert the same
// bytes twice into a job. So a timeout ends the transfer with an error and the
// whole job is sent again. The controller runs a job only once its end-of-file
// arrives, so an abandoned upload is never started.
#pragma once

#include "ncad/laser/ruida_codec.hpp"
#include "ncad/laser/transport.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

namespace ncad::laser::ruida {

// Memory addresses are 14-bit values sent as two 7-bit bytes, so the spec's
// "04 01" is address 0x201.
constexpr std::uint16_t mem_addr(std::uint8_t hi, std::uint8_t lo) {
    return static_cast<std::uint16_t>((hi << 7) | lo);
}

inline constexpr std::uint16_t kMemMachineStatus = mem_addr(0x04, 0x00);
inline constexpr std::uint16_t kMemPowerOnTime = mem_addr(0x04, 0x01);
inline constexpr std::uint16_t kMemWorkTime = mem_addr(0x04, 0x02);
inline constexpr std::uint16_t kMemJobCount = mem_addr(0x04, 0x03);
inline constexpr std::uint16_t kMemLaserOnTime = mem_addr(0x04, 0x11);
inline constexpr std::uint16_t kMemPosX = mem_addr(0x04, 0x21);
inline constexpr std::uint16_t kMemTravelX = mem_addr(0x04, 0x23);
inline constexpr std::uint16_t kMemPosY = mem_addr(0x04, 0x31);
inline constexpr std::uint16_t kMemTravelY = mem_addr(0x04, 0x33);
inline constexpr std::uint16_t kMemCardId = mem_addr(0x05, 0x7E);
inline constexpr std::uint16_t kMemFirmware = mem_addr(0x05, 0x7F);
inline constexpr std::uint16_t kMemBedX = mem_addr(0x00, 0x26);
inline constexpr std::uint16_t kMemBedY = mem_addr(0x00, 0x36);
inline constexpr std::uint16_t kMemFileTotal = mem_addr(0x06, 0x20);     // LightBurn writes cut mm here
inline constexpr std::uint16_t kMemFileProgress = mem_addr(0x06, 0x21);  // MeerK40t: "File Progress Len"

struct LinkOptions {
    int ack_timeout_ms{2000};
    int max_naks{8};             // per packet
    std::size_t max_packet{1000};  // payload bytes; split at command boundaries
};

struct LinkStats {
    std::size_t packets{0};
    std::size_t acks{0};
    std::size_t naks{0};
    std::size_t keepalives{0};
    std::size_t replies{0};
};

// A memory read's answer: the raw value bytes, and the first five read as a
// number when there are five.
struct MemReply {
    std::uint16_t mem{0};
    std::string raw;
    bool has_number{false};
    std::uint64_t number{0};
};

class RuidaLink {
public:
    RuidaLink(DatagramPort& port, std::uint8_t magic, const LinkOptions& opts);

    // ENQ until an ACK. Drains anything the controller had queued.
    bool connect(std::string& error);

    // One acknowledged packet.
    bool command(std::string_view plain, std::string& error);

    bool query(std::uint16_t mem, MemReply& out, std::string& error);

    // A whole plain .rd stream, split at command boundaries into packets of
    // at most max_packet bytes, each acknowledged before the next.
    bool upload(std::string_view plain, const Progress& progress, std::string& error);

    const LinkStats& stats() const { return stats_; }

private:
    enum class Got : std::uint8_t { Timeout, Failed, Ack, Nak, Err, Enq, Data };
    Got receive(std::string& data, int timeout_ms, std::string& error);

    DatagramPort& port_;
    Swizzle sw_;
    LinkOptions opts_;
    LinkStats stats_;
    bool have_early_reply_{false};
    std::string early_reply_;
};

}  // namespace ncad::laser::ruida
