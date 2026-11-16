// SPDX-License-Identifier: BSD-3-Clause AND MIT
// Copyright (c) 2026, Sadie Forbes
// Copyright (c) 2021 meerk40t
//
// The handshake follows meerk40t/ruida/udp_connection.py (connect,
// _ruida_handshaker); the serial framing follows meerk40t/ruida/
// usb_transport.py and ruidasession.py. See third_party/meerk40t/NOTICE.

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
//
// --- Serial framing: what MeerK40t (d511944) does, and how sure it is --------
//
// The controller's USB port is an FTDI serial device (0403:6001), and over it
// the protocol is the UDP one with the packet machinery taken off:
//  - Swizzle is the same.                          usb_transport.py:14
//  - No checksum header on what we send:           usb_transport.py:15;
//    _package() adds the 16-bit sum only for 'udp'. ruidasession.py:197-202
//  - No ACK/NAK at all: the ACK wait is entered only for UDP
//    (ruidasession.py:348-349, loop at 353-370), so a command is "sent" once
//    written; there is no NAK, hence no resend.   usb_transport.py:16
//  - ENQ is not answered, so MeerK40t connects by asking for machine status
//    (DA 00 04 00) and waiting for a reply starting DA 01.
//                                                  usb_transport.py:17;
//                                                  ruidasession.py:239-243
//  - After connect it drains whatever else the controller had queued.
//                                                  ruidasession.py:274-280
//  - Replies are swizzled, no checksum, and have no message boundary. The
//    session reads exactly 9 bytes for a memory read -- DA 01, 2 address
//    bytes, 5 value bytes ("Length is USB only"). ruidasession.py:396-404.
//    Every read MeerK40t makes is a 5-byte number (rdjob.py decode_reply,
//    :1339-1361); it never reads the firmware string, so a longer reply is
//    NOT covered by its framing.
//  - 8N1, RTS/CTS and DSR/DTR on, 0.25 s read timeout, and the baud rate is
//    hard-wired to 115200 ("doesn't seem to matter much").
//                                                  usb_transport.py:10-13,
//                                                  :49-55
//
// Where MeerK40t says it is unsure: usb_transport.py:3-5 calls the whole
// module "a work in progress ... reverse engineered using data captured using
// Wireshark"; line 13 ("Baud rate setting should be reasonably high") and
// line 51 are guesses; ruidasession.py has TODOs on lost sync (:393), and
// ruida/README.md:4-12 says only a Ruida RDC6442S + Monport MP570 on Fedora
// has been tested, and lists much else under "Untested". Its
// serial_connection.py (a bare write-through, timeout=0) is not referenced by
// device.py, which uses RuidaSession -- the older path, not evidence.
//
// What follows from that here, and what is OUR choice, not MeerK40t's:
//  - Serial framing is chosen by constructing the link over a StreamPort.
//    A command is written and not acknowledged; a lost byte is invisible
//    until the job fails to run. (Sending is still safe in one respect: the
//    job runs only once its end-of-file arrives.)
//  - A reply is accepted by finding its DA 01 <mem> header in the stream
//    (swallowing stale bytes before it) and then taking the bytes that follow
//    until the line goes quiet for `stream_quiet_ms`. That reads a 5-byte
//    number exactly as MeerK40t does and also tolerates a longer reply
//    (firmware); whether a real controller sends one over USB is unknown.
//  - Input is drained before each query, so a late or stray reply cannot be
//    mistaken for the next answer.
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
    int stream_quiet_ms{30};       // serial: end of a reply = this long with no byte
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
    // UDP framing: checksummed datagrams, ACK/NAK per packet.
    RuidaLink(DatagramPort& port, std::uint8_t magic, const LinkOptions& opts);
    // Serial framing: bare swizzled bytes, no checksum, no ACK/NAK. Two
    // constructors rather than a flag because the port type already says
    // which, and a flag could contradict it.
    RuidaLink(StreamPort& port, std::uint8_t magic, const LinkOptions& opts);

    bool serial_framing() const { return stream_ != nullptr; }

    // UDP: ENQ until an ACK, then drain what the controller had queued.
    // Serial: read machine status until it answers (ENQ gets no reply).
    bool connect(std::string& error);

    // UDP: one acknowledged packet. Serial: written, not acknowledged.
    bool command(std::string_view plain, std::string& error);

    bool query(std::uint16_t mem, MemReply& out, std::string& error);

    // A whole plain .rd stream, split at command boundaries into packets of
    // at most max_packet bytes, each acknowledged before the next.
    bool upload(std::string_view plain, const Progress& progress, std::string& error);

    const LinkStats& stats() const { return stats_; }

private:
    enum class Got : std::uint8_t { Timeout, Failed, Ack, Nak, Err, Enq, Data };
    Got receive(std::string& data, int timeout_ms, std::string& error);

    bool connect_serial(std::string& error);
    bool query_serial(std::uint16_t mem, MemReply& out, std::string& error);
    void drain_stream();

    DatagramPort* dgram_{nullptr};
    StreamPort* stream_{nullptr};
    Swizzle sw_;
    LinkOptions opts_;
    LinkStats stats_;
    bool have_early_reply_{false};
    std::string early_reply_;
};

}  // namespace ncad::laser::ruida
