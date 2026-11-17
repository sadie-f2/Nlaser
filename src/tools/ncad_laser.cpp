// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// ncad-laser: the file-driven front end: dxf2ljob, plan, encode, rd2txt, the
// devices listing, and the device commands counters, status, watch, stop, frame,
// dryrun, pulse and send.

#include "ncad/laser/device_config.hpp"
#include "ncad/laser/job.hpp"
#include "ncad/laser/lbrn2.hpp"
#include "ncad/laser/move.hpp"
#include "ncad/laser/plan.hpp"
#include "ncad/laser/raster.hpp"
#include "ncad/laser/ruida_codec.hpp"
#include "ncad/laser/ruida_decode.hpp"
#include "ncad/laser/ruida_encode.hpp"
#include "ncad/laser/ruida_job.hpp"
#ifdef NLASER_WITH_NOTOCAD
#include "ncad/laser/laser_renderer.hpp"
#endif
#ifdef NLASER_WITH_NET
#include "ncad/laser/keypad.hpp"
#include "ncad/laser/ruida_memory.hpp"
#include "ncad/laser/ruida_transport.hpp"
#include "ncad/laser/serial_port.hpp"
#include "ncad/laser/pdf_import.hpp"
#include "ncad/laser/svg_import.hpp"
#include "ncad/laser/udp_port.hpp"
#endif

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <vector>
#include <fstream>
#include <iterator>
#include <string>
#include <thread>

namespace {

using namespace ncad::laser::ruida;

void usage() {
    std::fputs(
        "usage: ncad-laser dxf2ljob IN.dxf -o OUT.ljob [--pens FILE.lpt] [--tol MM] [--origin X,Y]\n"
        "       ncad-laser svg2ljob IN.svg -o OUT.ljob [--pens FILE.lpt] [--px-per-in N] [--tol MM]\n"
        "       ncad-laser pdf2ljob IN.pdf|IN.ai -o OUT.ljob [--page N] [--pens FILE.lpt] [--tol MM]\n"
        "       ncad-laser pdfstream2ljob STREAM.txt -o OUT.ljob [--mediabox X0,Y0,X1,Y1] [--pens FILE.lpt] [--tol MM]\n"
        "       ncad-laser plan   IN.ljob -o OUT.ljob [--join-tol MM]\n"
        "       ncad-laser rasterplan IN.png [--size W,H] [--dpi N] [--dither threshold|floyd|jarvis|stucki|bayer|grey]\n"
        "                  [--speed MM_S] [--accel MM_S2] [--margin MM] [--bidir on|off] [--power MIN,MAX]\n"
        "                  [--black N] [--white N] [--gamma G] [--invert] [--no-serpentine] [--png OUT.png]\n"
        "       ncad-laser encode PLANNED.ljob -o OUT.rd [--origin WHERE] [--machine-origin bl|br|tl|tr] [--start current|absolute] [--magic N]\n"
        "       ncad-laser lbrn2ljob IN.lbrn2 -o OUT.ljob [--machine-origin bl|br|tl|tr] [--bed W,H] [--tol MM]\n"
        "                            [--start current|absolute] [--job-origin bl|br|tl|tr|c]\n"
        "       ncad-laser ljob2lbrn IN.ljob -o OUT.lbrn2 [--machine-origin bl|br|tl|tr] [--bed W,H]\n"
        "       ncad-laser rd2txt FILE.rd [options]\n"
        "       ncad-laser devices [--config PATH]\n"
        "       ncad-laser counters WHERE [--csv]\n"
        "       ncad-laser status   WHERE\n"
        "       ncad-laser watch    WHERE [--interval MS] [--duration S] [--csv]\n"
        "       ncad-laser watch    WHERE --listen [--duration S]\n"
        "       ncad-laser goto     X Y WHERE [--machine] [--speed MM_S]\n"
        "       ncad-laser jog      DIR... | DX DY WHERE [--fine|--coarse|--step MM] [--speed MM_S]\n"
        "       ncad-laser settings WHERE [--from ADDR] [--to ADDR] [--csv]\n"
        "       ncad-laser stop     WHERE\n"
        "       ncad-laser frame    PLANNED.ljob WHERE\n"
        "       ncad-laser dryrun   PLANNED.ljob WHERE\n"
        "       ncad-laser pulse    WHERE --power PCT [--ms N] [--method job|key] [--yes]\n"
        "       ncad-laser send     PLANNED.ljob WHERE [--max-power PCT] [--yes]\n"
        "         WHERE is --host IP, --serial DEV or --device NAME\n"
        "\n"
        "dxf2ljob (a raw job; plan it next):\n"
        "  --pens FILE.lpt  pen settings: 'layer' lines as in .ljob; a pen with\n"
        "                   none runs at power 0\n"
        "  --tol MM         chord tolerance for curves (default 0.01, LASERTOL)\n"
        "  --origin X,Y     job origin (default: the LASER UCS's origin, else 0,0)\n"
        "svg2ljob (a raw job, no NotoCAD needed; plan it next):\n"
        "  --px-per-in N    pixels per inch for an SVG with no real units (default 96;\n"
        "                   Illustrator writes 72)\n"
        "  --pens, --tol    as dxf2ljob; curves are flattened at --tol (default 0.01)\n"
        "pdf2ljob, pdfstream2ljob (PDF units are 1/72 in; needs qpdf unless a raw stream):\n"
        "  --page N         the page to import (default 1)\n"
        "  --mediabox X0,Y0,X1,Y1  for pdfstream2ljob: the page box (default 0,0,612,792)\n"
        "  --pens, --tol    as svg2ljob\n"
        "encode:\n"
        "  --origin WHERE  override the job's origin: x,y in job mm, or a corner of\n"
        "                  the job's box: bl, br, tl, tr (LightBurn's anchoring)\n"
        "  --machine-origin bl|br|tl|tr\n"
        "                  the machine's origin corner (default bl); coordinates are\n"
        "                  mapped to its axes, as LightBurn does\n"
        "  --start current|absolute\n"
        "                  where the controller places the job: from the head's current\n"
        "                  position (default) or at absolute machine coordinates;\n"
        "                  overrides the job's start=\n"
        "  --pointer-offset X,Y\n"
        "                  the red-dot pointer offset in mm (LightBurn's device setting),\n"
        "                  added to every move as LightBurn does (default 0,0)\n"
        "  --magic N       swizzle magic (default 0x88)\n"
        "device (counters, status, watch, stop, frame, dryrun, pulse, send):\n"
        "  --device NAME    a [NAME] section of lasers.conf; may be omitted when the file\n"
        "                   has exactly one device and no --host is given\n"
        "  --config PATH    lasers.conf to read (default $XDG_CONFIG_HOME/ncad/lasers.conf,\n"
        "                   else ~/.config/ncad/lasers.conf)\n"
        "  --host IP        the controller's address (UDP) ...\n"
        "  --serial DEV     ... or its USB serial device, e.g. /dev/cu.usbserial-X or\n"
        "                   /dev/ttyUSB0 (exactly one of the two)\n"
        "  --baud N         serial speed (default 115200; USB bridges ignore it)\n"
        "  --no-flow        serial: no RTS/CTS flow control\n"
        "  --magic N        swizzle magic (default 0x88)\n"
        "  --port N         controller UDP port (default 50200)\n"
        "  --local-port N   our UDP port (default 40200; the controller answers there)\n"
        "  --chunk N        largest packet payload in bytes (default 1000)\n"
        "  --timeout MS     wait for each acknowledgement (default 2000)\n"
        "  --pointer-offset X,Y  the red-dot offset in mm (lasers.conf pointer_offset): added by\n"
        "                   send and dryrun (the beam), not by frame, goto or jog (the dot)\n"
        "  --max-power PCT  send refuses any layer above this (default 100)\n"
        "  --keypad-port N  controller keypad UDP port for pulse --method key (default 50207)\n"
        "  Flags override the same setting from the device's section.\n"
        "watch: polls state, head x/y and file progress, one line per poll, until\n"
        "  --duration S (default: until Ctrl-C). --interval MS between polls (default 500).\n"
        "  A poll with no reply is printed as such and the watch goes on.\n"
        "watch --listen: sends nothing after the initial contact (UDP: one ENQ; serial: none) and\n"
        "  logs every datagram or burst of bytes that arrives, timestamped, decoded where it is\n"
        "  DA 01 replies, raw hex otherwise: does the controller push status while cutting?\n"
        "send/frame/dryrun from the head's position read the head and the bed first and refuse\n"
        "  a job whose box would leave the bed; if either cannot be read they WARN and go on.\n"
        "settings: read-only dump of controller memory (DA 00 reads only). By default every\n"
        "  address MeerK40t's emulator names; --from/--to (0xNNNN, or wire bytes HH:LL) read every\n"
        "  address in a range. Prints the address as 14-bit hex and wire bytes, the name, the raw\n"
        "  value (number, and text if it looks like text); '*' marks red/light/pointer/offset.\n"
        "  A read with no reply is marked and the dump goes on; --timeout MS per read (default 500).\n"
        "goto: moves the head, laser off, to a point: X Y are BED coordinates (0,0 at the\n"
        "  bed's bottom-left, X right, Y up; needs the bed size on a machine that counts from\n"
        "  its right or top edge) or, with --machine, raw machine coordinates. A point outside\n"
        "  the bed is refused.\n"
        "jog: reads the head, adds a step and goes there. DIR is +x -x +y -y (up to one per\n"
        "  axis, in bed directions: +x is always to the right), a step of 1 mm, --fine 0.1,\n"
        "  --coarse 10 or --step MM; or DX DY in mm (no step flag). --speed MM_S for either\n"
        "  sends C9 02 first, as MeerK40t does: UNVERIFIED on this controller.\n"
        "dryrun: uploads the planned path as a laser-off job (every cut a move, every\n"
        "  power 0, no cut command in the file) and the controller traces it at once.\n"
        "pulse FIRES THE LASER; it asks for 'yes' unless --yes.\n"
        "  --method job (default): a one-cut job at the head's position, at --power PCT,\n"
        "    1 mm/s for --ms N (10..2000, default 500), so the head moves N/1000 mm.\n"
        "    Refused above the device's max power. UDP or serial.\n"
        "  --method key: the panel's Pulse key, held --ms. UDP only. The power is the one\n"
        "    set on the controller's panel, NOT --power, and cannot be checked against\n"
        "    max_power or power_limit, so it is refused unless both are 100.\n"
        "rd2txt:\n"
        "  --magic N       unswizzle with magic N (e.g. 0x88); default: guess\n"
        "  --machine-origin bl|br|tl|tr   name the origin corner in design terms\n"
        "  --plain         input is already unswizzled\n"
        "  --structure     no offsets, no hex: for diffing two files\n"
        "  --no-offsets    --no-hex    --no-positions    --no-summary\n",
        stderr);
}

bool parse_magic(const char* s, int& magic) {
    char* end = nullptr;
    const long v = std::strtol(s, &end, 0);
    if (!end || *end || v < 0 || v > 255) return false;
    magic = static_cast<int>(v);
    return true;
}

// lasers.conf. An explicit --config must exist; the default location may be
// absent, which just means no devices are configured.
bool load_devices(const std::string& explicit_path, std::vector<ncad::laser::DeviceConfig>& devices,
                  std::string& error) {
    std::string path = explicit_path;
    if (path.empty()) {
        path = ncad::laser::default_device_config_path();
        if (path.empty() || !std::ifstream(path)) {
            devices.clear();
            return true;
        }
    }
    return ncad::laser::read_device_config_file(path, devices, error);
}

#ifdef NLASER_WITH_NET
// Value of `--NAME VALUE` anywhere in the arguments, or "" when absent.
std::string option_value(int argc, char** argv, const char* name) {
    std::string v;
    for (int i = 0; i + 1 < argc; ++i) {
        if (std::strcmp(argv[i], name) == 0) v = argv[i + 1];
    }
    return v;
}
#endif

// "W,H" or "WxH", mm, both positive.
bool parse_bed(const char* s, double& w, double& h) {
    char sep = 0, tail = 0;
    double a = 0.0, b = 0.0;
    if (std::sscanf(s, "%lf%c%lf%c", &a, &sep, &b, &tail) != 3 || (sep != ',' && sep != 'x') || !(a > 0.0) ||
        !(b > 0.0)) {
        return false;
    }
    w = a;
    h = b;
    return true;
}

// "X,Y", mm, any sign.
bool parse_xy(const char* s, double& x, double& y) {
    char tail = 0;
    double a = 0.0, b = 0.0;
    if (std::sscanf(s, "%lf,%lf%c", &a, &b, &tail) != 2 || !std::isfinite(a) || !std::isfinite(b)) return false;
    x = a;
    y = b;
    return true;
}

int devices_command(int argc, char** argv) {
    std::string config;
    for (int i = 0; i < argc; ++i) {
        if (std::strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            config = argv[++i];
        } else {
            usage();
            return 2;
        }
    }
    std::vector<ncad::laser::DeviceConfig> devices;
    std::string error;
    if (!load_devices(config, devices, error)) {
        std::fprintf(stderr, "ncad-laser: %s\n", error.c_str());
        return 1;
    }
    if (devices.empty()) {
        std::printf("no devices configured (%s)\n",
                    config.empty() ? ncad::laser::default_device_config_path().c_str() : config.c_str());
        return 0;
    }
    for (const ncad::laser::DeviceConfig& d : devices) {
        std::printf("%-16s %s  port %d, magic 0x%02X", d.name.c_str(), d.host.empty() ? "(no host)" : d.host.c_str(),
                    d.port, d.magic);
        std::printf(", power_limit %g%%", d.power_limit);
        if (d.bed_w_mm > 0.0) std::printf(", bed %g x %g mm", d.bed_w_mm, d.bed_h_mm);
        if (d.pointer_offset_x != 0.0 || d.pointer_offset_y != 0.0) {
            std::printf(", pointer offset %g, %g mm", d.pointer_offset_x, d.pointer_offset_y);
        }
        std::printf("\n");
    }
    return 0;
}

bool write_file(const char* path, const std::string& data) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) return false;
    out << data;
    return static_cast<bool>(out);
}

int dxf2ljob(int argc, char** argv) {
#ifdef NLASER_WITH_NOTOCAD
    using namespace ncad::laser;
    const char* in = nullptr;
    const char* out = nullptr;
    const char* pens_path = nullptr;
    DrawingOptions opts;
    for (int i = 0; i < argc; ++i) {
        if (std::strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            out = argv[++i];
        } else if (std::strcmp(argv[i], "--pens") == 0 && i + 1 < argc) {
            pens_path = argv[++i];
        } else if (std::strcmp(argv[i], "--tol") == 0 && i + 1 < argc) {
            char* end = nullptr;
            opts.chord_tolerance = std::strtod(argv[++i], &end);
            if (!end || *end || !(opts.chord_tolerance > 0.0)) {
                std::fprintf(stderr, "ncad-laser: bad --tol '%s'\n", argv[i]);
                return 2;
            }
        } else if (std::strcmp(argv[i], "--origin") == 0 && i + 1 < argc) {
            char tail = 0;
            if (std::sscanf(argv[++i], "%lf,%lf%c", &opts.origin.x, &opts.origin.y, &tail) != 2) {
                std::fprintf(stderr, "ncad-laser: bad --origin '%s'\n", argv[i]);
                return 2;
            }
            opts.has_origin = true;
        } else if (argv[i][0] != '-' && !in) {
            in = argv[i];
        } else {
            usage();
            return 2;
        }
    }
    if (!in || !out) {
        usage();
        return 2;
    }
    std::vector<LaserLayer> pens;
    LaserJob job;
    DrawingReport report;
    std::string error;
    if ((pens_path && !read_pen_table_file(pens_path, pens, error)) ||
        !dxf_to_job(in, opts, pens, job, report, error) || !write_ljob_file(out, job, error)) {
        std::fprintf(stderr, "ncad-laser: %s\n", error.c_str());
        return 1;
    }
    for (const std::string& w : report.warnings) std::fprintf(stderr, "ncad-laser: warning: %s\n", w.c_str());
    std::printf("%s: %zu paths on %zu pens from %zu entities (%zu on off or frozen layers not cut)\n", out,
                job.paths.size(), job.layers.size(), report.entities, report.hidden + report.hidden_in_blocks);
    std::printf("origin %.3f,%.3f%s; raw: plan it next\n", job.origin.x, job.origin.y,
                opts.has_origin ? " (--origin)" : report.laser_ucs ? " (LASER UCS)" : " (world)");
    return 0;
#else
    (void)argc;
    (void)argv;
    std::fprintf(stderr, "ncad-laser: built without NLASER_WITH_NOTOCAD; 'dxf2ljob' needs NotoCAD\n");
    return 2;
#endif
}

// A pen table is the `layer` lines of an .ljob. The same reading as the NotoCAD
// bridge's, kept here because svg2ljob must work without that bridge.
bool read_pens(const char* path, std::vector<ncad::laser::LaserLayer>& pens, std::string& error) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        error = std::string("cannot read '") + path + "'";
        return false;
    }
    const std::string text{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    ncad::laser::LaserJob j;
    if (!ncad::laser::read_ljob("ljob 1\n" + text, j, error)) {
        error = std::string(path) + ": " + error;
        return false;
    }
    if (!j.paths.empty()) {
        error = std::string(path) + ": a pen table holds 'layer' lines only";
        return false;
    }
    pens = std::move(j.layers);
    return true;
}

int svg2ljob(int argc, char** argv) {
    using namespace ncad::laser;
    const char* in = nullptr;
    const char* out = nullptr;
    const char* pens_path = nullptr;
    SvgOptions opts;
    double tol = 0.01;
    for (int i = 0; i < argc; ++i) {
        char* end = nullptr;
        if (std::strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            out = argv[++i];
        } else if (std::strcmp(argv[i], "--pens") == 0 && i + 1 < argc) {
            pens_path = argv[++i];
        } else if (std::strcmp(argv[i], "--tol") == 0 && i + 1 < argc) {
            tol = std::strtod(argv[++i], &end);
            if (!end || *end || !(tol > 0.0)) {
                std::fprintf(stderr, "ncad-laser: bad --tol '%s'\n", argv[i]);
                return 2;
            }
        } else if (std::strcmp(argv[i], "--px-per-in") == 0 && i + 1 < argc) {
            opts.px_per_in = std::strtod(argv[++i], &end);
            if (!end || *end || !(opts.px_per_in > 0.0)) {
                std::fprintf(stderr, "ncad-laser: bad --px-per-in '%s'\n", argv[i]);
                return 2;
            }
        } else if (argv[i][0] != '-' && !in) {
            in = argv[i];
        } else {
            usage();
            return 2;
        }
    }
    if (!in || !out) {
        usage();
        return 2;
    }
    std::vector<LaserLayer> pens;
    SvgImport svg;
    LaserJob job;
    std::vector<std::string> warnings;
    std::string error;
    if ((pens_path && !read_pens(pens_path, pens, error)) || !import_svg_file(in, opts, svg, error) ||
        !svg_to_job(svg, tol, pens, in, job, warnings, error) || !write_ljob_file(out, job, error)) {
        std::fprintf(stderr, "ncad-laser: %s\n", error.c_str());
        return 1;
    }
    for (const std::string& w : warnings) std::fprintf(stderr, "ncad-laser: warning: %s\n", w.c_str());
    std::printf("%s: %zu paths on %zu pens from %zu shapes (%.1f x %.1f mm)\n", out, job.paths.size(),
                job.layers.size(), svg.shapes.size(), svg.width_mm, svg.height_mm);
    std::printf("origin 0,0; raw: plan it next\n");
    return 0;
}

// pdf2ljob and pdfstream2ljob: the same flattening as svg2ljob, from a PDF page
// (needs qpdf) or from a raw, already decompressed content stream (does not).
int pdf2ljob(int argc, char** argv, bool raw_stream) {
    using namespace ncad::laser;
    const char* in = nullptr;
    const char* out = nullptr;
    const char* pens_path = nullptr;
    PdfOptions opts;
    PdfPageData page;
    double tol = 0.01;
    for (int i = 0; i < argc; ++i) {
        char* end = nullptr;
        if (std::strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            out = argv[++i];
        } else if (std::strcmp(argv[i], "--pens") == 0 && i + 1 < argc) {
            pens_path = argv[++i];
        } else if (std::strcmp(argv[i], "--tol") == 0 && i + 1 < argc) {
            tol = std::strtod(argv[++i], &end);
            if (!end || *end || !(tol > 0.0)) {
                std::fprintf(stderr, "ncad-laser: bad --tol '%s'\n", argv[i]);
                return 2;
            }
        } else if (!raw_stream && std::strcmp(argv[i], "--page") == 0 && i + 1 < argc) {
            const long n = std::strtol(argv[++i], &end, 10);
            if (!end || *end || n < 1 || n > 100000) {
                std::fprintf(stderr, "ncad-laser: bad --page '%s'\n", argv[i]);
                return 2;
            }
            opts.page = static_cast<int>(n);
        } else if (raw_stream && std::strcmp(argv[i], "--mediabox") == 0 && i + 1 < argc) {
            const char* a = argv[++i];
            int used = 0;
            if (std::sscanf(a, "%lf,%lf,%lf,%lf%n", &page.box[0], &page.box[1], &page.box[2], &page.box[3], &used) != 4 ||
                a[used] != '\0') {
                std::fprintf(stderr, "ncad-laser: bad --mediabox '%s'\n", a);
                return 2;
            }
        } else if (argv[i][0] != '-' && !in) {
            in = argv[i];
        } else {
            usage();
            return 2;
        }
    }
    if (!in || !out) {
        usage();
        return 2;
    }
    std::vector<LaserLayer> pens;
    SvgImport shapes;
    LaserJob job;
    std::vector<std::string> warnings;
    std::string error;
    bool ok = !pens_path || read_pens(pens_path, pens, error);
    if (ok && raw_stream) {
        std::ifstream f(in, std::ios::binary);
        if (!f) {
            error = std::string("cannot read '") + in + "'";
            ok = false;
        } else {
            page.content.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
            ok = import_pdf_content(page, shapes, error);
        }
    } else if (ok) {
        ok = import_pdf_file(in, opts, shapes, error);
    }
    if (!ok || !svg_to_job(shapes, tol, pens, in, job, warnings, error) || !write_ljob_file(out, job, error)) {
        std::fprintf(stderr, "ncad-laser: %s\n", error.c_str());
        return 1;
    }
    for (const std::string& w : warnings) std::fprintf(stderr, "ncad-laser: warning: %s\n", w.c_str());
    std::printf("%s: %zu paths on %zu pens from %zu shapes (%.1f x %.1f mm)\n", out, job.paths.size(),
                job.layers.size(), shapes.shapes.size(), shapes.width_mm, shapes.height_mm);
    std::printf("origin 0,0; raw: plan it next\n");
    return 0;
}

int plan(int argc, char** argv) {
    const char* in = nullptr;
    const char* out = nullptr;
    ncad::laser::PlanOptions opts;
    for (int i = 0; i < argc; ++i) {
        if (std::strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            out = argv[++i];
        } else if (std::strcmp(argv[i], "--join-tol") == 0 && i + 1 < argc) {
            char* end = nullptr;
            opts.join_tol = std::strtod(argv[++i], &end);
            if (!end || *end || !(opts.join_tol >= 0.0)) {
                std::fprintf(stderr, "ncad-laser: bad --join-tol '%s'\n", argv[i]);
                return 2;
            }
        } else if (argv[i][0] != '-' && !in) {
            in = argv[i];
        } else {
            usage();
            return 2;
        }
    }
    if (!in || !out) {
        usage();
        return 2;
    }
    ncad::laser::LaserJob raw, planned;
    ncad::laser::PlanReport report;
    std::string error;
    if (!ncad::laser::read_ljob_file(in, raw, error) ||
        !ncad::laser::plan_job(raw, planned, opts, report, error) ||
        !ncad::laser::write_ljob_file(out, planned, error)) {
        std::fprintf(stderr, "ncad-laser: %s\n", error.c_str());
        return 1;
    }
    std::printf("%zu paths in, %zu out (%zu joins, %zu closed by joining, %zu skipped)\n", report.paths_in,
                report.paths_out, report.joins, report.closed_by_join, report.skipped);
    std::printf("cut %.3f mm, travel %.3f mm\n", report.cut_mm, report.travel_mm);
    std::printf("%zu regions; %.3f mm of travel still crosses cut-free parts\n", report.regions,
                report.crossing_mm);
    std::printf("estimated %.1f s (cut %.1f s, travel %.1f s)\n", report.estimated_s, report.cut_s,
                report.travel_s);
    for (const std::string& w : report.warnings) std::printf("warning: %s\n", w.c_str());
    return 0;
}

// Plans a bitmap as a raster and reports on it; sends nothing. --png writes
// the dithered pixels (fired = black) to look at.
int rasterplan(int argc, char** argv) {
    using namespace ncad::laser;
    const char* in = nullptr;
    const char* png = nullptr;
    double w = 0.0, h = 0.0;
    LaserRaster place;
    LaserLayer layer;
    layer.speed = 300.0;
    layer.max_power = 20.0;
    RasterPrepOptions po;
    auto num = [](const char* s, double& v) {
        char* end = nullptr;
        v = std::strtod(s, &end);
        return end && !*end && s[0] && std::isfinite(v);
    };
    auto bad = [](const char* what, const char* v) {
        std::fprintf(stderr, "ncad-laser: bad %s '%s'\n", what, v);
        return 2;
    };
    for (int i = 0; i < argc; ++i) {
        const std::string a = argv[i];
        const bool more = i + 1 < argc;
        double v = 0.0;
        if (a == "--size" && more) {
            if (!parse_xy(argv[++i], w, h) || w <= 0.0 || h <= 0.0) return bad("--size (W,H in mm)", argv[i]);
        } else if (a == "--dpi" && more) {
            if (!num(argv[++i], place.dpi) || place.dpi <= 0.0) return bad("--dpi", argv[i]);
        } else if (a == "--dither" && more) {
            if (!parse_dither(argv[++i], place.dither)) return bad("--dither", argv[i]);
        } else if (a == "--speed" && more) {
            if (!num(argv[++i], layer.speed) || layer.speed <= 0.0) return bad("--speed", argv[i]);
        } else if (a == "--accel" && more) {
            if (!num(argv[++i], po.plan.accel_mm_s2)) return bad("--accel", argv[i]);
        } else if (a == "--margin" && more) {
            if (!num(argv[++i], po.plan.margin_mm)) return bad("--margin", argv[i]);
        } else if (a == "--bidir" && more) {
            const std::string b = argv[++i];
            if (b != "on" && b != "off") return bad("--bidir (on or off)", argv[i]);
            place.bidir = b == "on";
        } else if (a == "--power" && more) {
            if (!parse_xy(argv[++i], layer.min_power, layer.max_power)) return bad("--power (MIN,MAX)", argv[i]);
        } else if (a == "--black" && more) {
            if (!num(argv[++i], v)) return bad("--black", argv[i]);
            po.levels.black = static_cast<int>(v);
        } else if (a == "--white" && more) {
            if (!num(argv[++i], v)) return bad("--white", argv[i]);
            po.levels.white = static_cast<int>(v);
        } else if (a == "--gamma" && more) {
            if (!num(argv[++i], po.levels.gamma) || po.levels.gamma <= 0.0) return bad("--gamma", argv[i]);
        } else if (a == "--invert") {
            po.levels.invert = true;
        } else if (a == "--no-serpentine") {
            po.dither.serpentine = false;
        } else if (a == "--png" && more) {
            png = argv[++i];
        } else if (a[0] != '-' && !in) {
            in = argv[i];
        } else {
            usage();
            return 2;
        }
    }
    if (!in) {
        usage();
        return 2;
    }
    std::string error;
    GreyImage img;
    if (!load_grey_image(in, img, error)) {
        std::fprintf(stderr, "ncad-laser: %s\n", error.c_str());
        return 1;
    }
    if (w <= 0.0) {  // no --size: the image's own pixels at --dpi
        w = img.w * 25.4 / place.dpi;
        h = img.h * 25.4 / place.dpi;
    }
    place.width = w;
    place.height = h;
    place.image = in;
    RasterPlan rp;
    RasterBitmap bm;
    if (!prepare_raster(place, layer, po, rp, &bm, error)) {
        std::fprintf(stderr, "ncad-laser: %s\n", error.c_str());
        return 1;
    }
    std::printf("image %dx%d px -> %dx%d px at %.0f dpi, %.3f x %.3f mm, dither %s%s\n", img.w, img.h, rp.w, rp.h,
                place.dpi, w, h, dither_name(place.dither), place.bidir ? ", bidirectional" : "");
    std::printf("%zu lines of %d (%zu blank skipped), %zu fired pixels\n", rp.lines.size(), rp.h, rp.blank_rows,
                rp.fired_pixels);
    std::printf("overscan %.3f mm each end (speed %.0f mm/s, accel %.0f mm/s2)\n", rp.overscan_mm, rp.speed_mm_s,
                po.plan.accel_mm_s2);
    std::printf("estimated %.1f s\n", rp.estimated_s);
    if (png) {
        std::vector<std::uint8_t> px(bm.level.size());
        for (std::size_t i = 0; i < px.size(); ++i) px[i] = static_cast<std::uint8_t>(255 - bm.level[i]);
        if (!write_grey_png(png, bm.w, bm.h, px.data(), error)) {
            std::fprintf(stderr, "ncad-laser: %s\n", error.c_str());
            return 1;
        }
    }
    return 0;
}

int encode(int argc, char** argv) {
    const char* in = nullptr;
    const char* out = nullptr;
    const char* origin = nullptr;
    const char* start = nullptr;
    RdMachine machine;
    ncad::laser::ruida::RdPoint pointer;  // --pointer-offset; encode writes the beam's file
    int magic = kDefaultMagic;
    for (int i = 0; i < argc; ++i) {
        if (std::strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            out = argv[++i];
        } else if (std::strcmp(argv[i], "--machine-origin") == 0 && i + 1 < argc) {
            if (!ncad::laser::ruida::parse_machine_corner(argv[++i], machine.origin)) {
                std::fprintf(stderr, "ncad-laser: bad --machine-origin '%s' (bl, br, tl or tr)\n", argv[i]);
                return 2;
            }
        } else if (std::strcmp(argv[i], "--origin") == 0 && i + 1 < argc) {
            origin = argv[++i];
        } else if (std::strcmp(argv[i], "--start") == 0 && i + 1 < argc) {
            start = argv[++i];
            if (std::strcmp(start, "current") != 0 && std::strcmp(start, "absolute") != 0) {
                std::fprintf(stderr, "ncad-laser: bad --start '%s' (current or absolute)\n", start);
                return 2;
            }
        } else if (std::strcmp(argv[i], "--bed") == 0 && i + 1 < argc) {
            if (!parse_bed(argv[++i], machine.bed_w_mm, machine.bed_h_mm)) {
                std::fprintf(stderr, "ncad-laser: bad --bed '%s' (W,H in mm)\n", argv[i]);
                return 2;
            }
        } else if (std::strcmp(argv[i], "--pointer-offset") == 0 && i + 1 < argc) {
            if (!parse_xy(argv[++i], pointer.x, pointer.y)) {
                std::fprintf(stderr, "ncad-laser: bad --pointer-offset '%s' (X,Y in mm)\n", argv[i]);
                return 2;
            }
        } else if (std::strcmp(argv[i], "--magic") == 0 && i + 1 < argc) {
            if (!parse_magic(argv[++i], magic)) {
                std::fprintf(stderr, "ncad-laser: bad magic '%s'\n", argv[i]);
                return 2;
            }
        } else if (argv[i][0] != '-' && !in) {
            in = argv[i];
        } else {
            usage();
            return 2;
        }
    }
    if (!in || !out) {
        usage();
        return 2;
    }
    ncad::laser::LaserJob job;
    std::string error;
    if (!ncad::laser::read_ljob_file(in, job, error)) {
        std::fprintf(stderr, "ncad-laser: %s\n", error.c_str());
        return 1;
    }
    if (start) {
        job.start = std::strcmp(start, "absolute") == 0 ? ncad::laser::StartMode::Absolute
                                                        : ncad::laser::StartMode::Current;
    }
    if (origin) {
        ncad::laser::Pt lo, hi;
        if (!ncad::laser::job_bounds(job, lo, hi)) {
            std::fprintf(stderr, "ncad-laser: the job is empty\n");
            return 1;
        }
        const std::string o = origin;
        double x = 0.0, y = 0.0;
        if (o == "bl") { x = lo.x; y = lo.y; }
        else if (o == "br") { x = hi.x; y = lo.y; }
        else if (o == "tl") { x = lo.x; y = hi.y; }
        else if (o == "tr") { x = hi.x; y = hi.y; }
        else if (std::sscanf(origin, "%lf,%lf", &x, &y) != 2) {
            std::fprintf(stderr, "ncad-laser: bad --origin '%s'\n", origin);
            return 2;
        }
        job.origin = {x, y};
    }
    RdJob rd;
    std::string plain;
    if (!ncad::laser::job_to_rd(job, rd, error)) {
        std::fprintf(stderr, "ncad-laser: %s\n", error.c_str());
        return 1;
    }
    rd.machine = machine;
    rd.pointer_offset = pointer;
    if (!encode_rd(rd, plain, error)) {
        std::fprintf(stderr, "ncad-laser: %s\n", error.c_str());
        return 1;
    }
    if (!write_file(out, Swizzle(static_cast<std::uint8_t>(magic)).encode(plain))) {
        std::fprintf(stderr, "ncad-laser: cannot write '%s'\n", out);
        return 1;
    }
    const RdSummary s = summarize_rd(decode_rd(plain), machine);
    std::printf("%s: %zu bytes, %zu layers, cut %.3f mm, travel %.3f mm, origin %.3f,%.3f (%s), start %s\n",
                out, plain.size(), rd.layers.size(), s.cut_mm, s.travel_mm, job.origin.x, job.origin.y,
                s.origin_corner, ncad::laser::ruida::start_name(s.start));
    return 0;
}

// lbrn2ljob and ljob2lbrn share their frame options.
struct Lbrn2Args {
    const char* in{nullptr};
    const char* out{nullptr};
    ncad::laser::Lbrn2Options opts;
    bool ok{true};
};

Lbrn2Args parse_lbrn2_args(int argc, char** argv, bool reading) {
    Lbrn2Args a;
    for (int i = 0; i < argc && a.ok; ++i) {
        const char* v = i + 1 < argc ? argv[i + 1] : nullptr;
        if (std::strcmp(argv[i], "-o") == 0 && v) {
            a.out = argv[++i];
        } else if (std::strcmp(argv[i], "--machine-origin") == 0 && v) {
            a.ok = ncad::laser::ruida::parse_machine_corner(argv[++i], a.opts.machine.origin);
        } else if (std::strcmp(argv[i], "--bed") == 0 && v) {
            a.ok = parse_bed(argv[++i], a.opts.machine.bed_w_mm, a.opts.machine.bed_h_mm);
        } else if (reading && std::strcmp(argv[i], "--tol") == 0 && v) {
            char* end = nullptr;
            a.opts.tolerance = std::strtod(argv[++i], &end);
            a.ok = end && !*end && a.opts.tolerance > 0.0;
        } else if (reading && std::strcmp(argv[i], "--start") == 0 && v) {
            const std::string s = argv[++i];
            a.ok = s == "current" || s == "absolute";
            a.opts.start = s == "absolute" ? ncad::laser::StartMode::Absolute : ncad::laser::StartMode::Current;
        } else if (reading && std::strcmp(argv[i], "--job-origin") == 0 && v) {
            a.ok = ncad::laser::parse_job_origin(argv[++i], a.opts.job_origin);
        } else if (argv[i][0] != '-' && !a.in) {
            a.in = argv[i];
        } else {
            a.ok = false;
        }
    }
    if (!a.ok || !a.in || !a.out) {
        usage();
        a.ok = false;
    }
    return a;
}

int lbrn2ljob(int argc, char** argv) {
    const Lbrn2Args a = parse_lbrn2_args(argc, argv, true);
    if (!a.ok) return 2;
    ncad::laser::LaserJob job;
    ncad::laser::Lbrn2Info info;
    std::string error;
    if (!ncad::laser::read_lbrn2_file(a.in, job, info, a.opts, error)) {
        std::fprintf(stderr, "ncad-laser: %s\n", error.c_str());
        return 1;
    }
    job.source = a.in;
    if (!ncad::laser::write_ljob_file(a.out, job, error)) {
        std::fprintf(stderr, "ncad-laser: %s\n", error.c_str());
        return 1;
    }
    for (const std::string& w : info.warnings) std::fprintf(stderr, "ncad-laser: warning: %s\n", w.c_str());
    std::printf("%s: %zu layers, %zu paths, origin %.3f,%.3f, start %s (LightBurn %s)\n", a.out, job.layers.size(),
                job.paths.size(), job.origin.x, job.origin.y,
                job.start == ncad::laser::StartMode::Absolute ? "absolute" : "current", info.app_version.c_str());
    return 0;
}

int ljob2lbrn(int argc, char** argv) {
    const Lbrn2Args a = parse_lbrn2_args(argc, argv, false);
    if (!a.ok) return 2;
    ncad::laser::LaserJob job;
    std::string error;
    if (!ncad::laser::read_ljob_file(a.in, job, error) ||
        !ncad::laser::write_lbrn2_file(a.out, job, a.opts, error)) {
        std::fprintf(stderr, "ncad-laser: %s\n", error.c_str());
        return 1;
    }
    std::printf("%s: %zu layers, %zu paths\n", a.out, job.layers.size(), job.paths.size());
    return 0;
}

bool read_file(const char* path, std::string& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    out.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    return true;
}

int rd2txt(int argc, char** argv) {
    const char* path = nullptr;
    int magic = -1;
    bool plain_input = false;
    RdTextOptions opts;
    for (int i = 0; i < argc; ++i) {
        const char* a = argv[i];
        if (std::strcmp(a, "--magic") == 0 && i + 1 < argc) {
            if (!parse_magic(argv[++i], magic)) {
                std::fprintf(stderr, "ncad-laser: bad magic '%s'\n", argv[i]);
                return 2;
            }
        } else if (std::strcmp(a, "--machine-origin") == 0 && i + 1 < argc) {
            if (!ncad::laser::ruida::parse_machine_corner(argv[++i], opts.machine.origin)) {
                std::fprintf(stderr, "ncad-laser: bad --machine-origin '%s' (bl, br, tl or tr)\n", argv[i]);
                return 2;
            }
        } else if (std::strcmp(a, "--plain") == 0) {
            plain_input = true;
        } else if (std::strcmp(a, "--structure") == 0) {
            opts.offsets = false;
            opts.hex = false;
        } else if (std::strcmp(a, "--no-offsets") == 0) {
            opts.offsets = false;
        } else if (std::strcmp(a, "--no-hex") == 0) {
            opts.hex = false;
        } else if (std::strcmp(a, "--no-positions") == 0) {
            opts.positions = false;
        } else if (std::strcmp(a, "--no-summary") == 0) {
            opts.summary = false;
        } else if (a[0] == '-') {
            std::fprintf(stderr, "ncad-laser: unknown option '%s'\n", a);
            return 2;
        } else if (!path) {
            path = a;
        } else {
            usage();
            return 2;
        }
    }
    if (!path) {
        usage();
        return 2;
    }

    std::string data;
    if (!read_file(path, data)) {
        std::fprintf(stderr, "ncad-laser: cannot read '%s'\n", path);
        return 1;
    }

    std::string plain;
    if (plain_input) {
        plain = data;
        std::printf("; %s, %zu bytes, plain\n", path, data.size());
    } else {
        std::uint8_t m = 0;
        const bool guessed = magic < 0;
        if (guessed) {
            if (!guess_magic(data, m)) {
                std::fprintf(stderr, "ncad-laser: '%s' is empty\n", path);
                return 1;
            }
        } else {
            m = static_cast<std::uint8_t>(magic);
        }
        plain = Swizzle(m).decode(data);
        std::printf("; %s, %zu bytes, magic 0x%02X%s\n", path, data.size(), m,
                    guessed ? " (guessed)" : "");
    }
    std::fputs(rd_to_text(plain, opts).c_str(), stdout);
    return 0;
}


#ifdef NLASER_WITH_NET

// Set by Ctrl-C so `watch` can leave its loop and close the port normally.
volatile std::sig_atomic_t g_interrupted = 0;
void on_sigint(int) { g_interrupted = 1; }

// "-5", "-0.25", "-x", "-y": a negative number or a jog direction, which is
// an operand and not an option.
bool negative_operand(const char* a) {
    if (a[1] == '\0') return false;
    if ((a[1] == 'x' || a[1] == 'y' || a[1] == 'X' || a[1] == 'Y') && a[2] == '\0') return true;
    char* end = nullptr;
    std::strtod(a, &end);
    return end && *end == '\0';
}

// Options only some commands use.
struct Extra {
    long interval_ms{500};
    double duration_s{0.0};  // 0 = until interrupted
    double power{-1.0};      // pulse; negative = not given
    long ms{500};
    std::string method{"job"};
    std::string from, to;    // settings: an explicit address range
    bool listen{false};      // watch: send nothing, log what arrives
    bool machine{false};     // goto: X Y are raw machine coordinates
    double step{0.0};        // jog: --fine/--coarse/--step; 0 = not given
    double speed{0.0};       // goto/jog: --speed; 0 = not given
};

// Options shared by every command that talks to a controller; positional
// arguments are collected in `rest`. False on an unknown or malformed option.
bool device_args(int argc, char** argv, ncad::laser::DeviceConfig& cfg, std::vector<const char*>& rest,
                 bool& yes, bool& csv, Extra& x) {
    for (int i = 0; i < argc; ++i) {
        const char* a = argv[i];
        const bool has = i + 1 < argc;
        auto num = [&](long lo, long hi, long& v) {
            char* end = nullptr;
            v = std::strtol(argv[++i], &end, 0);
            return end && *end == '\0' && v >= lo && v <= hi;
        };
        long v = 0;
        if ((std::strcmp(a, "--device") == 0 || std::strcmp(a, "--config") == 0) && has) {
            ++i;  // already applied by device_command, which seeded `cfg` from the file
        } else if (std::strcmp(a, "--host") == 0 && has) {
            cfg.host = argv[++i];
        } else if (std::strcmp(a, "--serial") == 0 && has) {
            cfg.serial = argv[++i];
        } else if (std::strcmp(a, "--baud") == 0 && has) {
            if (!num(300, 4000000, v)) return false;
            cfg.baud = static_cast<int>(v);
        } else if (std::strcmp(a, "--no-flow") == 0) {
            cfg.hw_flow = false;
        } else if (std::strcmp(a, "--magic") == 0 && has) {
            if (!num(0, 255, v)) return false;
            cfg.magic = static_cast<std::uint8_t>(v);
        } else if (std::strcmp(a, "--port") == 0 && has) {
            if (!num(1, 65535, v)) return false;
            cfg.port = static_cast<int>(v);
        } else if (std::strcmp(a, "--local-port") == 0 && has) {
            if (!num(0, 65535, v)) return false;
            cfg.local_port = static_cast<int>(v);
        } else if (std::strcmp(a, "--chunk") == 0 && has) {
            if (!num(16, 65000, v)) return false;
            cfg.max_packet = static_cast<std::size_t>(v);
        } else if (std::strcmp(a, "--timeout") == 0 && has) {
            if (!num(10, 60000, v)) return false;
            cfg.ack_timeout_ms = static_cast<int>(v);
        } else if (std::strcmp(a, "--max-power") == 0 && has) {
            char* end = nullptr;
            cfg.max_power = std::strtod(argv[++i], &end);
            if (!end || *end || !(cfg.max_power >= 0.0 && cfg.max_power <= 100.0)) return false;
        } else if (std::strcmp(a, "--keypad-port") == 0 && has) {
            if (!num(1, 65535, v)) return false;
            cfg.keypad_port = static_cast<int>(v);
        } else if (std::strcmp(a, "--interval") == 0 && has) {
            if (!num(10, 3600000, x.interval_ms)) return false;
        } else if (std::strcmp(a, "--duration") == 0 && has) {
            char* end = nullptr;
            x.duration_s = std::strtod(argv[++i], &end);
            if (!end || *end || !(x.duration_s > 0.0)) return false;
        } else if (std::strcmp(a, "--power") == 0 && has) {
            char* end = nullptr;
            x.power = std::strtod(argv[++i], &end);
            if (!end || *end || !(x.power >= 0.0 && x.power <= 100.0)) return false;
        } else if (std::strcmp(a, "--ms") == 0 && has) {
            if (!num(-1000000, 1000000, x.ms)) return false;
        } else if (std::strcmp(a, "--method") == 0 && has) {
            x.method = argv[++i];
            if (x.method != "job" && x.method != "key") return false;
        } else if (std::strcmp(a, "--from") == 0 && has) {
            x.from = argv[++i];
        } else if (std::strcmp(a, "--to") == 0 && has) {
            x.to = argv[++i];
        } else if (std::strcmp(a, "--listen") == 0) {
            x.listen = true;
        } else if (std::strcmp(a, "--machine") == 0) {
            x.machine = true;
        } else if (std::strcmp(a, "--fine") == 0) {
            x.step = ncad::laser::kJogFineMm;
        } else if (std::strcmp(a, "--coarse") == 0) {
            x.step = ncad::laser::kJogCoarseMm;
        } else if (std::strcmp(a, "--step") == 0 && has) {
            char* end = nullptr;
            x.step = std::strtod(argv[++i], &end);
            if (!end || *end || !(x.step > 0.0) || x.step > 1000.0) return false;
        } else if (std::strcmp(a, "--speed") == 0 && has) {
            char* end = nullptr;
            x.speed = std::strtod(argv[++i], &end);
            if (!end || *end || !(x.speed > 0.0) || x.speed > 2000.0) return false;
        } else if (std::strcmp(a, "--bed") == 0 && has) {
            if (!parse_bed(argv[++i], cfg.bed_w_mm, cfg.bed_h_mm)) return false;
        } else if (std::strcmp(a, "--pointer-offset") == 0 && has) {
            if (!parse_xy(argv[++i], cfg.pointer_offset_x, cfg.pointer_offset_y)) return false;
        } else if (std::strcmp(a, "--yes") == 0) {
            yes = true;
        } else if (std::strcmp(a, "--csv") == 0) {
            csv = true;
        } else if (a[0] == '-' && !negative_operand(a)) {
            return false;
        } else {
            rest.push_back(a);
        }
    }
    return true;
}

// Polls until --duration or Ctrl-C. A poll that gets no reply is printed as
// such and the loop goes on: whether the controller answers while a job runs,
// and how quickly, is exactly what this is for finding out.
int watch(ncad::laser::LaserTransport& t, const std::string& where, const Extra& x, bool csv) {
    using clock = std::chrono::steady_clock;
    struct sigaction sa {};
    sa.sa_handler = on_sigint;  // no SA_RESTART: a blocked read returns early
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, nullptr);
    g_interrupted = 0;

    std::printf("%s", csv ? "t_s,latency_ms,state,x_mm,y_mm,file_total,file_progress,reply\n" : "");
    if (!csv) std::printf("watching %s every %ld ms; Ctrl-C to stop\n", where.c_str(), x.interval_ms);
    const auto start = clock::now();
    unsigned long polls = 0, misses = 0;
    while (!g_interrupted) {
        const auto t0 = clock::now();
        const double at = std::chrono::duration<double>(t0 - start).count();
        if (x.duration_s > 0.0 && at >= x.duration_s) break;
        ncad::laser::MachineStatus s;
        std::string error;
        const bool ok = t.poll(s, error);
        const double lat = std::chrono::duration<double, std::milli>(clock::now() - t0).count();
        if (g_interrupted) break;  // a poll cut short by Ctrl-C is not data
        ++polls;
        if (!ok) ++misses;
        auto num = [](bool has, std::uint64_t v) { return has ? std::to_string(v) : std::string(); };
        if (csv) {
            std::printf("%.3f,%.1f,%s,", at, lat, ok ? num(s.has_state, s.state).c_str() : "");
            if (ok && s.has_position) std::printf("%.3f,%.3f,", s.x_mm, s.y_mm);
            else std::printf(",,");
            std::printf("%s,%s,%s\n", ok ? num(s.has_progress, s.file_total).c_str() : "",
                        ok ? num(s.has_progress, s.file_progress).c_str() : "", ok ? "ok" : "none");
        } else if (!ok) {
            std::printf("%8.3f s  %7.1f ms  no reply (%s)\n", at, lat, error.c_str());
        } else {
            std::printf("%8.3f s  %7.1f ms  state %s", at, lat, s.has_state ? std::to_string(s.state).c_str() : "-");
            if (s.has_position) std::printf("  head %.3f, %.3f mm", s.x_mm, s.y_mm);
            else std::printf("  head -");
            if (s.has_progress) {
                std::printf("  file %llu/%llu", static_cast<unsigned long long>(s.file_progress),
                            static_cast<unsigned long long>(s.file_total));
            }
            std::printf("\n");
        }
        std::fflush(stdout);
        // Sleep in short slices so Ctrl-C and --duration are honoured promptly.
        const auto next = t0 + std::chrono::milliseconds(x.interval_ms);
        while (!g_interrupted && clock::now() < next) {
            const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(next - clock::now());
            std::this_thread::sleep_for(left < std::chrono::milliseconds(50) ? left : std::chrono::milliseconds(50));
        }
    }
    std::fprintf(stderr, "%s: %lu polls, %lu without a reply\n", g_interrupted ? "interrupted" : "done", polls,
                 misses);
    return 0;
}

// Sends nothing after the initial contact and logs every datagram (UDP) or
// burst of bytes (serial) that arrives, timestamped, until --duration or
// Ctrl-C. The question it answers: does the controller push status by itself
// while it cuts?
int listen_command(ncad::laser::ruida::RuidaTransport& t, const std::string& where, bool serial, const Extra& x) {
    using clock = std::chrono::steady_clock;
    struct sigaction sa {};
    sa.sa_handler = on_sigint;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, nullptr);
    g_interrupted = 0;
    std::fprintf(stderr, "listening on %s (%s); nothing is sent after the initial contact; Ctrl-C to stop\n",
                 where.c_str(), serial ? "serial: no contact at all" : "UDP: one ENQ handshake");
    const auto start = clock::now();
    unsigned long heard = 0;
    while (!g_interrupted) {
        const double at = std::chrono::duration<double>(clock::now() - start).count();
        if (x.duration_s > 0.0 && at >= x.duration_s) break;
        std::string raw, plain, error;
        const int r = t.listen_next(raw, plain, 100, error);
        if (r < 0) {
            if (g_interrupted) break;
            std::fprintf(stderr, "ncad-laser: %s\n", error.c_str());
            return 1;
        }
        if (r == 0) continue;
        const std::time_t now = std::time(nullptr);
        char when[16];
        std::strftime(when, sizeof when, "%H:%M:%S", std::localtime(&now));
        const double t_s = std::chrono::duration<double>(clock::now() - start).count();
        ++heard;
        std::string hex;
        for (char ch : plain) {
            char b[4];
            std::snprintf(b, sizeof b, "%02X ", static_cast<unsigned>(static_cast<unsigned char>(ch)));
            hex += b;
        }
        if (!hex.empty()) hex.pop_back();
        const std::string d = ncad::laser::ruida::describe_wire(plain);
        std::printf("%s +%8.3f s  %s %zu bytes  %s%s%s\n", when, t_s, serial ? "burst   " : "datagram", plain.size(),
                    d.c_str(), d.compare(0, 3, "raw") == 0 ? "" : "  | ", d.compare(0, 3, "raw") == 0 ? "" : hex.c_str());
        std::fflush(stdout);
    }
    std::fprintf(stderr, "%s: %lu bursts heard\n", g_interrupted ? "interrupted" : "done", heard);
    return 0;
}

// Read-only dump of controller memory. Every address MeerK40t's emulator names
// by default; --from/--to read EVERY address in a range instead. A missing
// reply is marked and the dump goes on. Entries whose names mention red,
// light, pointer or offset are flagged with '*': the red-dot pointer offset is
// what this is looking for.
int settings_command(ncad::laser::ruida::RuidaTransport& t, const Extra& x, bool csv) {
    using namespace ncad::laser::ruida;
    std::vector<std::uint16_t> addrs;
    if (!x.from.empty() || !x.to.empty()) {
        std::uint16_t lo = 0, hi = 0x3FFF;
        if ((!x.from.empty() && !parse_memory_address(x.from, lo)) || (!x.to.empty() && !parse_memory_address(x.to, hi))) {
            std::fprintf(stderr, "ncad-laser: bad address (0xNNNN, or the wire bytes HH:LL; at most 0x3FFF)\n");
            return 2;
        }
        if (x.to.empty() && !x.from.empty()) hi = lo;  // --from alone: that one address
        if (hi < lo) {
            std::fprintf(stderr, "ncad-laser: --to is below --from\n");
            return 2;
        }
        for (unsigned a = lo; a <= hi; ++a) addrs.push_back(static_cast<std::uint16_t>(a));
    } else {
        addrs = named_addresses();
    }
    struct sigaction sa {};
    sa.sa_handler = on_sigint;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, nullptr);
    g_interrupted = 0;
    std::fprintf(stderr, "reading %zu addresses (read-only; Ctrl-C to stop); '*' marks red/light/pointer/offset\n",
                 addrs.size());
    std::vector<SettingRow> rows;
    std::string error;
    read_settings(t, addrs, rows, [] { return g_interrupted != 0; },
                  [](std::size_t done, std::size_t total) {
                      if (done % 50 == 0 || done == total) std::fprintf(stderr, "\r%zu/%zu", done, total);
                  },
                  error);
    std::fputc('\n', stderr);
    if (csv) std::printf("address,wire,name,flag,replied,number,raw_hex,text\n");
    std::size_t replied = 0, flagged = 0;
    for (const SettingRow& r : rows) {
        std::string hex;
        for (char ch : r.raw) {
            char b[4];
            std::snprintf(b, sizeof b, "%02X ", static_cast<unsigned>(static_cast<unsigned char>(ch)));
            hex += b;
        }
        if (!hex.empty()) hex.pop_back();
        if (r.replied) ++replied;
        if (r.interesting) ++flagged;
        const std::string addr = format_memory_address(r.addr);
        const std::string name = r.name.empty() ? "(not named by MeerK40t)" : r.name;
        if (csv) {
            std::printf("0x%04X,%02X %02X,\"%s\",%s,%s,%s,%s,\"%s\"\n", static_cast<unsigned>(r.addr),
                        static_cast<unsigned>((r.addr >> 7) & 0x7Fu), static_cast<unsigned>(r.addr & 0x7Fu),
                        name.c_str(), r.interesting ? "*" : "", r.replied ? "yes" : "no",
                        r.has_number ? std::to_string(r.number).c_str() : "", hex.c_str(), r.text.c_str());
            continue;
        }
        std::printf("%c %s  %-44s ", r.interesting ? '*' : ' ', addr.c_str(), name.c_str());
        if (!r.replied) {
            std::printf("(no reply)\n");
            continue;
        }
        if (r.has_number) {
            std::printf("= %llu (0x%llX)", static_cast<unsigned long long>(r.number),
                        static_cast<unsigned long long>(r.number));
        }
        if (!r.text.empty()) std::printf("  \"%s\"", r.text.c_str());
        if (r.raw.size() != 5) std::printf("  [%s]", hex.c_str());
        std::printf("\n");
    }
    std::fprintf(stderr, "%zu read, %zu replied, %zu without a reply, %zu flagged\n", rows.size(), replied,
                 rows.size() - replied, flagged);
    return 0;
}

int device_command(const char* name, int argc, char** argv) {
    using namespace ncad::laser;
    DeviceConfig cfg;
    std::vector<const char*> rest;
    bool yes = false, csv = false;
    Extra extra;
    // Seed from the file first so explicit flags, applied after, win. With an
    // explicit --host or --serial and no --device the file is not read at all,
    // so a broken config cannot block a one-off address.
    const std::string device = option_value(argc, argv, "--device");
    const bool cli_host = !option_value(argc, argv, "--host").empty();
    const bool cli_serial = !option_value(argc, argv, "--serial").empty();
    if (!device.empty() || (!cli_host && !cli_serial)) {
        std::vector<DeviceConfig> devices;
        std::string cfg_error;
        if (!load_devices(option_value(argc, argv, "--config"), devices, cfg_error) ||
            !select_device(devices, device, cfg, cfg_error)) {
            std::fprintf(stderr, "ncad-laser: %s\n", cfg_error.c_str());
            return 1;
        }
    }
    if (!device_args(argc, argv, cfg, rest, yes, csv, extra)) {
        usage();
        return 2;
    }
    // A dump makes hundreds of reads, any of which may go unanswered: a
    // shorter wait than the 2 s a job upload gets, unless --timeout says.
    if (std::strcmp(name, "settings") == 0 && option_value(argc, argv, "--timeout").empty()) {
        cfg.ack_timeout_ms = 500;
    }
    // An explicit --serial replaces a host that came from the file, and an
    // explicit --host a file's serial device; giving both is still an error.
    if (cli_serial && !cli_host) cfg.host.clear();
    if (cli_host && !cli_serial) cfg.serial.clear();
    if (cfg.host.empty() == cfg.serial.empty()) {
        usage();  // neither or both of --host and --serial
        return 2;
    }
    const bool serial = !cfg.serial.empty();
    const std::string& where = serial ? cfg.serial : cfg.host;
    const bool is_pulse = std::strcmp(name, "pulse") == 0;
    const bool needs_job = std::strcmp(name, "frame") == 0 || std::strcmp(name, "send") == 0 ||
                           std::strcmp(name, "dryrun") == 0;
    const bool is_goto = std::strcmp(name, "goto") == 0;
    const bool is_jog = std::strcmp(name, "jog") == 0;
    const bool operand_ok = is_goto ? rest.size() == 2 : is_jog ? (rest.size() == 1 || rest.size() == 2)
                                                                 : rest.size() == (needs_job ? 1u : 0u);
    if (!operand_ok) {
        usage();
        return 2;
    }

    std::string error;
    LaserJob job;
    std::string plain;
    // What can be refused without a controller is refused first. An absolute
    // job on a machine that counts from its far edge needs the bed size, which
    // may come from the controller, so that one is prepared after connecting.
    auto prepare = [&](const DeviceConfig& c) {
        return std::strcmp(name, "send") == 0     ? prepare_send(job, c, plain, error)
               : std::strcmp(name, "dryrun") == 0 ? prepare_dryrun(job, c, plain, error)
                                                  : prepare_frame(job, c, plain, error);
    };
    bool prepared = false;
    if (needs_job) {
        if (!read_ljob_file(rest[0], job, error)) {
            std::fprintf(stderr, "ncad-laser: %s\n", error.c_str());
            return 1;
        }
        const bool needs_bed = job.start == StartMode::Absolute && machine_of(cfg).mirrored() &&
                               !(cfg.bed_w_mm > 0.0 && cfg.bed_h_mm > 0.0);
        if (!needs_bed) {
            if (!prepare(cfg)) {
                std::fprintf(stderr, "ncad-laser: %s\n", error.c_str());
                return 1;
            }
            prepared = true;
        }
    }

    double pulse_len = 0.0;
    if (is_pulse) {
        const bool key = extra.method == "key";
        const int ms = static_cast<int>(extra.ms);
        const std::string what = key ? "key" : "job";
        if (key) {
            if (serial) {
                std::fprintf(stderr, "ncad-laser: pulse --method key needs a UDP controller (--host): the keypad "
                                     "channel is not available over USB serial. Use --method job.\n");
                return 1;
            }
            if (!key_pulse_allowed(cfg, error)) {
                std::fprintf(stderr, "ncad-laser: %s\n", error.c_str());
                return 1;
            }
            if (ms < kPulseMinMs || ms > kPulseMaxMs) {
                std::fprintf(stderr, "ncad-laser: pulse duration must be %d..%d ms\n", kPulseMinMs, kPulseMaxMs);
                return 1;
            }
        } else {
            if (extra.power < 0.0) {
                std::fprintf(stderr, "ncad-laser: pulse --method job needs --power PCT\n");
                return 1;
            }
            if (!prepare_pulse(extra.power, ms, cfg, plain, pulse_len, error)) {
                std::fprintf(stderr, "ncad-laser: %s\n", error.c_str());
                return 1;
            }
        }
        std::printf("pulse    FIRES THE LASER on %s, method %s\n", where.c_str(), what.c_str());
        if (key) {
            std::printf("power    the panel's own manual power setting (NOT --power%s); not checked here\n",
                        extra.power >= 0.0 ? ", which is ignored" : "");
            std::printf("hold     Pulse key down for %d ms, then up; the head does not move\n", ms);
        } else {
            std::printf("power    %g%% (operator max %g%%, hard limit %g%%)\n", extra.power, cfg.max_power,
                        cfg.power_limit);
            std::printf("hold     one cut at %g mm/s for %d ms: the head moves exactly %.3f mm along design +X "
                        "from where it is now (origin_corner/flip_x may mirror this on the machine)\n",
                        kPulseSpeedMmS, ms, pulse_len);
        }
        if (!yes) {
            std::printf("Type 'yes' to FIRE the laser on %s now: ", where.c_str());
            std::fflush(stdout);
            char answer[16] = {};
            if (!std::fgets(answer, sizeof answer, stdin) || std::strcmp(answer, "yes\n") != 0) {
                std::printf("not fired\n");
                return 1;
            }
        }
        if (key) {
            if (!pulse_key(cfg, ms, error)) {
                std::fprintf(stderr, "ncad-laser: %s\n", error.c_str());
                return 1;
            }
            std::printf("keypad: Pulse down, %d ms, up sent to %s:%d (unacknowledged)\n", ms, cfg.host.c_str(),
                        cfg.keypad_port);
            return 0;
        }
    }

    // One port is opened; the other stays an unopened shell. Both framings
    // reach the same transport, which is the point of the two port types.
    UdpPort udp;
    SerialPort tty;
    const bool opened = serial ? tty.open(cfg.serial, cfg.baud, cfg.hw_flow, error)
                               : udp.open(cfg.host, cfg.port, cfg.local_port, error);
    if (!opened) {
        std::fprintf(stderr, "ncad-laser: %s\n", error.c_str());
        return 1;
    }
    RuidaTransport t = serial ? RuidaTransport(tty, cfg) : RuidaTransport(udp, cfg);

    if (std::strcmp(name, "stop") == 0) {
        // Stop does not wait on the handshake: if ENQ fails, try anyway.
        std::string ignore;
        t.open(ignore);
        if (!t.stop(error)) {
            std::fprintf(stderr, "ncad-laser: stop: %s\n", error.c_str());
            return 1;
        }
        std::printf("stop sent to %s%s\n", where.c_str(), serial ? " (not acknowledged over serial)" : "");
        return 0;
    }

    if (std::strcmp(name, "watch") == 0 && extra.listen) {
        if (!t.listen_begin(error)) {
            std::fprintf(stderr, "ncad-laser: %s: %s\n", where.c_str(), error.c_str());
            return 1;
        }
        return listen_command(t, where, serial, extra);
    }
    if (!t.open(error)) {
        std::fprintf(stderr, "ncad-laser: %s: %s\n", where.c_str(), error.c_str());
        return 1;
    }

    if (needs_job) {
        // The same check Send/Frame/Dryrun make themselves, shown first: where
        // the job lands, or why that could not be checked (policy: a link that
        // cannot report is warned about, not refused).
        DeviceConfig eff;
        PlacementReport rep;
        if (!t.check_placement(job, eff, rep, error)) {
            std::fprintf(stderr, "ncad-laser: %s\n", error.c_str());
            return 1;
        }
        if (!rep.note.empty()) std::printf("%s\n", rep.note.c_str());
        if (!prepared && !prepare(eff)) {
            std::fprintf(stderr, "ncad-laser: %s\n", error.c_str());
            return 1;
        }
        cfg = eff;
    }

    if (std::strcmp(name, "send") == 0) {
        const RdSummary s = summarize_rd(decode_rd(plain), machine_of(cfg));
        std::printf("job      %s\n", job.name.empty() ? rest[0] : job.name.c_str());
        for (const LaserLayer& l : job.layers) {
            std::printf("layer %-3d speed %g mm/s, power %g-%g%%, passes %d, air %s\n", l.pen, l.speed,
                        l.min_power, l.max_power, l.passes, l.air ? "on" : "off");
        }
        std::printf("bounds   %.3f,%.3f .. %.3f,%.3f mm\n", s.cut_min_x, s.cut_min_y, s.cut_max_x, s.cut_max_y);
        std::printf("origin   %.3f,%.3f (%s), %s\n", job.origin.x,
                    job.origin.y, s.origin_corner,
                    job.start == StartMode::Absolute ? "absolute: design coordinates are bed coordinates"
                                                     : "placed at the head's current position");
        std::printf("cut      %.1f mm, travel %.1f mm\n", s.cut_mm, s.travel_mm);
        if (!yes) {
            std::printf("Type 'yes' to run this job on %s now: ", where.c_str());
            std::fflush(stdout);
            char answer[16] = {};
            if (!std::fgets(answer, sizeof answer, stdin) || std::strcmp(answer, "yes\n") != 0) {
                std::printf("not sent\n");
                return 1;
            }
        }
    }

    if (std::strcmp(name, "settings") == 0) return settings_command(t, extra, csv);

    if (is_goto || is_jog) {
        MoveFrame frame = MoveFrame::Bed;
        double mx = 0.0, my = 0.0;
        if (is_goto) {
            char* e1 = nullptr;
            char* e2 = nullptr;
            mx = std::strtod(rest[0], &e1);
            my = std::strtod(rest[1], &e2);
            if (*e1 || *e2 || e1 == rest[0] || e2 == rest[1]) {
                std::fprintf(stderr, "ncad-laser: goto needs two numbers, X Y in mm\n");
                return 2;
            }
            frame = extra.machine ? MoveFrame::Machine : MoveFrame::Bed;
        } else {
            std::vector<std::string> args(rest.begin(), rest.end());
            if (!parse_jog(args, extra.step, mx, my, error)) {
                std::fprintf(stderr, "ncad-laser: %s\n", error.c_str());
                return 2;
            }
            frame = MoveFrame::Offset;
        }
        MoveReport rep;
        if (!t.move_head(frame, mx, my, extra.speed, rep, error)) {
            std::fprintf(stderr, "ncad-laser: %s\n", error.c_str());
            return 1;
        }
        std::printf("head -> machine %.3f, %.3f mm", rep.machine_x, rep.machine_y);
        if (rep.has_bed_pos) std::printf("  (bed %.3f, %.3f mm)", rep.bed_x, rep.bed_y);
        std::printf("%s\n", serial ? "  (serial: not acknowledged by the controller)" : "");
        if (!rep.note.empty()) std::printf("note: %s\n", rep.note.c_str());
        if (extra.speed > 0.0) {
            std::printf("note: --speed (C9 02, as MeerK40t does) is UNVERIFIED on this controller\n");
        }
        return 0;
    }

    if (std::strcmp(name, "counters") == 0) {
        Counters k;
        if (!t.counters(k, error)) {
            std::fprintf(stderr, "ncad-laser: %s\n", error.c_str());
            return 1;
        }
        auto field = [](bool has, std::uint64_t v) { return has ? std::to_string(v) : std::string(); };
        if (csv) {
            char when[32];
            const std::time_t now = std::time(nullptr);
            std::strftime(when, sizeof when, "%Y-%m-%dT%H:%M:%S", std::localtime(&now));
            std::printf("%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n", when, where.c_str(),
                        field(k.has_card_id, k.card_id).c_str(), k.firmware.c_str(),
                        field(k.has_power_on, k.power_on).c_str(), field(k.has_work_time, k.work_time).c_str(),
                        field(k.has_laser_on, k.laser_on).c_str(), field(k.has_jobs, k.jobs).c_str(),
                        field(k.has_travel_x, k.travel_x).c_str(), field(k.has_travel_y, k.travel_y).c_str());
            return 0;
        }
        if (k.has_card_id) std::printf("card id          0x%08llX\n", static_cast<unsigned long long>(k.card_id));
        if (k.has_firmware) std::printf("firmware         %s\n", k.firmware.c_str());
        std::printf("power-on time    %s\n", field(k.has_power_on, k.power_on).c_str());
        std::printf("work time        %s\n", field(k.has_work_time, k.work_time).c_str());
        std::printf("laser-on time    %s\n", field(k.has_laser_on, k.laser_on).c_str());
        std::printf("jobs             %s\n", field(k.has_jobs, k.jobs).c_str());
        std::printf("travel x         %s\n", field(k.has_travel_x, k.travel_x).c_str());
        std::printf("travel y         %s\n", field(k.has_travel_y, k.travel_y).c_str());
        std::printf("(raw values; units to be confirmed against the panel)\n");
        return 0;
    }

    if (std::strcmp(name, "status") == 0) {
        MachineStatus s;
        if (!t.status(s, error)) {
            std::fprintf(stderr, "ncad-laser: %s\n", error.c_str());
            return 1;
        }
        if (s.has_state) std::printf("state     %llu (0x%llX, raw)\n", static_cast<unsigned long long>(s.state),
                                     static_cast<unsigned long long>(s.state));
        if (s.has_position) std::printf("position  %.3f, %.3f mm\n", s.x_mm, s.y_mm);
        if (s.has_bed) std::printf("bed       %.3f x %.3f mm\n", s.bed_w_mm, s.bed_h_mm);
        if (s.has_progress) std::printf("file      %llu of %llu\n", static_cast<unsigned long long>(s.file_progress),
                                        static_cast<unsigned long long>(s.file_total));
        return 0;
    }

    if (std::strcmp(name, "watch") == 0) return watch(t, where, extra, csv);

    if (is_pulse) {
        if (!t.pulse(extra.power, static_cast<int>(extra.ms), error)) {
            std::fprintf(stderr, "ncad-laser: %s\n", error.c_str());
            return 1;
        }
        std::printf("pulse: %zu bytes sent; the head moves %.3f mm%s\n", plain.size(), pulse_len,
                    serial ? " (serial: not acknowledged by the controller)" : "");
        return 0;
    }

    const bool dry = std::strcmp(name, "dryrun") == 0;
    const bool sending = std::strcmp(name, "send") == 0;
    const bool ok = sending ? t.send(job, [](std::size_t done, std::size_t total) {
        std::fprintf(stderr, "\ruploading %3zu%%", total ? done * 100 / total : 100);
    }, error) : (dry ? t.dryrun(job, error) : t.frame(job, error));
    if (sending) std::fputc('\n', stderr);
    if (!ok) {
        std::fprintf(stderr, "ncad-laser: %s\n", error.c_str());
        return 1;
    }
    if (serial) {
        // No ACKs to count: say so, so nobody reads silence as confirmation.
        std::printf("%s: %zu bytes in %zu writes (serial: not acknowledged by the controller)\n",
                    sending ? "sent" : dry ? "dry run" : "framing", plain.size(), t.stats().packets);
        return 0;
    }
    std::printf("%s: %zu bytes in %zu packets (%zu NAKs)\n", sending ? "sent" : dry ? "dry run" : "framing",
                plain.size(), t.stats().packets, t.stats().naks);
    return 0;
}

#endif  // NLASER_WITH_NET

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        usage();
        return 2;
    }
    if (std::strcmp(argv[1], "dxf2ljob") == 0) return dxf2ljob(argc - 2, argv + 2);
    if (std::strcmp(argv[1], "svg2ljob") == 0) return svg2ljob(argc - 2, argv + 2);
    if (std::strcmp(argv[1], "pdf2ljob") == 0) return pdf2ljob(argc - 2, argv + 2, false);
    if (std::strcmp(argv[1], "pdfstream2ljob") == 0) return pdf2ljob(argc - 2, argv + 2, true);
    if (std::strcmp(argv[1], "plan") == 0) return plan(argc - 2, argv + 2);
    if (std::strcmp(argv[1], "rasterplan") == 0) return rasterplan(argc - 2, argv + 2);
    if (std::strcmp(argv[1], "encode") == 0) return encode(argc - 2, argv + 2);
    if (std::strcmp(argv[1], "lbrn2ljob") == 0) return lbrn2ljob(argc - 2, argv + 2);
    if (std::strcmp(argv[1], "ljob2lbrn") == 0) return ljob2lbrn(argc - 2, argv + 2);
    if (std::strcmp(argv[1], "rd2txt") == 0) return rd2txt(argc - 2, argv + 2);
    if (std::strcmp(argv[1], "devices") == 0) return devices_command(argc - 2, argv + 2);
    for (const char* dev : {"counters", "status", "watch", "stop", "frame", "dryrun", "pulse", "send", "goto", "jog", "settings"}) {
        if (std::strcmp(argv[1], dev) != 0) continue;
#ifdef NLASER_WITH_NET
        return device_command(dev, argc - 2, argv + 2);
#else
        std::fprintf(stderr, "ncad-laser: built without NLASER_WITH_NET; '%s' needs the network\n", dev);
        return 2;
#endif
    }
    std::fprintf(stderr, "ncad-laser: unknown command '%s'\n", argv[1]);
    usage();
    return 2;
}
