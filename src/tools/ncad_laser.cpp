// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// ncad-laser: the file-driven front end: plan, encode, rd2txt.

#include "ncad/laser/job.hpp"
#include "ncad/laser/plan.hpp"
#include "ncad/laser/ruida_codec.hpp"
#include "ncad/laser/ruida_decode.hpp"
#include "ncad/laser/ruida_encode.hpp"
#include "ncad/laser/ruida_job.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>

namespace {

using namespace ncad::laser::ruida;

void usage() {
    std::fputs(
        "usage: ncad-laser plan   IN.ljob -o OUT.ljob [--join-tol MM]\n"
        "       ncad-laser encode PLANNED.ljob -o OUT.rd [--origin WHERE] [--magic N]\n"
        "       ncad-laser rd2txt FILE.rd [options]\n"
        "\n"
        "encode:\n"
        "  --origin WHERE  override the job's origin: x,y in job mm, or a corner of\n"
        "                  the job's box: bl, br, tl, tr (LightBurn's anchoring)\n"
        "  --magic N       swizzle magic (default 0x88)\n"
        "rd2txt:\n"
        "  --magic N       unswizzle with magic N (e.g. 0x88); default: guess\n"
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

bool write_file(const char* path, const std::string& data) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) return false;
    out << data;
    return static_cast<bool>(out);
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
    return 0;
}

int encode(int argc, char** argv) {
    const char* in = nullptr;
    const char* out = nullptr;
    const char* origin = nullptr;
    int magic = kDefaultMagic;
    for (int i = 0; i < argc; ++i) {
        if (std::strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            out = argv[++i];
        } else if (std::strcmp(argv[i], "--origin") == 0 && i + 1 < argc) {
            origin = argv[++i];
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
    if (!ncad::laser::job_to_rd(job, rd, error) || !encode_rd(rd, plain, error)) {
        std::fprintf(stderr, "ncad-laser: %s\n", error.c_str());
        return 1;
    }
    if (!write_file(out, Swizzle(static_cast<std::uint8_t>(magic)).encode(plain))) {
        std::fprintf(stderr, "ncad-laser: cannot write '%s'\n", out);
        return 1;
    }
    const RdSummary s = summarize_rd(decode_rd(plain));
    std::printf("%s: %zu bytes, %zu layers, cut %.3f mm, travel %.3f mm, origin %.3f,%.3f (%s)\n", out,
                plain.size(), rd.layers.size(), s.cut_mm, s.travel_mm, job.origin.x, job.origin.y,
                s.origin_corner);
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

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        usage();
        return 2;
    }
    if (std::strcmp(argv[1], "plan") == 0) return plan(argc - 2, argv + 2);
    if (std::strcmp(argv[1], "encode") == 0) return encode(argc - 2, argv + 2);
    if (std::strcmp(argv[1], "rd2txt") == 0) return rd2txt(argc - 2, argv + 2);
    std::fprintf(stderr, "ncad-laser: unknown command '%s'\n", argv[1]);
    usage();
    return 2;
}
