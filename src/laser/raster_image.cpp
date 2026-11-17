// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// The only translation unit that includes stb_image / stb_image_write
// (third_party/stb, public domain or MIT). Warnings from vendored code are
// switched off for this file alone in CMake; the rest of Nlaser stays strict.

#include "ncad/laser/raster.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STBI_NO_LINEAR
#define STBI_NO_HDR
#include "stb/stb_image.h"
#include "stb/stb_image_write.h"

namespace ncad::laser {

bool decode_grey_image(const std::uint8_t* data, std::size_t size, GreyImage& out, std::string& error) {
    if (size == 0 || size > 0x7fffffffu) {
        error = "empty or oversized image";
        return false;
    }
    int w = 0, h = 0, comp = 0;
    stbi_uc* rgba = stbi_load_from_memory(data, static_cast<int>(size), &w, &h, &comp, 4);
    if (!rgba) {
        const char* why = stbi_failure_reason();
        error = std::string("cannot decode image: ") + (why ? why : "unknown format");
        return false;
    }
    GreyImage img;
    img.w = w;
    img.h = h;
    img.px.resize(static_cast<std::size_t>(w) * static_cast<std::size_t>(h));
    for (std::size_t i = 0; i < img.px.size(); ++i) {
        const stbi_uc* p = rgba + 4 * i;
        const double a = p[3] / 255.0;
        const double lum = 0.299 * p[0] + 0.587 * p[1] + 0.114 * p[2];
        const double v = lum * a + 255.0 * (1.0 - a);
        img.px[i] = static_cast<std::uint8_t>(std::lround(v));
    }
    stbi_image_free(rgba);
    out = std::move(img);
    return true;
}

bool load_grey_image(const std::string& path, GreyImage& out, std::string& error) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        error = "cannot read '" + path + "'";
        return false;
    }
    const std::string bytes{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    if (!decode_grey_image(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size(), out, error)) {
        error = path + ": " + error;
        return false;
    }
    return true;
}

bool write_grey_png(const std::string& path, int w, int h, const std::uint8_t* px, std::string& error) {
    if (!stbi_write_png(path.c_str(), w, h, 1, px, w)) {
        error = "cannot write '" + path + "'";
        return false;
    }
    return true;
}

}  // namespace ncad::laser
