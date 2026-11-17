// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

#include "ncad/laser/xml.hpp"

#include <cctype>
#include <cstdint>

namespace ncad::laser::xml {

const std::string* Node::attr(std::string_view key) const {
    for (const auto& a : attrs) {
        if (a.first == key) return &a.second;
    }
    return nullptr;
}

const Node* Node::child(std::string_view tag) const {
    for (const Node& c : children) {
        if (c.name == tag) return &c;
    }
    return nullptr;
}

std::string escape(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (char c : s) {
        switch (c) {
            case '&': out += "&amp;"; break;
            case '<': out += "&lt;"; break;
            case '>': out += "&gt;"; break;
            case '"': out += "&quot;"; break;
            case '\'': out += "&apos;"; break;
            default: out.push_back(c);
        }
    }
    return out;
}

namespace {

constexpr int kMaxDepth = 200;

struct Parser {
    const std::string& s;
    std::size_t i{0};
    int line{1};
    std::string error;

    explicit Parser(const std::string& text) : s(text) {}

    bool fail(const std::string& what) {
        if (error.empty()) error = "line " + std::to_string(line) + ": " + what;
        return false;
    }
    bool eof() const { return i >= s.size(); }
    char peek() const { return i < s.size() ? s[i] : '\0'; }
    bool starts(const char* lit) const { return s.compare(i, std::char_traits<char>::length(lit), lit) == 0; }
    void adv() {
        if (s[i] == '\n') ++line;
        ++i;
    }
    void skip_ws() {
        while (!eof() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\r' || s[i] == '\n')) adv();
    }

    static bool name_start(char c) { return std::isalpha(static_cast<unsigned char>(c)) || c == '_' || c == ':'; }
    static bool name_char(char c) {
        return name_start(c) || std::isdigit(static_cast<unsigned char>(c)) || c == '-' || c == '.';
    }

    bool read_name(std::string& out) {
        if (eof() || !name_start(peek())) return fail("a name was expected");
        const std::size_t b = i;
        while (!eof() && name_char(peek())) ++i;
        out.assign(s, b, i - b);
        return true;
    }

    static void put_utf8(std::string& out, std::uint32_t cp) {
        if (cp < 0x80) {
            out.push_back(static_cast<char>(cp));
        } else if (cp < 0x800) {
            out.push_back(static_cast<char>(0xC0 | (cp >> 6)));
            out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
        } else if (cp < 0x10000) {
            out.push_back(static_cast<char>(0xE0 | (cp >> 12)));
            out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
            out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
        } else {
            out.push_back(static_cast<char>(0xF0 | (cp >> 18)));
            out.push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3F)));
            out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
            out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
        }
    }

    // At '&'; appends the decoded character and leaves `i` on the ';'.
    bool read_entity(std::string& out) {
        const std::size_t semi = s.find(';', i);
        if (semi == std::string::npos || semi - i > 12) return fail("an '&' with no ';' (unknown entity)");
        const std::string body = s.substr(i + 1, semi - i - 1);
        if (body == "amp") {
            out.push_back('&');
        } else if (body == "lt") {
            out.push_back('<');
        } else if (body == "gt") {
            out.push_back('>');
        } else if (body == "quot") {
            out.push_back('"');
        } else if (body == "apos") {
            out.push_back('\'');
        } else if (body.size() >= 2 && body[0] == '#') {
            const bool hex = body[1] == 'x';
            const std::size_t from = hex ? 2 : 1;
            if (body.size() == from) return fail("an empty character reference");
            std::uint32_t cp = 0;
            for (std::size_t k = from; k < body.size(); ++k) {
                const char c = body[k];
                int d = -1;
                if (c >= '0' && c <= '9') d = c - '0';
                else if (hex && c >= 'a' && c <= 'f') d = c - 'a' + 10;
                else if (hex && c >= 'A' && c <= 'F') d = c - 'A' + 10;
                if (d < 0) return fail("a bad character reference '&" + body + ";'");
                cp = cp * (hex ? 16u : 10u) + static_cast<std::uint32_t>(d);
                if (cp > 0x10FFFF) return fail("a character reference beyond U+10FFFF");
            }
            if (cp == 0) return fail("a character reference to U+0000");
            put_utf8(out, cp);
        } else {
            return fail("unknown entity '&" + body + ";'");
        }
        i = semi;
        return true;
    }

    bool read_attr_value(std::string& out) {
        const char q = peek();
        if (q != '"' && q != '\'') return fail("an attribute value must be quoted");
        adv();
        while (!eof() && peek() != q) {
            const char c = peek();
            if (c == '<') return fail("'<' inside an attribute value");
            if (c == '&') {
                if (!read_entity(out)) return false;
                ++i;
            } else {
                out.push_back(c);
                adv();
            }
        }
        if (eof()) return fail("an attribute value is not closed");
        adv();
        return true;
    }

    // Comments only; every other '<!' is refused.
    bool skip_comment() {
        const int at = line;
        i += 4;
        while (!eof() && !starts("-->")) adv();
        if (eof()) {
            line = at;
            return fail("a comment is not closed");
        }
        i += 3;
        return true;
    }

    bool read_element(Node& n, int depth) {
        if (depth > kMaxDepth) return fail("elements nested more than 200 deep");
        n.line = line;
        adv();  // '<'
        if (!read_name(n.name)) return false;
        for (;;) {
            skip_ws();
            if (eof()) return fail("a tag is not closed");
            if (peek() == '/') {
                adv();
                if (peek() != '>') return fail("'/' must be followed by '>'");
                adv();
                return true;
            }
            if (peek() == '>') {
                adv();
                break;
            }
            std::string key, val;
            if (!read_name(key)) return false;
            skip_ws();
            if (peek() != '=') return fail("attribute '" + key + "' has no '='");
            adv();
            skip_ws();
            if (!read_attr_value(val)) return false;
            if (n.attr(key)) return fail("attribute '" + key + "' given twice");
            n.attrs.emplace_back(std::move(key), std::move(val));
        }
        for (;;) {
            if (eof()) return fail("<" + n.name + "> (line " + std::to_string(n.line) + ") is not closed");
            if (peek() != '<') {
                if (peek() == '&') {
                    if (!read_entity(n.text)) return false;
                    ++i;
                } else {
                    n.text.push_back(peek());
                    adv();
                }
                continue;
            }
            if (starts("</")) {
                i += 2;
                std::string close;
                if (!read_name(close)) return false;
                skip_ws();
                if (peek() != '>') return fail("a closing tag must end with '>'");
                adv();
                if (close != n.name) {
                    return fail("</" + close + "> closes <" + n.name + "> (opened on line " +
                                std::to_string(n.line) + ")");
                }
                break;
            }
            if (starts("<!--")) {
                if (!skip_comment()) return false;
                continue;
            }
            if (starts("<![CDATA[")) return fail("CDATA is not supported");
            if (starts("<!")) return fail("'<!' declarations are not supported");
            if (starts("<?")) return fail("processing instructions are not supported");
            n.children.emplace_back();
            if (!read_element(n.children.back(), depth + 1)) return false;
        }
        std::size_t b = 0, e = n.text.size();
        while (b < e && std::isspace(static_cast<unsigned char>(n.text[b]))) ++b;
        while (e > b && std::isspace(static_cast<unsigned char>(n.text[e - 1]))) --e;
        n.text = n.text.substr(b, e - b);
        return true;
    }

    bool run(Node& root) {
        if (starts("\xEF\xBB\xBF")) i = 3;
        if (starts("<?xml")) {
            while (!eof() && !starts("?>")) adv();
            if (eof()) return fail("the XML declaration is not closed");
            i += 2;
        }
        for (;;) {
            skip_ws();
            if (!starts("<!--")) break;
            if (!skip_comment()) return false;
        }
        if (eof()) return fail("no root element");
        if (peek() != '<' || starts("<!") || starts("<?") || starts("</")) return fail("a root element was expected");
        if (!read_element(root, 0)) return false;
        for (;;) {
            skip_ws();
            if (eof()) return true;
            if (!starts("<!--")) return fail("content after the root element");
            if (!skip_comment()) return false;
        }
    }
};

}  // namespace

bool parse(const std::string& text, Node& root, std::string& error) {
    Parser p(text);
    Node tmp;
    if (!p.run(tmp)) {
        error = p.error;
        return false;
    }
    root = std::move(tmp);
    return true;
}

}  // namespace ncad::laser::xml
