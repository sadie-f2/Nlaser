// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, Sadie Forbes

// A deliberately small XML reader: exactly the subset LightBurn's .lbrn2 uses,
// so that a third-party parser (and its attack surface: DTDs, external
// entities) stays out of the core.
//
// Understood: elements, attributes (single or double quoted), text, comments,
// the leading `<?xml ... ?>` declaration, the five named entities and numeric
// character references, self-closing tags.
// Refused, with the line number: DOCTYPE and any other `<!`, CDATA, other
// processing instructions, unknown entities, duplicate attributes, an
// unclosed or mismatched tag, nesting deeper than 200, text outside the root.
#pragma once

#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace ncad::laser::xml {

struct Node {
    std::string name;
    std::vector<std::pair<std::string, std::string>> attrs;
    std::vector<Node> children;
    std::string text;  // character data directly inside, entities decoded, trimmed
    int line{0};       // 1-based line of the opening '<'

    // nullptr when absent.
    const std::string* attr(std::string_view key) const;
    // The first child with this name, or nullptr.
    const Node* child(std::string_view tag) const;
};

// False with `error` ("line N: ...") when the text is outside the subset;
// `root` is then untouched.
bool parse(const std::string& text, Node& root, std::string& error);

// &, <, >, " and ' as entities, for writing attribute values and text.
std::string escape(const std::string& s);

}  // namespace ncad::laser::xml
