#pragma once

// Text -> outlines through Emboss (Emboss::create_font_file + text2shapes).
//
// Font lookup: a file path always works. A family name is resolved with Emboss::get_font_path,
// which only finds fonts on Windows; on Linux/macOS the GUI resolves names to paths through its
// wx font utilities (WxFontUtils) and stores the path in LaserShape::font. If name resolution
// needs wx on every platform, a GUI-side resolver is passed in by the caller: this header stays
// wx-free.

#include "libslic3r/Laser/LaserTypes.hpp"

#include <string>

namespace Slic3r::Laser {

// Closed outlines (holes reversed, union-ready), scaled, in the Text shape's local frame: first
// baseline at y = 0, later lines below it, each line aligned about x = 0. height_mm = the Emboss
// letter size; spacing_mm = extra letter spacing; bold/italic are synthesized (Emboss boldness /
// skew) when the font has no such face. Empty + `error` when the font cannot be loaded.
Polygons text_outlines(const std::string& text, const std::string& font_name_or_path, double height_mm,
                       double spacing_mm = 0, bool bold = false, bool italic = false,
                       TextAlign align = TextAlign::Left, std::string* error = nullptr);

// Recomputes shape.text_outlines from its text fields; false (outlines kept) when the font fails.
bool update_text_outlines(LaserShape& shape, std::string* error = nullptr);

} // namespace Slic3r::Laser
