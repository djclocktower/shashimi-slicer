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
// letter size; spacing_mm = extra letter spacing; bold/italic are synthesized (italic: Emboss skew
// 0.2; bold: a stroke of 3 % of the height) when no such face file is found (resolve_font_path).
// Empty + `error` when the font cannot be loaded.
Polygons text_outlines(const std::string& text, const std::string& font_name_or_path, double height_mm,
                       double spacing_mm = 0, bool bold = false, bool italic = false,
                       TextAlign align = TextAlign::Left, std::string* error = nullptr);

// A font file for `name`: an existing file path as is; otherwise a family name matched against
// the font file names in the system font directories (Windows: C:\Windows\Fonts, macOS:
// /Library/Fonts, /System/Library/Fonts, Linux: /usr/share/fonts, ~/.fonts, ~/.local/share/fonts),
// the bold / italic face's own file first (`exact_face` = true when one was found), then
// Emboss::get_font_path (Windows), then fallback_font_path(). Empty when nothing is found.
std::string resolve_font_path(const std::string& name, bool bold = false, bool italic = false, bool* exact_face = nullptr);
// The bundled font (resources/fonts/NotoSansKR-Regular.ttf); empty when missing.
std::string fallback_font_path();

// Recomputes shape.text_outlines from its text fields; false (outlines kept) when the font fails.
bool update_text_outlines(LaserShape& shape, std::string* error = nullptr);

} // namespace Slic3r::Laser
