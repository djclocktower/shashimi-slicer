#pragma once

#include "libslic3r/Color.hpp"

#include <cstddef>

namespace Slic3r {

class AppConfig;

namespace GUI {

// Optional interface themes, layered on top of the dark color mode.
//
// The theme is read once at startup and stays fixed for the session: fonts, widget colors and
// cached bitmaps are all built from it, so a change made in Preferences applies on the next launch.
// The default theme leaves every existing code path untouched.
namespace UITheme {

// AppConfig key and its values.
inline constexpr const char *CONFIG_KEY    = "ui_theme";
inline constexpr const char *THEME_DEFAULT = "default";
inline constexpr const char *THEME_R10     = "r10";

// Latch the theme from the config. Call once, before fonts and colors are initialized.
void init(const AppConfig &config);

// "R10 Drafting": a 1980s CAD workstation. Black screen, green system text in the IBM VGA
// bitmap face, gray inverse-video highlights, red frame rules and square corners.
// The design system lives in design/r10-drafting/. Forces the dark color mode.
bool is_r10();

// True when the R10 theme should also swap the UI font for the VGA face. The face covers
// Latin, Greek and Cyrillic; CJK, Thai and Vietnamese keep the regular fonts to stay legible.
bool use_r10_font();

// R10 font: face name as registered by the TTF, and the file under resources/fonts.
inline constexpr const char *R10_FONT_FACE = "WebPlus IBM VGA 8x16";
inline constexpr const char *R10_FONT_FILE = "WebPlus_IBM_VGA_8x16.ttf";

// R10 palette, Screen (VGA) theme of design/r10-drafting/tokens.json.
namespace R10 {
inline ColorRGBA vga(unsigned char r, unsigned char g, unsigned char b) { return ColorRGBA(r, g, b, (unsigned char) 255); }
inline const ColorRGBA GROUND      = vga(0x00, 0x00, 0x00); // screen and drawing area
inline const ColorRGBA SYSTEM_TEXT = vga(0x55, 0xFF, 0x55); // all system text
inline const ColorRGBA INK_DIM     = vga(0xAA, 0xAA, 0xAA); // dimmed text, highlight bar
inline const ColorRGBA INK_FAINT   = vga(0x55, 0x55, 0x55); // grid marks, underlays (ACI 8)
inline const ColorRGBA RULE        = vga(0xFF, 0x55, 0x55); // 1px frame rules
} // namespace R10

// Recolor a rasterized UI icon to the R10 palette in place: neutral line work becomes system-text
// green, dark fills become ground, the accent becomes white ink. Saturated colors that carry meaning
// (warnings, errors, axis colors) are left alone. No-op unless the R10 theme is active.
void recolor_icon(unsigned char *rgba, size_t n_pixels);

} // namespace UITheme
} // namespace GUI
} // namespace Slic3r
