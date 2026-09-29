// StateColor and UITheme live in libslic3r_gui; this is the only suite that links it.
// Same Windows include prologue as test_dev_mapping.cpp (wx pulls in <windows.h>; keep
// WIN32_LEAN_AND_MEAN / NOMINMAX ahead of the Catch2 headers).
#ifdef WIN32
    #ifndef WIN32_LEAN_AND_MEAN
        #define WIN32_LEAN_AND_MEAN
    #endif
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
    #include <Windows.h>
#endif

#include <catch2/catch_all.hpp>

#include <algorithm>
#include <set>

#include <wx/colour.h>

#include "libslic3r/AppConfig.hpp"
#include "slic3r/GUI/UITheme.hpp"
#include "slic3r/GUI/Widgets/StateColor.hpp"

using namespace Slic3r;

namespace {

// The final colors the R10 theme draws wx chrome with: the VGA palette, with #010101 standing in
// for black because #000000 is the key for light-mode text.
const std::set<unsigned long> r10_palette{0x010101, 0x55FF55, 0xAAAAAA, 0x555555, 0xFF5555, 0xFFFF55};

unsigned long rgb(const wxColour &c) { return (unsigned long) (c.Red() << 16 | c.Green() << 8 | c.Blue()); }

// Restores the default theme and color mode when a test ends.
struct ScopedR10Mode
{
    ScopedR10Mode()
    {
        StateColor::SetDarkMode(true);
        StateColor::SetR10Mode(true);
    }
    ~ScopedR10Mode()
    {
        StateColor::SetR10Mode(false);
        StateColor::SetDarkMode(false);
    }
};

void init_theme(const char *theme, const char *language)
{
    AppConfig config;
    config.set(GUI::UITheme::CONFIG_KEY, theme);
    config.set("language", language);
    GUI::UITheme::init(config);
}

} // namespace

TEST_CASE("The R10 color map covers every color the dark map covers", "[UITheme]")
{
    // wxColour ordering is private to StateColor.cpp, so look keys up by equality here.
    const auto &r10 = StateColor::GetR10Map();
    for (const auto &[light, dark] : StateColor::GetDarkMap()) {
        INFO("light color " << light.GetAsString(wxC2S_HTML_SYNTAX).ToStdString());
        CHECK(std::any_of(r10.begin(), r10.end(), [&light = light](const auto &entry) { return entry.first == light; }));
    }
}

TEST_CASE("The R10 color map sends every color into the VGA palette", "[UITheme]")
{
    for (const auto &[light, r10] : StateColor::GetR10Map()) {
        INFO("light color " << light.GetAsString(wxC2S_HTML_SYNTAX).ToStdString());
        CHECK(r10_palette.count(rgb(r10)) == 1);
    }
}

TEST_CASE("Mapping a color twice under R10 gives the same color as mapping it once", "[UITheme]")
{
    ScopedR10Mode r10_mode;
    for (const auto &[light, r10] : StateColor::GetR10Map()) {
        INFO("light color " << light.GetAsString(wxC2S_HTML_SYNTAX).ToStdString());
        const wxColour once = StateColor::darkModeColorFor(light);
        CHECK(once == r10);
        CHECK(StateColor::darkModeColorFor(once) == once);
    }
}

TEST_CASE("Leaving R10 mode off keeps the regular dark map", "[UITheme]")
{
    StateColor::SetDarkMode(true);
    StateColor::SetR10Mode(false);
    for (const auto &[light, dark] : StateColor::GetDarkMap())
        CHECK(StateColor::darkModeColorFor(light) == dark);
    StateColor::SetDarkMode(false);
}

TEST_CASE("R10 recolors neutral icon tones to the palette and keeps saturated colors", "[UITheme]")
{
    // light gray line, mid-gray button fill, dark fill, accent teal, warning orange, transparent pixel
    unsigned char pixels[] = {0xB6, 0xB6, 0xB6, 0xFF, 0x54, 0x54, 0x5A, 0xFF, 0x2D, 0x2D, 0x31, 0xFF,
                              0x00, 0x96, 0x88, 0xFF, 0xFF, 0x6F, 0x00, 0xFF, 0x80, 0x80, 0x80, 0x00};

    init_theme(GUI::UITheme::THEME_R10, "en");
    GUI::UITheme::recolor_icon(pixels, 6);
    init_theme(GUI::UITheme::THEME_DEFAULT, "en");

    // light line work: system text
    CHECK(pixels[0] == 0x55);
    CHECK(pixels[1] == 0xFF);
    CHECK(pixels[2] == 0x55);
    // mid gray: panel shade, so light lines drawn over it stay readable
    CHECK(pixels[4] == 0x55);
    CHECK(pixels[5] == 0x55);
    CHECK(pixels[6] == 0x55);
    // dark fill: ground
    CHECK(pixels[8] == 0x00);
    CHECK(pixels[10] == 0x00);
    // accent: white ink
    CHECK(pixels[12] == 0xFF);
    CHECK(pixels[13] == 0xFF);
    // warning orange untouched
    CHECK(pixels[16] == 0xFF);
    CHECK(pixels[17] == 0x6F);
    // transparent pixel untouched
    CHECK(pixels[20] == 0x80);
    CHECK(pixels[23] == 0x00);
}

TEST_CASE("R10 draws an icon filled with the accent as the gray highlight bar", "[UITheme]")
{
    // three accent pixels and one light line pixel: a selected-state icon
    unsigned char pixels[] = {0x00, 0x96, 0x88, 0xFF, 0x00, 0x96, 0x88, 0xFF, 0x00, 0x96, 0x88, 0xFF, 0xC4, 0xC4, 0xC4, 0xFF};

    init_theme(GUI::UITheme::THEME_R10, "en");
    GUI::UITheme::recolor_icon(pixels, 4);
    init_theme(GUI::UITheme::THEME_DEFAULT, "en");

    // accent fill: highlight gray
    CHECK(pixels[0] == 0xAA);
    CHECK(pixels[1] == 0xAA);
    // line work on it: on-highlight dark gray
    CHECK(pixels[12] == 0x55);
    CHECK(pixels[13] == 0x55);
}

TEST_CASE("The default theme leaves icons untouched", "[UITheme]")
{
    unsigned char pixels[] = {0xB6, 0xB6, 0xB6, 0xFF};
    init_theme(GUI::UITheme::THEME_DEFAULT, "en");
    GUI::UITheme::recolor_icon(pixels, 1);
    CHECK(pixels[0] == 0xB6);
    CHECK_FALSE(GUI::UITheme::is_r10());
}

TEST_CASE("R10 keeps the regular font for scripts the VGA face lacks", "[UITheme]")
{
    const auto [language, vga_font] = GENERATE(table<const char *, bool>({{"en", true}, {"ru", true}, {"de", true}, {"zh_CN", false}, {"ja", false}, {"ko", false}, {"th", false}, {"vi", false}}));
    init_theme(GUI::UITheme::THEME_R10, language);
    const bool uses_vga = GUI::UITheme::use_r10_font();
    init_theme(GUI::UITheme::THEME_DEFAULT, "en");
    INFO("language " << language);
    CHECK(uses_vga == vga_font);
}
