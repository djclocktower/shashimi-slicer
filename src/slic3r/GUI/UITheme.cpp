#include "UITheme.hpp"

#include "libslic3r/AppConfig.hpp"

#include <wx/intl.h>

#include <algorithm>

namespace Slic3r { namespace GUI { namespace UITheme {

static bool s_r10      = false;
static bool s_r10_font = false;

// The VGA face has no glyphs for these scripts.
static bool font_covers_language(const std::string &language)
{
    for (const char *prefix : {"ja", "ko", "th", "vi", "zh"})
        if (language.rfind(prefix, 0) == 0)
            return false;
    return true;
}

void init(const AppConfig &config)
{
    s_r10 = config.get(CONFIG_KEY) == THEME_R10;

    std::string language = config.get("language");
    if (language.empty())
        language = wxLocale::GetLanguageCanonicalName(wxLocale::GetSystemLanguage()).ToUTF8().data();
    s_r10_font = s_r10 && font_covers_language(language);
}

bool is_r10() { return s_r10; }

bool use_r10_font() { return s_r10_font; }

void recolor_icon(unsigned char *rgba, size_t n_pixels)
{
    if (!s_r10 || rgba == nullptr)
        return;

    for (size_t i = 0; i < n_pixels; ++i) {
        unsigned char *px = rgba + i * 4;
        if (px[3] == 0)
            continue;
        const int r = px[0], g = px[1], b = px[2];
        const int hi = std::max({r, g, b});
        const int lo = std::min({r, g, b});
        if (g > r + 60 && b > r + 50) {
            // Accent teal (#009688 and its hover / dark variants): white ink.
            px[0] = px[1] = px[2] = 0xFF;
        } else if (hi - lo < 48) {
            // Neutral line work and fills: dark fills fall to ground, everything else is system text.
            const bool dark = (hi + lo) / 2 < 64;
            px[0] = dark ? 0x00 : 0x55;
            px[1] = dark ? 0x00 : 0xFF;
            px[2] = dark ? 0x00 : 0x55;
        }
    }
}

}}} // namespace Slic3r::GUI::UITheme
