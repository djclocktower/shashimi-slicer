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

static bool is_accent(const unsigned char *px) { return px[1] > px[0] + 60 && px[2] > px[0] + 50; }

void recolor_icon(unsigned char *rgba, size_t n_pixels)
{
    if (!s_r10 || rgba == nullptr)
        return;

    // An icon whose area is largely an accent fill shows a selected state. It is drawn as the inverse
    // highlight bar: a gray fill with dark-gray line work. Otherwise the accent is white ink.
    size_t n_accent = 0;
    for (size_t i = 0; i < n_pixels; ++i) {
        const unsigned char *px = rgba + i * 4;
        n_accent += px[3] > 127 && is_accent(px);
    }
    const bool highlighted = n_accent * 4 > n_pixels;

    for (size_t i = 0; i < n_pixels; ++i) {
        unsigned char *px = rgba + i * 4;
        if (px[3] == 0)
            continue;
        const int hi = std::max({px[0], px[1], px[2]});
        const int lo = std::min({px[0], px[1], px[2]});
        if (is_accent(px)) {
            // Accent teal (#009688 and its hover / dark variants): highlight gray or white ink.
            px[0] = px[1] = px[2] = highlighted ? 0xAA : 0xFF;
        } else if (hi - lo < 48) {
            // Neutral tones in three steps, so light line work stays readable on the mid-gray button
            // fills some icons carry: dark fills fall to ground, mid grays to panel shade, light
            // tones become system text (on-highlight dark gray inside a highlighted icon).
            const int lightness = (hi + lo) / 2;
            if (lightness < 60)
                px[0] = px[1] = px[2] = 0x00;
            else if (lightness < 120 || highlighted)
                px[0] = px[1] = px[2] = 0x55;
            else {
                px[0] = 0x55;
                px[1] = 0xFF;
                px[2] = 0x55;
            }
        }
    }
}

}}} // namespace Slic3r::GUI::UITheme
