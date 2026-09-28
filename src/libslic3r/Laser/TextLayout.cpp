#include "libslic3r/Laser/TextLayout.hpp"

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Emboss.hpp"
#include "libslic3r/Utils.hpp"

#include <boost/filesystem.hpp>
#include <boost/nowide/convert.hpp>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <map>
#include <memory>
#include <mutex>

namespace Slic3r::Laser {

namespace fs = boost::filesystem;

namespace {

// "DejaVu Sans-Bold" -> "dejavusansbold": file stems and family names compare in this form.
std::string normalize(const std::string& s)
{
    std::string out;
    for (unsigned char c : s)
        if (std::isalnum(c)) out += char(std::tolower(c));
    return out;
}

bool is_font_file(const fs::path& p)
{
    const std::string ext = normalize(p.extension().string());
    return ext == "ttf" || ext == "otf" || ext == "ttc";
}

// Normalized file stem -> path of every font file in the usual system directories. Built once.
const std::map<std::string, std::string>& system_font_index()
{
    static std::map<std::string, std::string> index;
    static std::once_flag once;
    std::call_once(once, [] {
        std::vector<fs::path> dirs;
#ifdef _WIN32
        dirs.emplace_back("C:\\Windows\\Fonts");
        if (const char* local = std::getenv("LOCALAPPDATA")) dirs.push_back(fs::path(local) / "Microsoft" / "Windows" / "Fonts");
#elif defined(__APPLE__)
        dirs = {"/Library/Fonts", "/System/Library/Fonts", "/System/Library/Fonts/Supplemental"};
        if (const char* home = std::getenv("HOME")) dirs.push_back(fs::path(home) / "Library" / "Fonts");
#else
        dirs = {"/usr/share/fonts", "/usr/local/share/fonts"};
        if (const char* home = std::getenv("HOME")) {
            dirs.push_back(fs::path(home) / ".fonts");
            dirs.push_back(fs::path(home) / ".local" / "share" / "fonts");
        }
#endif
        for (const fs::path& d : dirs) {
            boost::system::error_code ec;
            if (!fs::is_directory(d, ec)) continue;
            for (fs::recursive_directory_iterator it(d, fs::directory_options::skip_permission_denied, ec), end; it != end && !ec;
                 it.increment(ec))
                if (is_font_file(it->path()))
                    index.emplace(normalize(it->path().stem().string()), it->path().string());
        }
    });
    return index;
}

// Loaded font files by path (the file data is immutable; each call gets its own glyph cache).
std::shared_ptr<const Emboss::FontFile> load_font(const std::string& path)
{
    static std::mutex mutex;
    static std::map<std::string, std::shared_ptr<const Emboss::FontFile>> cache;
    std::lock_guard<std::mutex> lock(mutex);
    auto it = cache.find(path);
    if (it != cache.end()) return it->second;
    std::shared_ptr<const Emboss::FontFile> ff(Emboss::create_font_file(path.c_str()));
    if (ff) cache.emplace(path, ff);
    return ff;
}

} // namespace

std::string fallback_font_path()
{
    const fs::path p = fs::path(resources_dir()) / "fonts" / "NotoSansKR-Regular.ttf";
    boost::system::error_code ec;
    return fs::exists(p, ec) ? p.string() : std::string();
}

std::string resolve_font_path(const std::string& name, bool bold, bool italic, bool* exact_face)
{
    if (exact_face) *exact_face = false;
    boost::system::error_code ec;
    if (!name.empty() && fs::is_regular_file(fs::path(name), ec)) return name;
    const std::string family = normalize(name);
    if (!family.empty()) {
        // The face's own file first ("Arial Bold" -> arialbd / arial-bold / arialbold ...).
        std::vector<std::string> suffixes;
        if (bold && italic) suffixes = {"bolditalic", "boldoblique", "bi", "z"};
        else if (bold) suffixes = {"bold", "bd", "b"};
        else if (italic) suffixes = {"italic", "oblique", "i"};
        const auto& index = system_font_index();
        for (const std::string& suf : suffixes) {
            auto it = index.find(family + suf);
            if (it != index.end()) {
                if (exact_face) *exact_face = true;
                return it->second;
            }
        }
#ifdef _WIN32
        if (std::optional<std::wstring> p = Emboss::get_font_path(boost::nowide::widen(name)))
            return boost::nowide::narrow(*p);
#endif
        for (const std::string& suf : {std::string(), std::string("regular"), std::string("book"), std::string("roman")}) {
            auto it = index.find(family + suf);
            if (it != index.end()) return it->second;
        }
    }
    return fallback_font_path();
}

Polygons text_outlines(const std::string& text, const std::string& font_name_or_path, double height_mm, double spacing_mm,
                       bool bold, bool italic, TextAlign align, std::string* error)
{
    auto fail = [&](const std::string& msg) {
        if (error) *error = msg;
        return Polygons();
    };
    if (height_mm <= 0) return fail("The text height must be greater than zero.");
    if (text.empty()) return {};
    bool              exact = false;
    const std::string path  = resolve_font_path(font_name_or_path, bold, italic, &exact);
    if (path.empty()) return fail("The font \"" + font_name_or_path + "\" was not found and no fallback font is installed.");
    std::shared_ptr<const Emboss::FontFile> ff = load_font(path);
    if (!ff || ff->infos.empty()) return fail("The font file \"" + path + "\" could not be read.");

    Emboss::FontFileWithCache font;
    font.font_file = ff;
    font.cache     = std::make_shared<Emboss::Glyphs>();
    FontProp prop{float(height_mm)};
    const Emboss::FontFile::Info& info = ff->infos.front();
    // Letter spacing: FontProp::char_gap is in font units (size_in_mm per unit_per_em).
    if (spacing_mm != 0) prop.char_gap = int(std::lround(spacing_mm * info.unit_per_em / height_mm));
    // Synthesized italic: 0.2 shear (about 11 degrees), used only when no italic face was found.
    if (italic && !exact) prop.skew = 0.2f;
    using HA = FontProp::HorizontalAlign;
    prop.align = FontProp::Align(align == TextAlign::Center ? HA::center : align == TextAlign::Right ? HA::right : HA::left,
                                 FontProp::VerticalAlign::bottom);

    ExPolygons shapes;
    try {
        shapes = Emboss::text2shapes(font, text.c_str(), prop).expolygons;
    } catch (...) {
        return fail("The text could not be converted with the font \"" + path + "\".");
    }
    // Emboss' "bottom" puts the LAST baseline at y = 0: move the first one there instead.
    const int    lines   = int(Emboss::get_count_lines(text));
    const double to_mm   = Emboss::get_text_shape_scale(prop, *ff);
    const double dy_mm   = -(lines - 1) * Emboss::get_line_height(*ff, prop) * to_mm;
    const double k       = to_mm / SCALING_FACTOR;
    for (ExPolygon& e : shapes) {
        auto conv = [&](Polygon& p) {
            for (Point& pt : p.points) pt = Point(coord_t(std::llround(pt.x() * k)), coord_t(std::llround(pt.y() * k + scale_(dy_mm))));
        };
        conv(e.contour);
        for (Polygon& h : e.holes) conv(h);
    }
    // Synthesized bold: a stroke of 3 % of the height around every outline (1.5 % per side).
    if (bold && !exact) shapes = offset_ex(shapes, float(scale_(0.015 * height_mm)));
    if (error) error->clear();
    return to_polygons(std::move(shapes));
}

bool update_text_outlines(LaserShape& shape, std::string* error)
{
    std::string err;
    Polygons    p = text_outlines(shape.text, shape.font, shape.text_height_mm, shape.text_spacing_mm, shape.bold, shape.italic,
                                  shape.text_align, &err);
    if (!err.empty()) {
        if (error) *error = err;
        return false;
    }
    shape.text_outlines = std::move(p);
    return true;
}

} // namespace Slic3r::Laser
