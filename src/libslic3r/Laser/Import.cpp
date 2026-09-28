#include "libslic3r/Laser/Import.hpp"

#include "libslic3r/Laser/TextLayout.hpp"
#include "libslic3r/NSVGUtils.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/beast/core/detail/base64.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/nowide/fstream.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <sstream>

namespace Slic3r::Laser {

namespace {

constexpr double kTol = 0.02;   // curve flattening tolerance, mm

bool read_file(const std::string& path, std::string& out)
{
    boost::nowide::ifstream f(path, std::ios::binary);
    if (!f) return false;
    std::ostringstream ss;
    ss << f.rdbuf();
    out = ss.str();
    return true;
}

int nearest_layer(int r, int g, int b)
{
    int best = 0;
    long best_d = -1;
    for (int i = 0; i < kLayerCount; ++i) {
        const Rgb c = kLayerPalette[i];
        const long d = long(c.r - r) * (c.r - r) + long(c.g - g) * (c.g - g) + long(c.b - b) * (c.b - b);
        if (best_d < 0 || d < best_d) { best_d = d; best = i; }
    }
    return best;
}

// A layer named "C05" or "5" -> 5; -1 otherwise.
int layer_from_name(const std::string& name)
{
    std::string s = boost::algorithm::trim_copy(name);
    if (!s.empty() && (s[0] == 'C' || s[0] == 'c')) s = s.substr(1);
    if (s.empty() || s.size() > 2 || !std::all_of(s.begin(), s.end(), ::isdigit)) return -1;
    const int v = std::stoi(s);
    return v < kLayerCount ? v : -1;
}

// Adaptive cubic Bezier flattening (control-point distance to the chord), end point included.
void flatten_cubic(std::vector<Vec2d>& out, const Vec2d& p0, const Vec2d& p1, const Vec2d& p2, const Vec2d& p3, int depth = 0)
{
    const Vec2d  d  = p3 - p0;
    const double l2 = d.squaredNorm();
    auto dist = [&](const Vec2d& p) {
        return l2 > 1e-18 ? std::abs(d.x() * (p.y() - p0.y()) - d.y() * (p.x() - p0.x())) / std::sqrt(l2) : (p - p0).norm();
    };
    if (depth >= 16 || std::max(dist(p1), dist(p2)) <= kTol) {
        out.push_back(p3);
        return;
    }
    const Vec2d a = (p0 + p1) / 2, b = (p1 + p2) / 2, c = (p2 + p3) / 2, ab = (a + b) / 2, bc = (b + c) / 2, m = (ab + bc) / 2;
    flatten_cubic(out, p0, a, ab, m, depth + 1);
    flatten_cubic(out, m, bc, c, p3, depth + 1);
}

// Arc points from a0 to a1 (radians, signed sweep), start excluded, end included.
void flatten_arc(std::vector<Vec2d>& out, const Vec2d& c, double r, double a0, double a1)
{
    const double sweep = a1 - a0;
    const double step  = r > kTol ? 2 * std::acos(1 - kTol / r) : M_PI / 4;
    const int    n     = std::clamp(int(std::ceil(std::abs(sweep) / std::max(step, 1e-3))), 1, 20000);
    for (int k = 1; k <= n; ++k) {
        const double a = a0 + sweep * k / n;
        out.emplace_back(c + r * Vec2d(std::cos(a), std::sin(a)));
    }
}

LaserPath make_path(const std::vector<Vec2d>& pts, bool closed, const Transform2d& t = Transform2d::Identity())
{
    LaserPath p;
    p.closed = closed;
    for (const Vec2d& v : pts) {
        const Vec2d w = t * v;
        if (!std::isfinite(w.x()) || !std::isfinite(w.y()) || std::abs(w.x()) > 1e6 || std::abs(w.y()) > 1e6) continue;   // corrupt input
        const Point q = Point::new_scale(w);
        if (p.pts.points.empty() || p.pts.points.back() != q) p.pts.points.push_back(q);
    }
    if (closed && p.pts.size() > 1 && p.pts.points.front() == p.pts.points.back()) p.pts.points.pop_back();
    return p;
}

LaserShape path_shape(LaserPaths paths, int layer, const std::string& name)
{
    LaserShape s;
    s.type  = ShapeType::Path;
    s.paths = std::move(paths);
    s.layer = layer;
    s.name  = name;
    return s;
}

std::string lower_ext(const std::string& path)
{
    return boost::algorithm::to_lower_copy(boost::filesystem::path(path).extension().string());
}

// Grayscale image shape from encoded bytes (PNG, JPEG, ...); transparent pixels become white.
constexpr double kMaxImagePixels = 36e6;   // 6000 x 6000

// `downscale` (optional) gets the resize factor when the image was larger than kMaxImagePixels.
bool decode_gray(const std::string& bytes, LaserShape& s, double* downscale = nullptr)
{
    try {
        const cv::Mat raw(1, int(bytes.size()), CV_8UC1, const_cast<char*>(bytes.data()));
        cv::Mat img = cv::imdecode(raw, cv::IMREAD_UNCHANGED);
        if (img.empty()) return false;
        if (img.depth() != CV_8U) img.convertTo(img, CV_8U, img.depth() == CV_16U ? 1. / 257. : 1.);
        cv::Mat gray;
        if (img.channels() == 4) {
            std::vector<cv::Mat> ch;
            cv::split(img, ch);
            cv::Mat bgr;
            cv::merge(std::vector<cv::Mat>{ch[0], ch[1], ch[2]}, bgr);
            cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
            // Composite on white: g' = g * a + 255 * (1 - a).
            gray.convertTo(gray, CV_32F);
            cv::Mat a;
            ch[3].convertTo(a, CV_32F, 1. / 255.);
            gray = gray.mul(a) + (1. - a) * 255.;
            gray.convertTo(gray, CV_8U);
        } else if (img.channels() == 3)
            cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        else
            gray = img;
        // Cap the stored resolution: the pixels live in the document, the 3MF and every undo step.
        if (const double px = double(gray.cols) * gray.rows; px > kMaxImagePixels) {
            const double f = std::sqrt(kMaxImagePixels / px);
            cv::resize(gray, gray, cv::Size(std::max(1, int(gray.cols * f)), std::max(1, int(gray.rows * f))), 0, 0, cv::INTER_AREA);
            if (downscale) *downscale = f;
        }
        s.type    = ShapeType::Image;
        s.image_w = gray.cols;
        s.image_h = gray.rows;
        s.gray.resize(size_t(gray.cols) * gray.rows);
        for (int y = 0; y < gray.rows; ++y) std::copy_n(gray.ptr<uint8_t>(y), gray.cols, s.gray.data() + size_t(y) * gray.cols);
        return true;
    } catch (...) {
        return false;
    }
}

} // namespace

// ---- SVG ---------------------------------------------------------------------------------------

ImportResult import_svg(const std::string& path)
{
    ImportResult res;
    std::unique_ptr<std::string> data = read_from_disk(path);
    if (!data) {
        res.error = "The SVG file could not be opened.";
        return res;
    }
    // NanoSVG converts px (96 dpi), in, cm, pt ... to mm.
    NSVGimage_ptr img = nsvgParse(*data, "mm", 96.f);
    if (!img || !img->shapes) {
        res.error = "The SVG file has no shapes that can be imported.";
        return res;
    }
    // NanoSVG leaves image->height in px (96 dpi) while it scales the shapes to mm.
    const double H = img->height * 25.4 / 96.;
    auto to_ws = [H](float x, float y) { return Vec2d(x, H - y); };
    std::vector<LaserShape> flat;
    bool gradient_warned = false;
    for (NSVGshape* sh = img->shapes; sh; sh = sh->next) {
        if (!(sh->flags & NSVG_FLAGS_VISIBLE)) continue;
        const bool fill   = sh->fill.type != NSVG_PAINT_NONE;
        const bool stroke = sh->stroke.type != NSVG_PAINT_NONE;
        if (!fill && !stroke) continue;
        const NSVGpaint& paint = stroke ? sh->stroke : sh->fill;
        int layer = 0;
        if (paint.type == NSVG_PAINT_COLOR)
            layer = nearest_layer(paint.color & 0xFF, (paint.color >> 8) & 0xFF, (paint.color >> 16) & 0xFF);
        else if (!gradient_warned) {
            res.warnings.push_back("Gradient colours are not supported; those shapes were put on layer C00.");
            gradient_warned = true;
        }
        LaserPaths paths;
        for (NSVGpath* p = sh->paths; p; p = p->next) {
            if (p->npts < 1) continue;
            std::vector<Vec2d> pts{to_ws(p->pts[0], p->pts[1])};
            for (int i = 0; i + 3 < p->npts; i += 3) {
                const float* c = &p->pts[i * 2];
                flatten_cubic(pts, to_ws(c[0], c[1]), to_ws(c[2], c[3]), to_ws(c[4], c[5]), to_ws(c[6], c[7]));
            }
            // Filled shapes are regions (closed); a stroke-only path keeps the SVG's own closed flag.
            LaserPath lp = make_path(pts, fill || p->closed);
            if (lp.pts.size() >= 2) paths.push_back(std::move(lp));
        }
        if (paths.empty()) continue;
        flat.push_back(path_shape(std::move(paths), layer, sh->id));
    }
    // NanoSVG flattens groups; a group's id is inherited by its id-less children, so a run of
    // shapes with the same id is read back as one group.
    for (size_t i = 0; i < flat.size();) {
        size_t j = i + 1;
        while (!flat[i].name.empty() && j < flat.size() && flat[j].name == flat[i].name) ++j;
        int parent = -1;
        if (j - i > 1) {
            LaserShape g;
            g.type = ShapeType::Group;
            g.name = flat[i].name;
            res.shapes.push_back(g);
            parent = int(res.shapes.size()) - 1;
        }
        for (; i < j; ++i) {
            flat[i].parent = parent;
            res.shapes.push_back(std::move(flat[i]));
        }
    }
    if (res.shapes.empty()) res.error = "The SVG file has no visible shapes.";
    return res;
}

// ---- DXF ---------------------------------------------------------------------------------------

namespace {

struct DxfEnt {
    std::string                              type;
    std::vector<std::pair<int, std::string>> codes;
    std::vector<DxfEnt>                      vertices;   // POLYLINE
    double num(int code, double def = 0) const
    {
        for (const auto& [c, v] : codes)
            if (c == code) {
                try { return std::stod(v); } catch (...) { return def; }
            }
        return def;
    }
    std::string str(int code) const
    {
        for (const auto& [c, v] : codes)
            if (c == code) return v;
        return {};
    }
    std::vector<double> all(int code) const
    {
        std::vector<double> out;
        for (const auto& [c, v] : codes)
            if (c == code) {
                try { out.push_back(std::stod(v)); } catch (...) { out.push_back(0); }
            }
        return out;
    }
};

struct DxfBlock {
    Vec2d               base{0, 0};
    std::vector<DxfEnt> ents;
};

Rgb aci_rgb(int aci)
{
    static const Rgb t[10] = {{0, 0, 0},     {255, 0, 0},   {255, 255, 0},   {0, 255, 0},     {0, 255, 255},
                              {0, 0, 255},   {255, 0, 255}, {0, 0, 0},       {128, 128, 128}, {192, 192, 192}};
    return t[std::clamp(aci, 0, 9)];
}

class DxfReader {
public:
    ImportResult                    res;
    double                          units{1};
    std::map<std::string, int>      layer_aci;
    std::map<std::string, DxfBlock> blocks;
    std::vector<DxfEnt>             entities;
    std::map<std::string, int>      unsupported;
    bool                            capped{false};

    bool parse(const std::string& text)
    {
        std::vector<std::pair<int, std::string>> pairs;
        std::istringstream in(text);
        std::string code, value;
        while (std::getline(in, code) && std::getline(in, value)) {
            boost::algorithm::trim(code);
            if (!value.empty() && value.back() == '\r') value.pop_back();
            try {
                pairs.emplace_back(std::stoi(code), boost::algorithm::trim_copy(value));
            } catch (...) {
                return false;   // binary DXF or garbage
            }
        }
        std::string section;
        DxfBlock*   block = nullptr;
        for (size_t i = 0; i < pairs.size();) {
            const auto& [c, v] = pairs[i];
            if (c != 0) { ++i; continue; }
            // Collect one record: the 0-code line and everything up to the next 0 code.
            DxfEnt e;
            e.type = v;
            size_t j = i + 1;
            for (; j < pairs.size() && pairs[j].first != 0; ++j) e.codes.push_back(pairs[j]);
            i = j;
            if (e.type == "SECTION") {
                section = e.str(2);
                // HEADER variables (9 name, then value codes) have no 0 codes: they are all in this record.
                for (size_t k = 0; k + 1 < e.codes.size(); ++k)
                    if (e.codes[k].first == 9 && e.codes[k].second == "$INSUNITS") units = unit_scale(std::atoi(e.codes[k + 1].second.c_str()));
                continue;
            }
            if (e.type == "ENDSEC") { section.clear(); block = nullptr; continue; }
            if (section == "TABLES") {
                if (e.type == "LAYER") layer_aci[e.str(2)] = std::abs(int(e.num(62, 7)));
            } else if (section == "BLOCKS") {
                if (e.type == "BLOCK") {
                    block = &blocks[e.str(2)];
                    block->base = Vec2d(e.num(10), e.num(20));
                } else if (e.type == "ENDBLK")
                    block = nullptr;
                else if (block)
                    add(block->ents, std::move(e));
            } else if (section == "ENTITIES")
                add(entities, std::move(e));
        }
        return true;
    }

    static double unit_scale(int u)
    {
        switch (u) {
        case 1: return 25.4;
        case 2: return 304.8;
        case 3: return 1609344.;
        case 5: return 10;
        case 6: return 1000;
        case 8: return 25.4e-6;
        case 9: return 0.0254;
        case 10: return 914.4;
        case 13: return 1e-3;
        case 14: return 100;
        default: return 1;   // 0 unitless, 4 mm
        }
    }

    // VERTEX / SEQEND attach to the POLYLINE before them.
    void add(std::vector<DxfEnt>& list, DxfEnt&& e)
    {
        if (e.type == "VERTEX" && !list.empty() && list.back().type == "POLYLINE") list.back().vertices.push_back(std::move(e));
        else if (e.type != "SEQEND") list.push_back(std::move(e));
    }

    int layer_of(const DxfEnt& e, int block_aci)
    {
        const std::string name = e.str(8);
        const int named = layer_from_name(name);
        if (named >= 0) return named;
        int aci = int(e.num(62, 256));
        if (aci == 0) aci = block_aci;                       // BYBLOCK
        if (aci == 256) {                                    // BYLAYER
            auto it = layer_aci.find(name);
            aci = it != layer_aci.end() ? it->second : 7;
        }
        aci = std::abs(aci);
        if (aci >= 1 && aci <= 9) {
            const Rgb c = aci_rgb(aci);
            return nearest_layer(c.r, c.g, c.b);
        }
        return aci % kLayerCount;
    }

    void emit(const std::vector<DxfEnt>& ents, const Transform2d& T, int block_aci, int parent, int depth)
    {
        for (const DxfEnt& e : ents) {
            // Nested INSERTs multiply: a block inserting itself ten times is 10^16 shapes (~400 bytes each).
            if (res.shapes.size() >= 250000) {
                if (!capped) res.warnings.push_back("The drawing has more than 250000 shapes; the rest was skipped.");
                capped = true;
                return;
            }
            const int layer = layer_of(e, block_aci);
            std::vector<Vec2d> pts;
            bool closed = false;
            if (e.type == "LINE") {
                pts = {{e.num(10), e.num(20)}, {e.num(11), e.num(21)}};
            } else if (e.type == "LWPOLYLINE" || e.type == "POLYLINE") {
                std::vector<Vec2d>  v;
                std::vector<double> bulge;
                if (e.type == "LWPOLYLINE") {
                    // Vertices: each 10 starts one; 42 (bulge) belongs to the vertex before it.
                    for (const auto& [c, val] : e.codes) {
                        const double d = std::atof(val.c_str());
                        if (c == 10) { v.emplace_back(d, 0); bulge.push_back(0); }
                        else if (c == 20 && !v.empty()) v.back().y() = d;
                        else if (c == 42 && !bulge.empty()) bulge.back() = d;
                    }
                } else
                    for (const DxfEnt& vx : e.vertices) {
                        v.emplace_back(vx.num(10), vx.num(20));
                        bulge.push_back(vx.num(42));
                    }
                closed = (int(e.num(70)) & 1) != 0;
                if (v.empty()) continue;
                pts.push_back(v[0]);
                const size_t n = closed ? v.size() : v.size() - 1;
                for (size_t k = 0; k < n; ++k) {
                    const Vec2d a = v[k], b = v[(k + 1) % v.size()];
                    const double bg = bulge[k];
                    if (std::abs(bg) < 1e-9) { pts.push_back(b); continue; }
                    // Bulge = tan(sweep / 4); positive = counter-clockwise.
                    const double sweep = 4 * std::atan(bg), chord = (b - a).norm();
                    if (chord < 1e-12) continue;
                    const double r = chord / (2 * std::sin(std::abs(sweep) / 2));
                    const Vec2d  dir = (b - a) / chord, left(-dir.y(), dir.x());
                    // Signed centre offset from the chord midpoint: left of a->b for a CCW arc under 180 deg.
                    const Vec2d  c = (a + b) / 2 + left * (chord / 2 / std::tan(sweep / 2));
                    const double a0 = std::atan2(a.y() - c.y(), a.x() - c.x());
                    flatten_arc(pts, c, r, a0, a0 + sweep);
                    pts.back() = b;
                }
                if (closed && pts.size() > 1) pts.pop_back();   // the closing point repeats v[0]
            } else if (e.type == "CIRCLE") {
                const Vec2d c(e.num(10), e.num(20));
                const double r = e.num(40);
                pts.push_back(c + Vec2d(r, 0));
                flatten_arc(pts, c, r, 0, 2 * M_PI);
                pts.pop_back();
                closed = true;
            } else if (e.type == "ARC") {
                const Vec2d  c(e.num(10), e.num(20));
                const double r = e.num(40), a0 = e.num(50) * M_PI / 180;
                double a1 = e.num(51) * M_PI / 180;
                // fmod, not a += 2 pi loop: an angle of 1e300 or inf in a corrupt file must not hang.
                a1 = a0 + std::fmod(a1 - a0, 2 * M_PI);
                if (a1 <= a0) a1 += 2 * M_PI;
                if (!std::isfinite(a1) || !std::isfinite(r)) continue;
                pts.push_back(c + r * Vec2d(std::cos(a0), std::sin(a0)));
                flatten_arc(pts, c, r, a0, a1);
            } else if (e.type == "ELLIPSE") {
                const Vec2d  c(e.num(10), e.num(20)), major(e.num(11), e.num(21));
                const double ratio = e.num(40, 1), t0 = e.num(41, 0);
                double t1 = e.num(42, 2 * M_PI);
                t1 = t0 + std::fmod(t1 - t0, 2 * M_PI);
                if (t1 <= t0) t1 += 2 * M_PI;
                if (!std::isfinite(t1)) continue;
                const Vec2d minor = Vec2d(-major.y(), major.x()) * ratio;
                const double R = major.norm();
                const int n = std::clamp(int(std::ceil((t1 - t0) / (2 * std::acos(1 - std::min(kTol / std::max(R, kTol), 1.))))), 8, 20000);
                for (int k = 0; k <= n; ++k) {
                    const double t = t0 + (t1 - t0) * k / n;
                    pts.push_back(c + major * std::cos(t) + minor * std::sin(t));
                }
                closed = std::abs(t1 - t0 - 2 * M_PI) < 1e-6;
                if (closed) pts.pop_back();
            } else if (e.type == "SPLINE") {
                pts = spline(e);
                closed = (int(e.num(70)) & 1) != 0;
                if (closed && pts.size() > 2 && (pts.front() - pts.back()).norm() < 1e-9) pts.pop_back();
            } else if (e.type == "TEXT" || e.type == "MTEXT") {
                LaserShape s;
                s.type = ShapeType::Text;
                s.layer = layer;
                s.parent = parent;
                s.text = e.type == "MTEXT" ? mtext(e.str(3) + e.str(1)) : e.str(1);
                const Vec2d sx = T.linear().col(0);
                s.text_height_mm = e.num(40, 2.5) * sx.norm();
                Transform2d local = Transform2d::Identity();
                local.translate(Vec2d(e.num(10), e.num(20)));
                local.rotate(e.num(50) * M_PI / 180);
                s.xform = T * local;
                s.xform.linear() /= std::max(sx.norm(), 1e-12);   // height already carries the scale
                update_text_outlines(s);
                s.name = s.text;
                res.shapes.push_back(std::move(s));
                continue;
            } else if (e.type == "INSERT") {
                auto it = blocks.find(e.str(2));
                if (it == blocks.end() || depth > 16) continue;
                Transform2d local = Transform2d::Identity();
                local.translate(Vec2d(e.num(10), e.num(20)));
                local.rotate(e.num(50) * M_PI / 180);
                local.scale(Vec2d(e.num(41, 1), e.num(42, 1)));
                local.translate(-it->second.base);
                LaserShape g;
                g.type   = ShapeType::Group;
                g.name   = e.str(2);
                g.parent = parent;
                res.shapes.push_back(g);
                const int gi = int(res.shapes.size()) - 1;
                int aci = int(e.num(62, 256));
                if (aci == 256) { auto l = layer_aci.find(e.str(8)); aci = l != layer_aci.end() ? l->second : 7; }
                emit(it->second.ents, T * local, aci, gi, depth + 1);
                continue;
            } else {
                ++unsupported[e.type];
                continue;
            }
            if (pts.size() < 2) continue;
            LaserPath p = make_path(pts, closed, T);
            if (p.pts.size() < 2) continue;
            LaserShape s = path_shape({std::move(p)}, layer, e.type);
            s.parent = parent;
            res.shapes.push_back(std::move(s));
        }
    }

    // Non-uniform rational B-spline by de Boor (weights from code 41); fit points only -> polyline.
    static std::vector<Vec2d> spline(const DxfEnt& e)
    {
        const int           p  = std::clamp(int(e.num(71, 3)), 1, 11);
        std::vector<double> U  = e.all(40);
        const std::vector<double> xs = e.all(10), ys = e.all(20);
        std::vector<double> w = e.all(41);
        std::vector<Vec2d> P;
        for (size_t i = 0; i < std::min(xs.size(), ys.size()); ++i) P.emplace_back(xs[i], ys[i]);
        if (P.size() < 2) {
            std::vector<Vec2d> fit;
            const std::vector<double> fx = e.all(11), fy = e.all(21);
            for (size_t i = 0; i < std::min(fx.size(), fy.size()); ++i) fit.emplace_back(fx[i], fy[i]);
            return fit;   // ponytail: fit points joined by lines; interpolate if fit-only splines matter
        }
        const size_t n = P.size();
        if (int(n) <= p) return P;   // too few control points for the degree: the control polygon
        if (w.size() != n) w.assign(n, 1.);
        if (U.size() != n + p + 1) {   // missing / bad knots: clamped uniform
            U.clear();
            for (size_t i = 0; i < n + p + 1; ++i) U.push_back(double(std::clamp<long>(long(i) - p, 0, long(n) - p)));
        }
        const double u0 = U[p], u1 = U[n];
        double poly = 0;
        for (size_t i = 1; i < n; ++i) poly += (P[i] - P[i - 1]).norm();
        // ponytail: uniform parameter sampling sized by the control polygon; adaptive if needed.
        const int samples = std::clamp(int(poly / 0.2), int(n) * 8, 20000);
        std::vector<Vec2d> out;
        for (int s = 0; s <= samples; ++s) {
            const double u = u0 + (u1 - u0) * s / samples;
            size_t k = p;
            while (k + 1 < n && U[k + 1] <= u) ++k;
            std::vector<Eigen::Vector3d> d(p + 1);
            for (int j = 0; j <= p; ++j) {
                const size_t idx = k - p + j;
                d[j] = Eigen::Vector3d(P[idx].x() * w[idx], P[idx].y() * w[idx], w[idx]);
            }
            for (int r = 1; r <= p; ++r)
                for (int j = p; j >= r; --j) {
                    const size_t i     = k - p + j;
                    const double denom = U[i + p - r + 1] - U[i];
                    const double a     = denom > 0 ? (u - U[i]) / denom : 0;
                    d[j] = (1 - a) * d[j - 1] + a * d[j];
                }
            out.emplace_back(d[p].x() / d[p].z(), d[p].y() / d[p].z());
        }
        return out;
    }

    // MTEXT: \P = new line; strip inline format codes (\f...; \H...; braces, \~ etc).
    static std::string mtext(const std::string& s)
    {
        std::string out;
        for (size_t i = 0; i < s.size(); ++i) {
            if (s[i] == '{' || s[i] == '}') continue;
            if (s[i] == '\\' && i + 1 < s.size()) {
                const char c = s[i + 1];
                if (c == 'P') { out += '\n'; ++i; continue; }
                if (c == '\\' || c == '{' || c == '}') { out += c; ++i; continue; }
                if (c == '~') { out += ' '; ++i; continue; }
                const size_t semi = s.find(';', i);
                if (std::string("fFHhCcTtQqWwAaSp").find(c) != std::string::npos && semi != std::string::npos) { i = semi; continue; }
                ++i;   // \L \l \O \o \K \k toggles
                continue;
            }
            out += s[i];
        }
        return out;
    }
};

} // namespace

ImportResult import_dxf(const std::string& path)
{
    std::string text;
    if (!read_file(path, text)) {
        ImportResult r;
        r.error = "The DXF file could not be opened.";
        return r;
    }
    DxfReader rd;
    if (text.compare(0, 18, "AutoCAD Binary DXF") == 0 || !rd.parse(text)) {
        rd.res.error = "Only ASCII DXF files can be imported; save the drawing as ASCII DXF.";
        return std::move(rd.res);
    }
    // Drawing units -> mm about the origin; INSERT transforms and text heights follow.
    Transform2d T = Transform2d::Identity();
    T.scale(rd.units);
    rd.emit(rd.entities, T, 7, -1, 0);
    for (const auto& [type, n] : rd.unsupported)
        rd.res.warnings.push_back(std::to_string(n) + " DXF " + type + " entit" + (n == 1 ? "y was" : "ies were") + " skipped (not supported).");
    bool any = false;
    for (size_t i = 0; i < rd.res.shapes.size(); ++i) any |= rd.res.shapes[i].type != ShapeType::Group;
    if (!any) rd.res.error = "The DXF file has no entities that can be imported.";
    return std::move(rd.res);
}

// ---- Bitmaps -----------------------------------------------------------------------------------

ImportResult import_image(const std::string& path, double dpi)
{
    ImportResult res;
    std::string bytes;
    LaserShape  s;
    double      downscale = 1;
    if (!read_file(path, bytes) || !decode_gray(bytes, s, &downscale)) {
        res.error = "The image could not be read. Supported formats: PNG, JPEG, BMP, TIFF.";
        return res;
    }
    if (dpi <= 0) dpi = 254;
    s.width_mm  = s.image_w / downscale * 25.4 / dpi;   // the size of the original pixels
    s.height_mm = s.image_h / downscale * 25.4 / dpi;
    if (downscale < 1) res.warnings.push_back("The image is very large; it was reduced to 36 megapixels.");
    s.xform.translation() = Vec2d(s.width_mm / 2, s.height_mm / 2);   // lower-left corner at the origin
    s.name = boost::filesystem::path(path).filename().string();
    res.shapes.push_back(std::move(s));
    return res;
}

// ---- LightBurn .lbrn2 --------------------------------------------------------------------------

namespace {

namespace pt = boost::property_tree;

// LightBurn stores most settings as <name Value="..."/>.
std::string value_of(const pt::ptree& node, const std::string& child)
{
    if (auto c = node.get_child_optional(child)) return c->get<std::string>("<xmlattr>.Value", c->data());
    return {};
}
double dvalue(const pt::ptree& node, const std::string& child, double def)
{
    const std::string v = value_of(node, child);
    try { return v.empty() ? def : std::stod(v); } catch (...) { return def; }
}
bool bvalue(const pt::ptree& node, const std::string& child, bool def)
{
    const std::string v = boost::algorithm::to_lower_copy(value_of(node, child));
    return v.empty() ? def : (v == "1" || v == "true");
}

Transform2d parse_xform(const std::string& s)
{
    std::istringstream in(s);
    double m[6] = {1, 0, 0, 1, 0, 0};
    for (double& v : m) in >> v;
    Transform2d t = Transform2d::Identity();
    t.linear() << m[0], m[2], m[1], m[3];
    t.translation() = Vec2d(m[4], m[5]);
    return t;
}

struct LbVert { Vec2d p{0, 0}; std::optional<Vec2d> c0, c1; };

// "V1 2c0x1c0y2c1x3c1y4V..." -> vertices with optional control points.
std::vector<LbVert> parse_verts(const std::string& s)
{
    std::vector<LbVert> out;
    size_t i = 0;
    auto number = [&](double& v) {
        while (i < s.size() && std::isspace((unsigned char) s[i])) ++i;
        const char* b = s.c_str() + i;
        char*       e = nullptr;
        v = std::strtod(b, &e);
        i += size_t(e - b);
    };
    while (i < s.size()) {
        const char c = s[i];
        if (c == 'V') {
            ++i;
            LbVert v;
            number(v.p.x());
            number(v.p.y());
            out.push_back(v);
        } else if (c == 'c' && i + 2 < s.size() && !out.empty()) {
            const int  which = s[i + 1] - '0';
            const char axis  = s[i + 2];
            i += 3;
            double v = 0;
            number(v);
            std::optional<Vec2d>& cp = which == 0 ? out.back().c0 : out.back().c1;
            if (!cp) cp = out.back().p;
            (axis == 'x' ? cp->x() : cp->y()) = v;
        } else
            ++i;
    }
    return out;
}

class LbrnReader {
public:
    ImportResult res;

    void cut_setting(const pt::ptree& n, bool image)
    {
        LaserLayer l;
        const int idx = int(dvalue(n, "index", -1));
        if (idx < 0 || idx >= kLayerCount) return;
        l.name        = value_of(n, "name");
        l.speed_mm_s  = dvalue(n, "speed", l.speed_mm_s);
        l.power_max   = dvalue(n, "maxPower", l.power_max);
        l.power_min   = dvalue(n, "minPower", l.power_min);
        l.passes      = std::max(1, int(dvalue(n, "numPasses", l.passes)));
        l.priority    = int(dvalue(n, "priority", l.priority));
        l.interval_mm = dvalue(n, "interval", l.interval_mm);
        l.angle_deg   = dvalue(n, "angle", l.angle_deg);
        l.crosshatch  = bvalue(n, "crossHatch", l.crosshatch);
        l.bidirectional = bvalue(n, "bidir", l.bidirectional);
        l.output      = bvalue(n, "doOutput", l.output);
        l.visible     = !bvalue(n, "hide", false);
        l.air_assist  = bvalue(n, "airAssist", l.air_assist);
        l.kerf_offset_mm = dvalue(n, "kerf", l.kerf_offset_mm);
        l.z_offset_mm = dvalue(n, "zOffset", l.z_offset_mm);
        l.z_step_per_pass = dvalue(n, "zPerPass", l.z_step_per_pass);
        if (auto os = value_of(n, "overscanning"); !os.empty()) l.overscan_mm = dvalue(n, "overscanning", 0);
        std::string type = boost::algorithm::to_lower_copy(n.get<std::string>("<xmlattr>.type", value_of(n, "type")));
        if (type == "scan") l.mode = LayerMode::Fill;
        else if (type == "scan+cut") l.mode = LayerMode::FillLine;
        else if (type == "offset fill") l.mode = LayerMode::OffsetFill;
        if (image) {
            l.image_dpi = dvalue(n, "dpi", l.image_dpi);
            const std::string d = boost::algorithm::to_lower_copy(value_of(n, "ditherMode"));
            static const std::map<std::string, DitherMode> modes{
                {"threshold", DitherMode::Threshold}, {"ordered", DitherMode::Ordered},   {"dither", DitherMode::FloydSteinberg},
                {"jarvis", DitherMode::Jarvis},       {"stucki", DitherMode::Stucki},     {"atkinson", DitherMode::Atkinson},
                {"newsprint", DitherMode::Newsprint}, {"halftone", DitherMode::Halftone}, {"grayscale", DitherMode::Grayscale}};
            if (auto it = modes.find(d); it != modes.end()) l.image_dither = it->second;
            l.image_negative     = bvalue(n, "negativeImage", l.image_negative);
            l.image_pass_through = bvalue(n, "passThrough", l.image_pass_through);
            l.image_cells_per_inch   = dvalue(n, "cellsPerInch", l.image_cells_per_inch);
            l.image_screen_angle_deg = dvalue(n, "halftoneAngle", l.image_screen_angle_deg);
        }
        for (auto& cs : res.cut_settings)
            if (cs.first == idx) { cs.second = l; return; }
        res.cut_settings.emplace_back(idx, l);
    }

    void shape(const pt::ptree& n, const Transform2d& parent_xf, int parent)
    {
        const std::string type = n.get<std::string>("<xmlattr>.Type", "");
        const Transform2d xf   = parent_xf * parse_xform(n.get<std::string>("XForm", ""));
        LaserShape s;
        s.layer  = std::clamp(n.get<int>("<xmlattr>.CutIndex", 0), 0, kLayerCount - 1);
        s.parent = parent;
        s.xform  = xf;
        if (type == "Rect") {
            s.type = ShapeType::Rect;
            s.width = n.get<double>("<xmlattr>.W", 10);
            s.height = n.get<double>("<xmlattr>.H", 10);
            s.corner_radius = n.get<double>("<xmlattr>.Cr", 0);
        } else if (type == "Ellipse") {
            s.type = ShapeType::Ellipse;
            s.rx = n.get<double>("<xmlattr>.Rx", 5);
            s.ry = n.get<double>("<xmlattr>.Ry", 5);
        } else if (type == "Polygon") {
            s.type = ShapeType::Polygon;
            s.rx = s.ry = n.get<double>("<xmlattr>.Rx", n.get<double>("<xmlattr>.R", 5));
            s.sides = n.get<int>("<xmlattr>.N", 6);
        } else if (type == "Path") {
            s.type = ShapeType::Path;
            s.paths = path(n);
            if (s.paths.empty()) return;
        } else if (type == "Text") {
            s.type = ShapeType::Text;
            s.text = n.get<std::string>("<xmlattr>.Str", "");
            std::string font = n.get<std::string>("<xmlattr>.Font", "");
            s.font = font.substr(0, font.find(','));
            s.text_height_mm = n.get<double>("<xmlattr>.H", 10);
            s.text_spacing_mm = n.get<double>("<xmlattr>.LS", 0);
            s.bold = n.get<int>("<xmlattr>.Bold", 0) != 0;
            s.italic = n.get<int>("<xmlattr>.Italic", 0) != 0;
            const int ah = n.get<int>("<xmlattr>.Ah", 0);
            s.text_align = ah == 1 ? TextAlign::Center : ah == 2 ? TextAlign::Right : TextAlign::Left;
            s.name = s.text;
            if (!update_text_outlines(s)) res.warnings.push_back("The font of text \"" + s.text + "\" was not found.");
        } else if (type == "Bitmap") {
            std::string data = n.get<std::string>("<xmlattr>.Data", "");
            boost::algorithm::erase_all(data, "\n");
            boost::algorithm::erase_all(data, "\r");
            std::string bytes(boost::beast::detail::base64::decoded_size(data.size()), '\0');
            const auto r = boost::beast::detail::base64::decode(&bytes[0], data.data(), data.size());
            bytes.resize(r.first);
            if (!decode_gray(bytes, s)) {
                res.warnings.push_back("An embedded bitmap could not be decoded and was skipped.");
                return;
            }
            s.width_mm  = n.get<double>("<xmlattr>.W", s.image_w * 0.1);
            s.height_mm = n.get<double>("<xmlattr>.H", s.image_h * 0.1);
        } else if (type == "Group") {
            s.type = ShapeType::Group;
            res.shapes.push_back(s);
            const int gi = int(res.shapes.size()) - 1;
            if (auto ch = n.get_child_optional("Children"))
                for (const auto& [name, c] : *ch)
                    if (name == "Shape") shape(c, xf, gi);
            return;
        } else {
            res.warnings.push_back("A LightBurn \"" + type + "\" shape was skipped (not supported).");
            return;
        }
        res.shapes.push_back(std::move(s));
    }

    LaserPaths path(const pt::ptree& n)
    {
        const std::vector<LbVert> v = parse_verts(n.get<std::string>("VertList", ""));
        const std::string prims = boost::algorithm::trim_copy(n.get<std::string>("PrimList", ""));
        struct Prim { char kind; int a, b; };
        std::vector<Prim> pl;
        if (prims.rfind("LineClosed", 0) == 0 || prims.rfind("LineOpen", 0) == 0) {
            for (int k = 0; k + 1 < int(v.size()); ++k) pl.push_back({'L', k, k + 1});
            if (prims[4] == 'C' && v.size() > 2) pl.push_back({'L', int(v.size()) - 1, 0});
        } else {
            std::istringstream in(prims);
            char k;
            int  a, b;
            while (in >> k >> a >> b) pl.push_back({k, a, b});
        }
        LaserPaths out;
        std::vector<Vec2d> cur;
        int first = -1, last = -1;
        auto flush = [&](bool closed) {
            if (cur.size() >= 2) out.push_back(make_path(cur, closed));
            cur.clear();
        };
        for (const Prim& p : pl) {
            if (p.a < 0 || p.b < 0 || p.a >= int(v.size()) || p.b >= int(v.size())) continue;
            if (p.a != last) {   // a new sub-path
                flush(false);
                cur.push_back(v[p.a].p);
                first = p.a;
            }
            if (p.kind == 'B')
                flatten_cubic(cur, v[p.a].p, v[p.a].c0.value_or(v[p.a].p), v[p.b].c1.value_or(v[p.b].p), v[p.b].p);
            else
                cur.push_back(v[p.b].p);
            last = p.b;
            if (last == first) { flush(true); last = -1; }
        }
        flush(false);
        return out;
    }
};

} // namespace

ImportResult import_lbrn2(const std::string& path)
{
    LbrnReader  rd;
    std::string xml;
    if (!read_file(path, xml)) {
        rd.res.error = "The LightBurn file could not be opened.";
        return std::move(rd.res);
    }
    pt::ptree tree;
    try {
        std::istringstream in(xml);
        pt::read_xml(in, tree);
    } catch (...) {
        rd.res.error = "The LightBurn file is not valid XML.";
        return std::move(rd.res);
    }
    auto root = tree.get_child_optional("LightBurnProject");
    if (!root) {
        rd.res.error = "This is not a LightBurn project file.";
        return std::move(rd.res);
    }
    for (const auto& [name, n] : *root) {
        if (name == "CutSetting") rd.cut_setting(n, false);
        else if (name == "CutSetting_Img") rd.cut_setting(n, true);
        else if (name == "Shape") rd.shape(n, Transform2d::Identity(), -1);
    }
    if (rd.res.shapes.empty()) rd.res.error = "The LightBurn file has no shapes that can be imported.";
    return std::move(rd.res);
}

ImportResult import_file(const std::string& path)
{
    const std::string ext = lower_ext(path);
    if (ext == ".svg") return import_svg(path);
    if (ext == ".dxf") return import_dxf(path);
    if (ext == ".lbrn2" || ext == ".lbrn") return import_lbrn2(path);
    if (ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".bmp" || ext == ".tif" || ext == ".tiff") return import_image(path);
    ImportResult r;
    r.error = "This file type cannot be imported. Supported: SVG, DXF, LightBurn (.lbrn2), PNG, JPEG, BMP, TIFF.";
    return r;
}

} // namespace Slic3r::Laser
