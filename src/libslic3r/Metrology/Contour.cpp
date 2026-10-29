#include "Contour.hpp"

#include "Geometry.hpp"
#include "Image.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <unordered_map>

namespace Slic3r::Metrology {

Field field_from_image(const GrayImage& rectified, double x0, double y0, double res)
{
    Field f(rectified.w, rectified.h, x0, y0, res);
    f.v = rectified.px;
    return f;
}

std::vector<Region> regions_from_loops(std::vector<Polygon2> loops, double min_area)
{
    std::vector<Polygon2> outers, holes;
    for (Polygon2& l : loops) {
        const double a = signed_area(l);
        if (std::abs(a) < min_area || l.size() < 3) continue;
        (a > 0 ? outers : holes).push_back(std::move(l));
    }
    std::vector<Region> out;
    for (Polygon2& o : outers) out.push_back({std::move(o), {}});
    for (Polygon2& hl : holes) {
        int    best = -1;
        double ba   = std::numeric_limits<double>::max();
        for (size_t k = 0; k < out.size(); ++k) {
            const double a = signed_area(out[k].outer);
            if (a < ba && point_in_polygon(out[k].outer, hl.front())) {
                ba   = a;
                best = int(k);
            }
        }
        if (best >= 0) out[size_t(best)].holes.push_back(std::move(hl));
    }
    return out;
}

std::vector<Region> contour_regions(const Field& f, float iso, bool inside_above, double simplify_tol, double min_area)
{
    if (f.w <= 0 || f.h <= 0) return {};
    // Padded node grid: indices -1..w and -1..h, padding is outside.
    const int W = f.w + 2;
    auto inside_val = [&](int i, int j) -> double {
        if (i < 0 || j < 0 || i >= f.w || j >= f.h) return -1.0;   // outside, crossing at the midpoint
        double v = f.at(i, j) - iso;
        v        = inside_above ? v : -v;
        return v == 0 ? -1e-6 : v;   // a sample exactly on the iso level counts as outside
    };
    auto node_pos = [&](int i, int j) { return P2{f.x0 + (i + 0.5) * f.res, f.y0 + (j + 0.5) * f.res}; };
    auto is_in    = [&](int i, int j) { return inside_val(i, j) > 0; };
    // Edge ids: horizontal edge from node (i, j) to (i+1, j) and vertical edge from (i, j) to (i, j+1).
    auto hid = [&](int i, int j) { return (int64_t((j + 1)) * W + (i + 1)) * 2; };
    auto vid = [&](int i, int j) { return (int64_t((j + 1)) * W + (i + 1)) * 2 + 1; };
    auto cross_pt = [&](int i0, int j0, int i1, int j1) {
        const double a = inside_val(i0, j0), b = inside_val(i1, j1);
        double       t = std::abs(a - b) > 1e-12 ? a / (a - b) : 0.5;
        t              = std::min(1.0, std::max(0.0, t));
        return node_pos(i0, j0) + (node_pos(i1, j1) - node_pos(i0, j0)) * t;
    };
    std::unordered_map<int64_t, P2>      pts;
    std::unordered_map<int64_t, int64_t> next;   // start edge -> end edge
    for (int j = -1; j < f.h; ++j)
        for (int i = -1; i < f.w; ++i) {
            const bool c0 = is_in(i, j), c1 = is_in(i + 1, j), c2 = is_in(i + 1, j + 1), c3 = is_in(i, j + 1);
            if (c0 == c1 && c1 == c2 && c2 == c3) continue;
            // Edges: e0 bottom (c0-c1), e1 right (c1-c2), e2 top (c3-c2), e3 left (c0-c3).
            const int64_t e[4] = {hid(i, j), vid(i + 1, j), hid(i, j + 1), vid(i, j)};
            auto ensure = [&](int k) {
                if (pts.count(e[k])) return;
                switch (k) {
                case 0: pts[e[0]] = cross_pt(i, j, i + 1, j); break;
                case 1: pts[e[1]] = cross_pt(i + 1, j, i + 1, j + 1); break;
                case 2: pts[e[2]] = cross_pt(i, j + 1, i + 1, j + 1); break;
                default: pts[e[3]] = cross_pt(i, j, i, j + 1); break;
                }
            };
            const bool st[4] = {c0, c1, c2, c3};
            auto link = [&](int from, int to) {
                ensure(from);
                ensure(to);
                next[e[from]] = e[to];
            };
            // Segment cutting off corner k (edges k-1 and k), inside kept on the left: an inside
            // corner runs from edge k to edge k-1, an outside one the other way.
            auto cut = [&](int k) {
                if (st[k]) link(k, (k + 3) % 4);
                else link((k + 3) % 4, k);
            };
            const int cnt = int(c0) + int(c1) + int(c2) + int(c3);
            if (cnt == 1 || cnt == 3) {
                for (int k = 0; k < 4; ++k)
                    if (st[k] == (cnt == 1)) cut(k);
            } else if (c0 == c2) {
                // Saddle: decide by the cell centre.
                const double centre = inside_val(i, j) + inside_val(i + 1, j) + inside_val(i + 1, j + 1) + inside_val(i, j + 1);
                if ((centre > 0) == c0) {   // c0 and c2 joined through the centre: c1 and c3 cut off
                    cut(1);
                    cut(3);
                } else {
                    cut(0);
                    cut(2);
                }
            } else if (c0 == c1) {
                // Bottom row against top row, crossings on the right (e1) and left (e3) edges.
                if (c0) link(1, 3);
                else link(3, 1);
            } else {
                // Left column against right column, crossings on the bottom (e0) and top (e2) edges.
                if (c0) link(0, 2);
                else link(2, 0);
            }
        }
    std::vector<Polygon2> loops;
    std::unordered_map<int64_t, char> used;
    for (const auto& kv : next) {
        if (used.count(kv.first)) continue;
        Polygon2 loop;
        int64_t  cur = kv.first;
        int      guard = 0;
        while (!used.count(cur) && guard++ < 10000000) {
            used[cur] = 1;
            loop.push_back(pts[cur]);
            auto it = next.find(cur);
            if (it == next.end()) break;
            cur = it->second;
        }
        if (loop.size() >= 3) loops.push_back(simplify_tol > 0 ? simplify_closed(loop, simplify_tol) : loop);
    }
    return regions_from_loops(std::move(loops), min_area);
}

Field rasterize(const std::vector<Region>& regions, double x0, double y0, int w, int h, double res)
{
    Field f(w, h, x0, y0, res, 0.f);
    std::vector<const Polygon2*> loops;
    for (const Region& r : regions) {
        loops.push_back(&r.outer);
        for (const Polygon2& hl : r.holes) loops.push_back(&hl);
    }
    std::vector<double> xs;
    for (int j = 0; j < h; ++j) {
        const double y = y0 + (j + 0.5) * res;
        xs.clear();
        for (const Polygon2* l : loops)
            for (size_t k = 0, n = l->size(); k < n; ++k) {
                const P2& a = (*l)[k];
                const P2& b = (*l)[(k + 1) % n];
                if ((a.y > y) != (b.y > y)) xs.push_back(a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y));
            }
        std::sort(xs.begin(), xs.end());
        for (size_t k = 0; k + 1 < xs.size(); k += 2) {
            const int i0 = std::max(0, int(std::ceil((xs[k] - x0) / res - 0.5)));
            const int i1 = std::min(w - 1, int(std::floor((xs[k + 1] - x0) / res - 0.5)));
            for (int i = i0; i <= i1; ++i) f.at(i, j) = 1.f;
        }
    }
    soften_boundary(f, [&regions](const P2& p) {
        for (const Region& r : regions)
            if (point_in_region(r, p)) return true;
        return false;
    });
    return f;
}

void soften_boundary(Field& f, const std::function<bool(const P2&)>& inside, int ss)
{
    if (f.w <= 0 || f.h <= 0 || ss < 2) return;
    const Field src = f;
    auto bin = [&](int i, int j) { return src.at(std::min(std::max(i, 0), f.w - 1), std::min(std::max(j, 0), f.h - 1)) >= 0.5f; };
    for (int j = 0; j < f.h; ++j)
        for (int i = 0; i < f.w; ++i) {
            const bool c = bin(i, j);
            if (c == bin(i + 1, j) && c == bin(i - 1, j) && c == bin(i, j + 1) && c == bin(i, j - 1)) continue;
            const P2 base{f.x0 + i * f.res, f.y0 + j * f.res};
            int      n = 0;
            for (int b = 0; b < ss; ++b)
                for (int a = 0; a < ss; ++a) n += inside(base + P2{(a + 0.5) / ss, (b + 0.5) / ss} * f.res);
            f.at(i, j) = float(n) / float(ss * ss);
        }
}

std::vector<Region> trace_silhouette(const GrayImage& img, const CameraModel& cam, const P2& lo, const P2& hi, double res,
                                     double threshold, bool part_bright, double simplify_tol, double min_area)
{
    if (!cam.ok || res <= 0 || hi.x <= lo.x || hi.y <= lo.y) return {};
    const int w = int(std::ceil((hi.x - lo.x) / res)), h = int(std::ceil((hi.y - lo.y) / res));
    if (w <= 1 || h <= 1 || double(w) * h > 4.0e7) return {};
    const GrayImage rect = rectify(img, [&cam](const P2& p) { return cam.to_px(p, 0.0); }, lo.x, lo.y, w, h, res);
    if (threshold >= 0) return contour_regions(field_from_image(rect, lo.x, lo.y, res), float(threshold), part_bright, simplify_tol, min_area);
    // Each edge at half its own step: brightness above the background as a fraction of the brightest
    // part nearby, so a dim side wall and a bright top face both meet the background at 0.5.
    const Levels lv  = part_levels(rect, part_bright);
    const double sgn = part_bright ? 1.0 : -1.0;
    const int    r   = std::max(2, int(std::lround(0.6 / res)));
    GrayImage    rel(w, h), tmp(w, h), peak(w, h);
    for (size_t i = 0; i < rel.px.size(); ++i) rel.px[i] = float(sgn * (rect.px[i] - lv.background));
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            float m = rel.at(x, y);
            for (int k = std::max(0, x - r); k <= std::min(w - 1, x + r); ++k) m = std::max(m, rel.at(k, y));
            tmp.at(x, y) = m;
        }
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            float m = tmp.at(x, y);
            for (int k = std::max(0, y - r); k <= std::min(h - 1, y + r); ++k) m = std::max(m, tmp.at(x, k));
            peak.at(x, y) = m;
        }
    const double min_step = 0.25 * std::abs(lv.part - lv.background);
    Field        f(w, h, lo.x, lo.y, res);
    for (size_t i = 0; i < f.v.size(); ++i) f.v[i] = peak.px[i] > min_step ? std::max(0.f, rel.px[i]) / peak.px[i] : 0.f;
    return contour_regions(f, 0.5f, true, simplify_tol, min_area);
}

std::string regions_svg(const std::vector<Region>& regions)
{
    P2 lo, hi;
    bounding_box(regions, lo, hi);
    if (regions.empty()) lo = hi = P2{};
    const double m = 2;
    std::ostringstream s;
    s << std::fixed << std::setprecision(4);
    s << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    s << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << (hi.x - lo.x + 2 * m) << "mm\" height=\"" << (hi.y - lo.y + 2 * m)
      << "mm\" viewBox=\"" << (lo.x - m) << ' ' << (-hi.y - m) << ' ' << (hi.x - lo.x + 2 * m) << ' ' << (hi.y - lo.y + 2 * m) << "\">\n";
    s << "<path fill=\"none\" stroke=\"#000000\" stroke-width=\"0.1\" fill-rule=\"evenodd\" d=\"";
    auto loop = [&](const Polygon2& l) {
        for (size_t k = 0; k < l.size(); ++k) s << (k == 0 ? "M" : "L") << l[k].x << ',' << -l[k].y << ' ';
        s << "Z ";
    };
    for (const Region& r : regions) {
        loop(r.outer);
        for (const Polygon2& hl : r.holes) loop(hl);
    }
    s << "\"/>\n</svg>\n";
    return s.str();
}

} // namespace Slic3r::Metrology
