#include <catch2/catch_all.hpp>

#include "libslic3r/CAM/Adaptive.hpp"
#include "libslic3r/ClipperUtils.hpp"

#include <chrono>
#include <cmath>

using namespace Slic3r;
namespace A = Slic3r::CAM::Adaptive;

namespace {

Polygon rect(double x0, double y0, double x1, double y1)
{
    return Polygon::new_scale({{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}});
}

Polygon circle(double cx, double cy, double r, int n = 64)
{
    std::vector<Vec2d> pts;
    for (int i = 0; i < n; ++i)
        pts.emplace_back(cx + r * cos(2 * M_PI * i / n), cy + r * sin(2 * M_PI * i / n));
    return Polygon::new_scale(pts);
}

// 100 x 60 mm pocket, one round island (+ one square island for the benchmark).
ExPolygons region() { return {ExPolygon(rect(0, 0, 100, 60))}; }
ExPolygons islands(bool two)
{
    ExPolygons out{ExPolygon(circle(30, 30, 8))};
    if (two)
        out.emplace_back(rect(65, 22, 80, 37));
    return out;
}

bool inside(const ExPolygons& ex, const Point& p)
{
    for (const ExPolygon& e : ex)
        if (e.contains(p))
            return true;
    return false;
}

// Material simulation: marks cells swept by the tool, returns newly removed area (mm^2).
struct Raster
{
    double               x0, y0, res;
    int                  nx, ny;
    std::vector<uint8_t> cut;

    Raster(double x0_, double y0_, double x1, double y1, double res_) : x0(x0_), y0(y0_), res(res_)
    {
        nx = int((x1 - x0) / res) + 1;
        ny = int((y1 - y0) / res) + 1;
        cut.assign(size_t(nx) * ny, 0);
    }
    Vec2d center(int i, int j) const { return {x0 + (i + 0.5) * res, y0 + (j + 0.5) * res}; }

    double sweep(const Vec2d& a, const Vec2d& b, double r)
    {
        const Vec2d  ab = b - a;
        const double l2 = ab.squaredNorm();
        const int    i0 = std::max(0, int((std::min(a.x(), b.x()) - r - x0) / res));
        const int    i1 = std::min(nx - 1, int((std::max(a.x(), b.x()) + r - x0) / res) + 1);
        const int    j0 = std::max(0, int((std::min(a.y(), b.y()) - r - y0) / res));
        const int    j1 = std::min(ny - 1, int((std::max(a.y(), b.y()) + r - y0) / res) + 1);
        size_t       n  = 0;
        for (int j = j0; j <= j1; ++j)
            for (int i = i0; i <= i1; ++i) {
                uint8_t& c = cut[size_t(j) * nx + i];
                if (c)
                    continue;
                const Vec2d  p = center(i, j);
                const double t = l2 > 0 ? std::clamp((p - a).dot(ab) / l2, 0., 1.) : 0.;
                if ((p - (a + t * ab)).squaredNorm() <= r * r) {
                    c = 1;
                    ++n;
                }
            }
        return double(n) * res * res;
    }
};

Vec2d mm(const Point& p) { return unscale(p); }

double signed_area(const Polyline& pl)
{
    double a = 0;
    for (size_t i = 0; i < pl.points.size(); ++i) {
        const Vec2d p = mm(pl.points[i]), q = mm(pl.points[(i + 1) % pl.points.size()]);
        a += p.x() * q.y() - q.x() * p.y();
    }
    return a / 2;
}

} // namespace

TEST_CASE("Adaptive clearing keeps the tool inside the region and out of the keep-out", "[CamAdaptive]")
{
    A::Params p;
    const double r   = p.tool_diameter / 2;
    A::Result    res = A::clear(region(), islands(false), p);
    REQUIRE(res.ok);
    REQUIRE(res.error.empty());
    REQUIRE_FALSE(res.paths.empty());
    REQUIRE_FALSE(res.cleared.empty());

    // The tool center must stay >= r - eps from every wall: region shrunk by r, keep-out grown by r.
    const double     eps     = 0.02;
    const ExPolygons allowed = offset_ex(diff_ex(region(), islands(false)), float(-scale_(r - eps)), ClipperLib::jtRound);
    // The contract's weaker bounds, checked explicitly as well.
    const ExPolygons region_plus_r  = offset_ex(region(), float(scale_(r)));
    const ExPolygons keep_out_minus = offset_ex(islands(false), float(-scale_(r)));
    size_t           n_cut          = 0;
    for (const A::Path& path : res.paths) {
        if (path.type != A::MotionType::Cutting)
            continue;
        for (const Point& pt : path.pts.points) {
            ++n_cut;
            INFO("point " << unscale<double>(pt.x()) << ", " << unscale<double>(pt.y()));
            REQUIRE(inside(region_plus_r, pt));
            REQUIRE_FALSE(inside(keep_out_minus, pt));
            REQUIRE(inside(allowed, pt));
        }
    }
    REQUIRE(n_cut > 100);

    // deterministic: a second run gives the same program
    A::Result again = A::clear(region(), islands(false), p);
    REQUIRE(again.paths.size() == res.paths.size());
    for (size_t i = 0; i < res.paths.size(); ++i) {
        REQUIRE(again.paths[i].type == res.paths[i].type);
        REQUIRE(again.paths[i].pts.points == res.paths[i].pts.points);
    }
}

TEST_CASE("Adaptive clearing covers the region", "[CamAdaptive]")
{
    A::Params p;
    const double r   = p.tool_diameter / 2;
    A::Result    res = A::clear(region(), islands(false), p);
    REQUIRE(res.ok);

    Raster ras(-5, -5, 105, 65, 0.25);
    for (const A::Path& path : res.paths)
        if (path.type == A::MotionType::Cutting)
            for (size_t i = 1; i < path.pts.points.size(); ++i)
                ras.sweep(mm(path.pts.points[i - 1]), mm(path.pts.points[i]), r);

    const ExPolygons target = diff_ex(region(), islands(false));
    size_t           total = 0, covered = 0;
    for (int j = 0; j < ras.ny; ++j)
        for (int i = 0; i < ras.nx; ++i) {
            const Vec2d c = ras.center(i, j);
            if (!inside(target, Point::new_scale(c.x(), c.y())))
                continue;
            ++total;
            covered += ras.cut[size_t(j) * ras.nx + i];
        }
    const double coverage = double(covered) / double(total);
    INFO("coverage " << coverage);
    REQUIRE(coverage >= 0.97);
}

TEST_CASE("Adaptive clearing keeps the engagement near the optimal load", "[CamAdaptive]")
{
    A::Params p;
    const double r        = p.tool_diameter / 2;
    const double stepover = p.stepover_fraction * p.tool_diameter;
    A::Result    res      = A::clear(region(), islands(false), p);
    REQUIRE(res.ok);

    // Replay the program on a material raster; material removed per unit of cut length is the
    // radial engagement. Checked over ~3 mm windows (a single step is below raster resolution).
    Raster       ras(-5, -5, 105, 65, 0.1);
    const double window = 3.0;
    double       worst  = 0;
    size_t       n_windows = 0;
    size_t       entry     = 0;
    for (size_t k = 0; k < res.paths.size(); ++k) {
        const A::Path& path = res.paths[k];
        while (entry < res.entries.size() && res.entries[entry].path_index == k) {
            const A::Entry& e  = res.entries[entry++];
            const double    hr = (mm(e.start) - mm(e.center)).norm();
            if (hr > 0)
                ras.sweep(mm(e.center), mm(e.center), hr + r); // helix bore
        }
        if (path.type == A::MotionType::LinkNotClear)
            continue; // retracted
        double area = 0, len = 0;
        for (size_t i = 1; i < path.pts.points.size(); ++i) {
            const Vec2d a = mm(path.pts.points[i - 1]), b = mm(path.pts.points[i]);
            area += ras.sweep(a, b, r);
            len += (b - a).norm();
            if (len >= window) {
                worst = std::max(worst, area / (len * stepover));
                ++n_windows;
                area = len = 0;
            }
        }
    }
    INFO("worst engagement ratio " << worst << " over " << n_windows << " windows");
    REQUIRE(n_windows > 100);
    REQUIRE(worst <= 1.3);
}

TEST_CASE("Adaptive helix entry is centered inside the region", "[CamAdaptive]")
{
    A::Params p;
    A::Result res = A::clear(region(), islands(false), p);
    REQUIRE(res.ok);
    REQUIRE_FALSE(res.entries.empty());
    REQUIRE(res.helix_radius > 0);
    REQUIRE(res.helix_center == res.entries.front().center);
    const ExPolygons target = diff_ex(region(), islands(false));
    for (const A::Entry& e : res.entries)
        REQUIRE(inside(target, e.center));
}

TEST_CASE("Adaptive clearing reports a region too small for the tool", "[CamAdaptive]")
{
    A::Params p; // 6 mm tool
    A::Result res = A::clear({ExPolygon(rect(0, 0, 5, 5))}, {}, p);
    REQUIRE_FALSE(res.ok);
    REQUIRE_FALSE(res.error.empty());
    REQUIRE(res.paths.empty());
}

TEST_CASE("Conventional adaptive clearing runs opposite to climb", "[CamAdaptive]")
{
    // Largest closed-ish cutting loop = the wall finishing pass. Climb (CW spindle, material on the
    // right) runs a pocket wall counter-clockwise; conventional runs it clockwise.
    const auto wall_loop_area = [](bool climb) {
        A::Params p;
        p.climb   = climb;
        A::Result res = A::clear(region(), {}, p);
        REQUIRE(res.ok);
        double best = 0;
        for (const A::Path& path : res.paths)
            if (path.type == A::MotionType::Cutting) {
                double a = signed_area(path.pts);
                if (std::abs(a) > std::abs(best))
                    best = a;
            }
        return best;
    };
    const double climb        = wall_loop_area(true);
    const double conventional = wall_loop_area(false);
    INFO("climb " << climb << " conventional " << conventional);
    REQUIRE(climb > 1000);
    REQUIRE(conventional < -1000);
}

TEST_CASE("Adaptive clearing of a 100x60 mm pocket with two islands takes under 2 s", "[CamAdaptive]")
{
    A::Params  p;
    const auto t0      = std::chrono::steady_clock::now();
    A::Result  res     = A::clear(region(), islands(true), p);
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    size_t       points  = 0;
    for (const A::Path& path : res.paths)
        points += path.pts.points.size();
    WARN("adaptive benchmark: " << elapsed << " s, " << res.paths.size() << " paths, " << points << " points");
    REQUIRE(res.ok);
    REQUIRE(elapsed < 2.0);
}

TEST_CASE("Adaptive clearing with air outside enters from outside the stock", "[CamAdaptive]")
{
    // 3D-level use: region = stock section, keep_out = model section; no helix needed.
    A::Params p;
    p.outside_is_air  = true;
    p.stock_to_leave  = 0.5;
    const double r    = p.tool_diameter / 2;
    const ExPolygons stock{ExPolygon(rect(0, 0, 60, 40))};
    const ExPolygons model{ExPolygon(circle(30, 20, 10))};
    A::Result        res = A::clear(stock, model, p);
    REQUIRE(res.ok);
    REQUIRE_FALSE(res.entries.empty());
    REQUIRE(res.entries.front().center == res.entries.front().start); // plunge, no helix
    REQUIRE_FALSE(inside(stock, res.entries.front().center));          // ... in the air

    const ExPolygons forbidden = offset_ex(model, float(scale_(r + p.stock_to_leave - 0.02)));
    for (const A::Path& path : res.paths)
        if (path.type == A::MotionType::Cutting)
            for (const Point& pt : path.pts.points)
                REQUIRE_FALSE(inside(forbidden, pt));
}

TEST_CASE("Adaptive rest machining only cuts what the previous tool left", "[CamAdaptive]")
{
    const ExPolygons pocket{ExPolygon(rect(0, 0, 40, 30))};
    A::Params        big;
    big.tool_diameter = 10;
    A::Result first   = A::clear(pocket, {}, big);
    REQUIRE(first.ok);
    REQUIRE_FALSE(first.cleared.empty());

    A::Params small;
    small.tool_diameter = 4;
    A::Result rest      = A::clear(pocket, {}, small, first.cleared);
    A::Result full      = A::clear(pocket, {}, small);
    REQUIRE(rest.ok);
    REQUIRE(full.ok);
    const auto cut_length = [](const A::Result& res) {
        double l = 0;
        for (const A::Path& path : res.paths)
            if (path.type == A::MotionType::Cutting)
                l += unscale<double>(path.pts.length());
        return l;
    };
    INFO("rest " << cut_length(rest) << " mm, full " << cut_length(full) << " mm");
    REQUIRE(cut_length(rest) < 0.5 * cut_length(full));
}

TEST_CASE("Adaptive clearing can be cancelled", "[CamAdaptive]")
{
    A::Params p;
    double    last = -1;
    p.cancel       = [&last](double progress) {
        last = progress;
        return true;
    };
    A::Result res = A::clear(region(), islands(false), p);
    REQUIRE_FALSE(res.ok);
    REQUIRE(res.error == "Cancelled.");
    REQUIRE(last >= 0);
}
