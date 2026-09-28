#include "libslic3r/Laser/VectorOps.hpp"

#include "libslic3r/ClipperUtils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

namespace Slic3r::Laser {

namespace {

constexpr double kArcTolMm = 0.01;
// Finest fill interval (the Cut Settings minimum): a corrupt or hand-edited 1e-9 would otherwise
// ask for billions of scan lines or offset rings.
constexpr double kMinIntervalMm = 0.01;

Polygons closed_polygons(const LaserPaths& paths)
{
    Polygons out;
    for (const LaserPath& p : paths)
        if (p.closed && p.pts.size() >= 3) out.emplace_back(p.pts.points);
    return out;
}

ClipperLib::JoinType join_of(CornerStyle c)
{
    return c == CornerStyle::Round ? ClipperLib::jtRound : c == CornerStyle::Miter ? ClipperLib::jtMiter : ClipperLib::jtSquare;
}
// Miter limit, or the arc tolerance (scaled) for round joins (ClipperUtils convention).
double limit_of(CornerStyle c) { return c == CornerStyle::Round ? scale_(kArcTolMm) : 3.; }

// Point at arc length s (0..L) along a closed polyline (cum[i] = length up to vertex i).
Vec2d point_at(const Points& pts, const std::vector<double>& cum, double s)
{
    const size_t n = pts.size();
    size_t i = std::upper_bound(cum.begin(), cum.end(), s) - cum.begin();
    i = std::clamp<size_t>(i, 1, n);
    const Vec2d a = unscale(pts[i - 1]), b = unscale(pts[i % n]);
    const double seg = cum[i] - cum[i - 1];
    return seg > 0 ? a + (b - a) * ((s - cum[i - 1]) / seg) : a;
}

// The open stretch of a closed polyline between arc lengths 0 <= a < b < 2L (wraps past the start).
Polyline extract(const Points& pts, const std::vector<double>& cum, double a, double b)
{
    const size_t n = pts.size();
    const double L = cum[n];
    Polyline out;
    out.points.push_back(Point::new_scale(point_at(pts, cum, a)));
    for (int lap = 0; lap < 2; ++lap)
        for (size_t i = 0; i < n; ++i) {
            const double s = cum[i] + lap * L;
            if (s > a && s < b) out.points.push_back(pts[i]);
        }
    out.points.push_back(Point::new_scale(point_at(pts, cum, b > L ? b - L : b)));
    return out;
}

std::vector<double> cumulative(const Points& pts)
{
    std::vector<double> cum(pts.size() + 1, 0);
    for (size_t i = 1; i <= pts.size(); ++i) cum[i] = cum[i - 1] + (unscale(pts[i % pts.size()]) - unscale(pts[i - 1])).norm();
    return cum;
}

// Number of other closed paths that contain the path's first point (0 = outermost).
std::vector<int> containment_depth(const LaserPaths& paths, std::vector<std::vector<int>>* inside = nullptr)
{
    const int n = int(paths.size());
    std::vector<Polygon>     polys(n);
    std::vector<BoundingBox> boxes(n);
    for (int i = 0; i < n; ++i)
        if (paths[i].closed && paths[i].pts.size() >= 3) {
            polys[i] = Polygon(paths[i].pts.points);
            boxes[i] = get_extents(polys[i]);
        }
    std::vector<int> depth(n, 0);
    if (inside) inside->assign(n, {});
    // ponytail: O(n^2) containment, fine for hundreds of paths; an AABB tree if jobs get huge.
    for (int j = 0; j < n; ++j) {
        if (paths[j].pts.empty()) continue;
        const Point& q = paths[j].pts.points.front();
        for (int i = 0; i < n; ++i)
            if (i != j && !polys[i].empty() && boxes[i].contains(q) && polys[i].contains(q) &&
                // Identical outlines: only the earlier one counts as the container.
                !(polys[i].points == paths[j].pts.points && i > j)) {
                ++depth[j];
                if (inside) (*inside)[i].push_back(j);
            }
    }
    return depth;
}

} // namespace

ExPolygons to_expolygons(const LaserPaths& paths) { return union_ex(closed_polygons(paths), ClipperLib::pftEvenOdd); }

LaserPaths to_paths(const ExPolygons& regions)
{
    LaserPaths out;
    for (const ExPolygon& e : regions) {
        out.push_back({Polyline(e.contour.points), true});
        for (const Polygon& h : e.holes) out.push_back({Polyline(h.points), true});
    }
    return out;
}

LaserPaths offset_paths(const LaserPaths& paths, double distance_mm, OffsetDir dir, CornerStyle corners)
{
    const float d = float(scale_(std::abs(distance_mm)));
    const ExPolygons regions = to_expolygons(paths);
    ExPolygons out;
    if (dir != OffsetDir::Inward) append(out, offset_ex(regions, d, join_of(corners), limit_of(corners)));
    if (dir != OffsetDir::Outward) append(out, offset_ex(regions, -d, join_of(corners), limit_of(corners)));
    // Open paths become the outline around them (both sides, ends rounded or squared).
    Polylines open;
    for (const LaserPath& p : paths)
        if (!p.closed && p.pts.size() >= 2) open.push_back(p.pts);
    if (!open.empty() && d > 0)
        append(out, union_ex(offset(open, d, join_of(corners), limit_of(corners),
                                    corners == CornerStyle::Round ? ClipperLib::etOpenRound : ClipperLib::etOpenSquare)));
    return to_paths(out);
}

LaserPaths boolean_op(const LaserPaths& a, const LaserPaths& b, BooleanOp op)
{
    const ExPolygons ea = to_expolygons(a), eb = to_expolygons(b);
    switch (op) {
    case BooleanOp::Union: return to_paths(union_ex(ea, to_polygons(eb)));
    case BooleanOp::Subtract: return to_paths(diff_ex(ea, eb));
    default: return to_paths(intersection_ex(ea, eb));
    }
}

LaserPaths weld(const std::vector<LaserPaths>& shapes)
{
    ExPolygons all;
    for (const LaserPaths& s : shapes) append(all, to_expolygons(s));
    return to_paths(union_ex(all));
}

namespace {

// One set of parallel scan lines at `angle_deg`, rows ordered along -normal ("top to bottom" at
// angle 0), each row's segments in travel order.
Polylines hatch_set(const ExPolygons& regions, double interval_mm, double angle_deg, bool bidirectional)
{
    Polylines out;
    if (regions.empty() || !(interval_mm > 0)) return out;
    interval_mm = std::max(interval_mm, kMinIntervalMm);
    const double a = angle_deg * M_PI / 180.;
    // Rotate the regions by -a so the lines are horizontal, clip, rotate back.
    ExPolygons rot = regions;
    for (ExPolygon& e : rot) e.rotate(-a);
    const BoundingBox bb = get_extents(rot);
    const coord_t step = coord_t(scale_(interval_mm));
    if (step <= 0) return out;
    const coord_t margin = step;
    Polylines lines;
    // Rows from the top: the first row half an interval below the top edge.
    for (coord_t y = bb.max.y() - step / 2; y > bb.min.y(); y -= step)
        lines.emplace_back(Point(bb.min.x() - margin, y), Point(bb.max.x() + margin, y));
    Polylines clipped = intersection_pl(lines, rot);
    // Group by row (y), top row first, segments left to right.
    std::map<coord_t, Polylines, std::greater<coord_t>> rows;
    for (Polyline& pl : clipped) {
        if (pl.size() < 2) continue;
        if (pl.first_point().x() > pl.last_point().x()) pl.reverse();
        rows[pl.first_point().y()].push_back(std::move(pl));
    }
    bool forward = true;
    for (auto& [y, segs] : rows) {
        std::sort(segs.begin(), segs.end(), [](const Polyline& l, const Polyline& r) { return l.first_point().x() < r.first_point().x(); });
        if (!forward) {
            std::reverse(segs.begin(), segs.end());
            for (Polyline& s : segs) s.reverse();
        }
        for (Polyline& s : segs) {
            s.rotate(a);
            out.push_back(std::move(s));
        }
        if (bidirectional) forward = !forward;
    }
    return out;
}

} // namespace

Polylines hatch_fill(const ExPolygons& regions, double interval_mm, double angle_deg, bool crosshatch, bool bidirectional)
{
    Polylines out = hatch_set(regions, interval_mm, angle_deg, bidirectional);
    if (crosshatch) append(out, hatch_set(regions, interval_mm, angle_deg + 90., bidirectional));
    return out;
}

LaserPaths offset_fill(const ExPolygons& regions, double interval_mm)
{
    LaserPaths rings;
    if (!(interval_mm > 0)) return rings;
    interval_mm = std::max(interval_mm, kMinIntervalMm);
    // First ring half an interval inside the edge, so the burnt line lands on the outline.
    for (double d = interval_mm / 2;; d += interval_mm) {
        const ExPolygons r = offset_ex(regions, -float(scale_(d)), ClipperLib::jtRound, scale_(kArcTolMm));
        if (r.empty()) break;
        append(rings, to_paths(r));
    }
    std::reverse(rings.begin(), rings.end());
    return rings;
}

LaserPaths kerf_offset(const LaserPaths& paths, double kerf_mm)
{
    if (kerf_mm == 0) return paths;
    LaserPaths out = to_paths(offset_ex(to_expolygons(paths), float(scale_(kerf_mm)), ClipperLib::jtMiter, 3.));
    for (const LaserPath& p : paths)
        if (!p.closed) out.push_back(p);
    return out;
}

LaserPaths insert_tabs(const LaserPaths& paths, int count, double spacing_mm, double tab_size_mm)
{
    LaserPaths out;
    for (const LaserPath& p : paths) {
        if (!p.closed || p.pts.size() < 2 || tab_size_mm <= 0) {
            out.push_back(p);
            continue;
        }
        const std::vector<double> cum = cumulative(p.pts.points);
        const double              L   = cum.back();
        const int n = spacing_mm > 0 ? std::max(1, int(std::floor(L / spacing_mm))) : count;
        if (n <= 0 || n * tab_size_mm >= L) {   // no tabs, or they would eat the whole path
            out.push_back(p);
            continue;
        }
        // Tab i centred at (i + 0.5) * L / n; the cut runs between consecutive tabs.
        const double pitch = L / n, half = tab_size_mm / 2;
        for (int i = 0; i < n; ++i) {
            const double a = (i + 0.5) * pitch + half, b = (i + 1.5) * pitch - half;
            out.push_back({extract(p.pts.points, cum, a, b), false});
        }
    }
    return out;
}

LaserPaths add_lead_in(const LaserPaths& paths, double length_mm)
{
    if (length_mm <= 0) return paths;
    const std::vector<int> depth = containment_depth(paths);
    LaserPaths out;
    for (size_t k = 0; k < paths.size(); ++k) {
        const LaserPath& p = paths[k];
        if (!p.closed || p.pts.size() < 3) {
            out.push_back(p);
            continue;
        }
        // The cut starts mid-way along the first edge: a lead-in at a corner could cross the part.
        const Vec2d p0 = unscale(p.pts.points[0]), p1 = unscale(p.pts.points[1]);
        const Vec2d t  = (p1 - p0).normalized(), m = (p0 + p1) / 2;
        // Outward normal of this polygon; a hole (odd depth) is approached from its inside (the scrap).
        Vec2d n = Polygon(p.pts.points).is_counter_clockwise() ? Vec2d(t.y(), -t.x()) : Vec2d(-t.y(), t.x());
        if (depth[k] % 2 == 1) n = -n;
        const double len = std::min(length_mm, (p1 - p0).norm() / 2 * M_SQRT2);   // stays beside the first edge
        LaserPath q;
        q.closed = false;
        q.pts.points.reserve(p.pts.size() + 3);
        q.pts.points.push_back(Point::new_scale(m + (n - t).normalized() * len));   // 45 deg approach
        q.pts.points.push_back(Point::new_scale(m));
        q.pts.points.insert(q.pts.points.end(), p.pts.points.begin() + 1, p.pts.points.end());
        q.pts.points.push_back(p.pts.points.front());
        q.pts.points.push_back(Point::new_scale(m));
        out.push_back(std::move(q));
    }
    return out;
}

Vec2d optimize_order(LaserPaths& paths, const Vec2d& start, const JobSettings::Optimize& opts)
{
    const int n = int(paths.size());
    if (n == 0) return start;
    std::vector<std::vector<int>> inside;
    if (opts.cut_inner_first) containment_depth(paths, &inside);
    std::vector<int> blockers(n, 0);   // paths inside i still to cut
    std::vector<std::vector<int>> containers(n);
    for (int i = 0; i < int(inside.size()); ++i)
        for (int j : inside[i]) {
            ++blockers[i];
            containers[j].push_back(i);
        }

    std::vector<bool> done(n, false);
    LaserPaths        out;
    out.reserve(n);
    Vec2d pos = start;
    bool  have_winding = false, ccw_winding = true;
    for (int k = 0; k < n; ++k) {
        int best = -1, best_vertex = 0;
        bool best_rev = false;
        double best_d = std::numeric_limits<double>::max();
        for (int i = 0; i < n; ++i) {
            if (done[i] || blockers[i] > 0) continue;
            if (!opts.reduce_travel) { best = i; break; }
            const Points& pts = paths[i].pts.points;
            if (pts.empty()) { best = i; break; }
            if (paths[i].closed) {
                for (int v = 0; v < int(pts.size()); ++v) {
                    const double d = (unscale(pts[v]) - pos).squaredNorm();
                    if (d < best_d) { best_d = d; best = i; best_vertex = v; best_rev = false; }
                }
            } else {
                const double d0 = (unscale(pts.front()) - pos).squaredNorm(), d1 = (unscale(pts.back()) - pos).squaredNorm();
                if (d0 < best_d) { best_d = d0; best = i; best_vertex = 0; best_rev = false; }
                if (d1 < best_d) { best_d = d1; best = i; best_vertex = 0; best_rev = true; }
            }
        }
        if (best < 0) {   // containment cycle (identical outlines): take any remaining path
            for (int i = 0; i < n && best < 0; ++i)
                if (!done[i]) best = i;
            best_vertex = 0;
            best_rev    = false;
        }
        LaserPath p = std::move(paths[best]);
        done[best] = true;
        for (int c : containers[best]) --blockers[c];
        if (p.closed && p.pts.size() >= 3) {
            if (best_vertex > 0) std::rotate(p.pts.points.begin(), p.pts.points.begin() + best_vertex, p.pts.points.end());
            if (opts.reduce_direction_changes) {
                // Every closed path runs with the winding of the first one.
                const bool ccw = Polygon(p.pts.points).is_counter_clockwise();
                if (!have_winding) { have_winding = true; ccw_winding = ccw; }
                else if (ccw != ccw_winding) std::reverse(p.pts.points.begin() + 1, p.pts.points.end());
            }
        } else if (best_rev)
            p.pts.reverse();
        if (!p.pts.empty()) pos = unscale(p.closed ? p.pts.points.front() : p.pts.points.back());
        out.push_back(std::move(p));
    }
    // ponytail: nearest neighbour only, no 2-opt pass; add one if travel on big jobs matters.
    paths = std::move(out);
    return pos;
}

} // namespace Slic3r::Laser
