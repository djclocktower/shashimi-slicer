#include "libslic3r/CAM/Op2D.hpp"

#include "libslic3r/CAM/CamGeometry.hpp"
#include "libslic3r/CAM/FeedsSpeeds.hpp"
#include "libslic3r/CAM/Machines.hpp"
#include "libslic3r/CAM/Toolpath.hpp"
#include "libslic3r/ClipperUtils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>

namespace Slic3r::CAM {

namespace {

using K = Move::Kind;

const double kArcTol = scale_(0.005);   // round-join arc tolerance for offsets (scaled)

Vec3d p3(const Point& p, double z) { return {unscale<double>(p.x()), unscale<double>(p.y()), z}; }
Point p2(const Vec3d& v) { return Point(coord_t(scale_(v.x())), coord_t(scale_(v.y()))); }
Vec2d v2(const Point& p) { return {unscale<double>(p.x()), unscale<double>(p.y())}; }

std::string fmt(const char* f, double a, double b = 0)
{
    char buf[256];
    snprintf(buf, sizeof(buf), f, a, b);
    return buf;
}

void add(Toolpath& tp, K kind, const Vec3d& to, double feed, ArcDir arc = ArcDir::None, const Vec3d& center = Vec3d::Zero())
{
    Move m;
    m.kind   = kind;
    m.to     = to;
    m.feed   = kind == K::Rapid ? 0 : feed;
    m.arc    = arc;
    m.center = center;
    m.a_deg  = tp.moves.empty() ? 0 : tp.moves.back().a_deg;
    tp.moves.push_back(m);
}

Vec3d cur(const Toolpath& tp) { return tp.moves.back().to; }

double dist_to_boundary(const ExPolygons& area, const Point& p)
{
    double best = 1e30;
    for (const ExPolygon& e : area)
        best = std::min(best, (e.point_projection(p) - p).cast<double>().norm());
    return unscale<double>(best);
}

bool inside(const ExPolygons& area, const Point& p)
{
    for (const ExPolygon& e : area)
        if (e.contains(p))
            return true;
    return false;
}

// The straight XY segment a-b lies inside `area` (the tool-centre safe area).
bool segment_inside(const ExPolygons& area, const Point& a, const Point& b)
{
    if (area.empty())
        return false;
    if (a == b)
        return inside(area, a);
    double len = 0;
    for (const Polyline& pl : intersection_pl(Polylines{Polyline(a, b)}, area))
        len += pl.length();
    return len >= (b - a).cast<double>().norm() - scale_(0.01);
}

// Rotates a closed loop so it starts at the point of the loop nearest to `near` (inserted when it
// falls inside a segment), or, without `near`, at the middle of its longest segment: a lead arc
// fits there, it would not at a corner.
Points start_loop_at(const Points& pts, const Point* near)
{
    const size_t n = pts.size();
    size_t       seg = 0;
    Point        at  = pts.front();
    double       best = 0;
    bool         found = false;
    for (size_t i = 0; i < n; ++i) {
        const Point& a = pts[i];
        const Point& b = pts[(i + 1) % n];
        Point        q;
        double       score;
        if (near) {
            const Vec2d ab = (b - a).cast<double>();
            const double t = ab.squaredNorm() > 0 ? std::clamp((*near - a).cast<double>().dot(ab) / ab.squaredNorm(), 0., 1.) : 0.;
            q     = a + (ab * t).cast<coord_t>();
            score = -(q - *near).cast<double>().squaredNorm();
        } else {
            q     = a + ((b - a) / 2);
            score = (b - a).cast<double>().squaredNorm();
        }
        if (!found || score > best) { found = true; best = score; seg = i; at = q; }
    }
    Points out;
    out.reserve(n + 1);
    const size_t first = (seg + 1) % n;
    if (at != pts[first]) out.push_back(at);
    for (size_t k = 0; k < n; ++k) out.push_back(pts[(first + k) % n]);
    if (out.back() == out.front()) out.pop_back();
    return out;
}

// Loop points for cutting: material on the right of travel for climb (CW spindle), on the left
// for conventional. `material_left` says where the material is for the loop's own orientation.
Points oriented(const Polygon& poly, bool material_left, bool climb)
{
    Points pts = poly.points;
    if (material_left == climb)
        std::reverse(pts.begin(), pts.end());
    return pts;
}

struct Gen {
    Toolpath&              tp;
    const CamTool&         tool;
    const CamOperation&    op;
    const FeedsSpeeds&     fs;
    const ResolvedHeights& h;
    double                 r;           // tool radius
    double                 short_link;  // XY distance under which links hop at retract height

    Gen(Toolpath& tp_, const CamTool& t, const CamOperation& o, const FeedsSpeeds& f, const ResolvedHeights& h_)
        : tp(tp_), tool(t), op(o), fs(f), h(h_), r(0.5 * t.diameter), short_link(std::max(o.stepover, 0.5 * t.diameter)) {}

    void feed(const Vec3d& to, K kind = K::Feed) { add(tp, kind, to, kind == K::Ramp ? fs.ramp_feed : kind == K::Plunge ? fs.plunge_feed : fs.feed); }

    // Link to `to`: a feed move when the segment stays inside `safe` at the current Z, else the
    // standard retract/rapid/plunge link.
    void link(const Vec3d& to, const ExPolygons* safe = nullptr)
    {
        if (!tp.moves.empty()) {
            const Vec3d c = cur(tp);
            if ((to - c).norm() < 1e-9)
                return;
            if (safe && std::abs(c.z() - to.z()) < 1e-6 && segment_inside(*safe, p2(c), p2(to))) {
                feed(to);
                return;
            }
        }
        append_link_move(tp, to, h, short_link, fs);
    }

    // From the current position (pts[0] at z_from) down to z_to along pts at the ramp angle,
    // ping-ponging on a short path, then back to pts[0] at z_to.
    void ramp_along(const std::vector<Vec2d>& pts, double z_from, double z_to)
    {
        double len = 0;
        for (size_t i = 1; i < pts.size(); ++i)
            len += (pts[i] - pts[i - 1]).norm();
        const double slope = std::tan(std::clamp(op.ramp_angle_deg, 0.5, 45.0) * M_PI / 180.0);
        if (pts.size() < 2 || len < 0.5 * r) {
            feed(Vec3d(pts.front().x(), pts.front().y(), z_to), K::Plunge);
            return;
        }
        std::vector<Vec2d> trail{pts.front()};
        double             remaining = (z_from - z_to) / slope, z = z_from;
        int                i = 0, dir = 1;
        Vec2d              pos = pts.front();
        while (remaining > 1e-9) {
            if (i + dir < 0 || i + dir >= int(pts.size()))
                dir = -dir;
            const Vec2d  next = pts[i + dir];
            const double seg  = (next - pos).norm();
            if (seg >= remaining) {
                pos = pos + (next - pos) * (remaining / seg);
                feed(Vec3d(pos.x(), pos.y(), z_to), K::Ramp);
                trail.push_back(pos);
                break;
            }
            z -= seg * slope;
            feed(Vec3d(next.x(), next.y(), z), K::Ramp);
            trail.push_back(next);
            pos = next;
            i += dir;
            remaining -= seg;
        }
        for (int k = int(trail.size()) - 2; k >= 0; --k)
            feed(Vec3d(trail[k].x(), trail[k].y(), z_to));
    }

    // Helical ramp around `c` (radius rh, starting at c + (rh, 0)) from z_from to z_to in
    // half-turn arcs, then one flat turn. The tool must already be at the start point, z_from.
    void helix(const Vec2d& c, double rh, double z_from, double z_to, double angle_deg)
    {
        const ArcDir dir   = op.climb ? ArcDir::CCW : ArcDir::CW;
        const double pitch = M_PI * rh * std::tan(std::clamp(angle_deg, 0.5, 45.0) * M_PI / 180.0);   // per half turn
        const int    n     = std::max(1, int(std::ceil((z_from - z_to) / pitch - 1e-9)));
        const Vec3d  center(c.x(), c.y(), 0);
        for (int i = 1; i <= n + 2; ++i) {
            const double z    = i <= n ? z_from + (z_to - z_from) * i / n : z_to;
            const double side = (i % 2) ? -1 : 1;
            add(tp, i <= n ? K::Ramp : K::Feed, Vec3d(c.x() + side * rh, c.y(), z), i <= n ? fs.ramp_feed : fs.feed, dir, center);
        }
    }

    // Tangent lead arc ending (lead-in) or starting (lead-out) at p with travel direction t, on
    // the side away from the material. Returns the far end; radius 0 when it does not fit.
    // `ok` tells whether a tool-centre point is allowed.
    double fit_lead(const Vec2d& p, const Vec2d& t, bool material_right, bool in, const std::function<bool(const Vec2d&)>& ok) const
    {
        const Vec2d left(-t.y(), t.x());
        const Vec2d n_air = material_right ? left : Vec2d(-left);
        for (double R = op.lead_in_radius; R >= 0.1; R *= 0.5) {
            const Vec2d c = p + R * n_air;
            bool        fits = true;
            for (int k = 1; k <= 8 && fits; ++k) {
                // quarter arc from p (angle 0) away from it
                const double a  = 0.5 * M_PI * k / 8;
                const Vec2d  q  = c + R * (-n_air * std::cos(a) + (in ? -t : t) * std::sin(a));
                fits            = ok(q);
            }
            if (fits)
                return R;
        }
        return 0;
    }

    // Cuts closed loops at Z with optional tangent lead arcs. `ok` = allowed tool-centre points
    // for leads; `safe` = area where the tool may feed between loops at this Z (or null).
    void cut_loops(std::vector<Points> loops, double z, const std::function<bool(const Vec2d&)>& ok, const ExPolygons* safe,
                   bool leads)
    {
        while (!loops.empty()) {
            // nearest loop next
            const Point here = tp.moves.empty() ? loops.front().front() : p2(cur(tp));
            size_t      best = 0;
            double      bd   = 1e300;
            for (size_t i = 0; i < loops.size(); ++i)
                for (const Point& q : loops[i]) {
                    const double d = (q - here).cast<double>().squaredNorm();
                    if (d < bd) { bd = d; best = i; }
                }
            Points pts = start_loop_at(loops[best], tp.moves.empty() ? nullptr : &here);
            loops.erase(loops.begin() + best);
            if (pts.size() < 2)
                continue;
            const Vec2d p  = v2(pts.front());
            const Vec2d t0 = (v2(pts[1]) - p).normalized();
            const Vec2d t1 = (p - v2(pts.back())).normalized();
            // travel orientation decides the material side: material right for climb
            const bool mat_right = op.climb;
            double     rin = leads && op.lead_in_radius > 0 ? fit_lead(p, t0, mat_right, true, ok) : 0;
            double     rout = leads && op.lead_in_radius > 0 ? fit_lead(p, t1, mat_right, false, ok) : 0;
            const ArcDir dir = mat_right ? ArcDir::CCW : ArcDir::CW;   // turning away from the air side
            if (rin > 0) {
                const Vec2d left(-t0.y(), t0.x());
                const Vec2d c = p + rin * (mat_right ? left : Vec2d(-left));
                const Vec2d s = c - rin * t0;
                link(Vec3d(s.x(), s.y(), z), safe);
                add(tp, K::LeadIn, Vec3d(p.x(), p.y(), z), fs.feed, dir, Vec3d(c.x(), c.y(), 0));
            } else
                link(Vec3d(p.x(), p.y(), z), safe);
            for (size_t i = 1; i < pts.size(); ++i)
                feed(p3(pts[i], z));
            feed(Vec3d(p.x(), p.y(), z));
            if (rout > 0) {
                const Vec2d left(-t1.y(), t1.x());
                const Vec2d c = p + rout * (mat_right ? left : Vec2d(-left));
                const Vec2d e = c + rout * t1;
                add(tp, K::LeadOut, Vec3d(e.x(), e.y(), z), fs.feed, dir, Vec3d(c.x(), c.y(), 0));
            }
        }
    }
};

std::vector<Points> loops_of(const ExPolygons& ex, bool contours, bool holes, bool material_left, bool climb)
{
    std::vector<Points> out;
    for (const ExPolygon& e : ex) {
        if (contours)
            out.push_back(oriented(e.contour, material_left, climb));
        if (holes)
            for (const Polygon& hp : e.holes)
                out.push_back(oriented(hp, material_left, climb));
    }
    return out;
}

// Rough validation shared by the generators; empty = ok.
std::string check_common(const CamTool& tool, const CamOperation& op, const ResolvedHeights& h, bool needs_stepover)
{
    if (tool.diameter <= 0)
        return "The tool has no diameter. Set its diameter in the tool library.";
    if (op.stepdown <= 0)
        return "The stepdown must be greater than zero.";
    if (needs_stepover && (op.stepover <= 0 || op.stepover > tool.diameter + 1e-9))
        return fmt("The stepover must be between 0 and the tool diameter (%g mm).", tool.diameter);
    if (h.bottom >= h.top - 1e-9)
        return "Nothing to cut: the bottom height is at or above the top height.";
    return {};
}

// A point well inside `ex`: shrink until it would vanish, take the smallest piece's centroid.
Point inner_point(const ExPolygon& ex, double step)
{
    ExPolygon cur_ex = ex;
    for (int i = 0; i < 8; ++i) {
        ExPolygons next = offset_ex(cur_ex, -float(scale_(step)), ClipperLib::jtRound, kArcTol);
        if (next.empty())
            break;
        cur_ex = next.front();
    }
    const Point c = cur_ex.contour.centroid();
    return cur_ex.contains(c) ? c : cur_ex.contour.points.front();
}

struct RingNode {
    ExPolygon             ex;
    std::vector<RingNode> kids;
};

void build_rings(RingNode& n, double step, int depth = 0)
{
    if (depth > 2000)
        return;
    for (ExPolygon& e : offset_ex(n.ex, -float(scale_(step)), ClipperLib::jtRound, kArcTol)) {
        n.kids.push_back({std::move(e), {}});
        build_rings(n.kids.back(), step, depth + 1);
    }
}

} // namespace

std::vector<double> z_levels(double top, double bottom, double stepdown)
{
    if (top <= bottom + 1e-9 || stepdown <= 0)
        return {bottom};
    const int n = std::max(1, int(std::ceil((top - bottom) / stepdown - 1e-6)));
    std::vector<double> out;
    for (int i = 1; i <= n; ++i)
        out.push_back(i == n ? bottom : top - (top - bottom) * i / n);
    return out;
}

// ---- Pocket ------------------------------------------------------------------------------------

Toolpath pocket_region(const ExPolygons& region, const CamTool& tool, const CamOperation& op, const FeedsSpeeds& fs,
                       const ResolvedHeights& h)
{
    Toolpath tp;
    if (std::string e = check_common(tool, op, h, true); !e.empty()) {
        tp.error = e;
        return tp;
    }
    Gen          g(tp, tool, op, fs, h);
    const double r   = g.r;
    const double stl = std::max(0., op.stock_to_leave_radial);
    const int    nf  = std::max(0, op.finishing_passes);
    const double fst = nf > 0 ? std::max(0.01, op.finish_stepover) : 0;

    // Tool-centre area at the wall, and the roughing start (leaves the finishing allowance).
    const ExPolygons wall = offset_ex(region, -float(scale_(r + stl)), ClipperLib::jtRound, kArcTol);
    if (wall.empty()) {
        tp.error = fmt("The %g mm tool is larger than the pocket's narrowest region. Use a smaller tool.", tool.diameter);
        return tp;
    }
    for (const ExPolygon& e : region)
        if (offset_ex(e, -float(scale_(r + stl)), ClipperLib::jtRound, kArcTol).empty()) {
            tp.warnings.push_back({-1, Vec3d::Zero(), fmt("Part of the pocket is narrower than the %g mm tool and is left uncut.", tool.diameter)});
            break;
        }
    std::vector<RingNode> roots;
    for (const ExPolygon& e : nf > 0 ? offset_ex(wall, -float(scale_(nf * fst)), ClipperLib::jtRound, kArcTol) : wall) {
        roots.push_back({e, {}});
        build_rings(roots.back(), op.stepover);
    }
    std::vector<ExPolygons> finish;   // outermost last
    for (int j = nf - 1; j >= 0; --j)
        finish.push_back(offset_ex(wall, -float(scale_(j * fst)), ClipperLib::jtRound, kArcTol));

    const double              bottom = h.bottom + std::max(0., op.stock_to_leave_axial);
    const std::vector<double> zs     = z_levels(h.top, bottom, op.stepdown);
    const auto                ok_all = [](const Vec2d&) { return true; };
    const double helix_r0 = 0.5 * (op.helix_diameter > 0 ? op.helix_diameter : 0.9 * tool.diameter);

    // Entry into a leaf (innermost ring) at z from z_prev: helix at its innermost point, else a
    // ramp along its loop, else a plunge.
    auto enter = [&](const RingNode& leaf, const Points& loop, double z_prev, double z) {
        const Point  c  = inner_point(leaf.ex, 0.25 * op.stepover);
        const double rh = std::min(helix_r0, dist_to_boundary(wall, c) - 0.01);
        if (op.entry == EntryType::Helix && rh >= 0.1 * tool.diameter) {
            const Vec2d cc = v2(c);
            g.link(Vec3d(cc.x() + rh, cc.y(), z_prev), &wall);   // the level above is cleared
            g.helix(cc, rh, z_prev, z, op.ramp_angle_deg);
            g.link(p3(loop.front(), z), &wall);
            return;
        }
        g.link(p3(loop.front(), z_prev), &wall);
        if (op.entry == EntryType::Plunge) {
            g.feed(p3(loop.front(), z), K::Plunge);
            return;
        }
        std::vector<Vec2d> pts;
        for (const Point& p : loop)
            pts.push_back(v2(p));
        pts.push_back(pts.front());
        g.ramp_along(pts, z_prev, z);
    };

    // Post-order walk: children (inner rings) first, then this ring.
    std::function<void(const RingNode&, double, double)> cut_node = [&](const RingNode& n, double z_prev, double z) {
        for (const RingNode& k : n.kids)
            cut_node(k, z_prev, z);
        std::vector<Points> loops = loops_of({n.ex}, true, true, false, op.climb);
        if (n.kids.empty()) {
            const Point here = tp.moves.empty() ? loops.front().front() : p2(cur(tp));
            loops.front()    = start_loop_at(loops.front(), tp.moves.empty() ? nullptr : &here);
            enter(n, loops.front(), z_prev, z);
        }
        g.cut_loops(std::move(loops), z, ok_all, &wall, false);
    };

    auto cut_level = [&](const RingNode& root, double z_prev, double z) {
        cut_node(root, z_prev, z);
        for (const ExPolygons& f : finish) {
            // only the finishing rings around this root
            ExPolygons mine;
            for (const ExPolygon& e : f)
                if (inside({e}, root.ex.contour.points.front()) || inside({root.ex}, e.contour.points.front()))
                    mine.push_back(e);
            g.cut_loops(loops_of(mine, true, true, false, op.climb), z, ok_all, &wall, false);
        }
    };

    if (op.ordering == CutOrdering::LevelFirst) {
        double z_prev = h.top;
        for (double z : zs) {
            for (const RingNode& root : roots)
                cut_level(root, z_prev, z);
            z_prev = z;
        }
    } else {
        for (const RingNode& root : roots) {
            double z_prev = h.top;
            for (double z : zs) {
                cut_level(root, z_prev, z);
                z_prev = z;
            }
        }
    }
    append_retract(tp, h.clearance);
    return tp;
}

// ---- Contour -----------------------------------------------------------------------------------

Toolpath contour_region(const ExPolygons& region, const CamTool& tool, const CamOperation& op, const FeedsSpeeds& fs,
                        const ResolvedHeights& h)
{
    Toolpath tp;
    if (std::string e = check_common(tool, op, h, false); !e.empty()) {
        tp.error = e;
        return tp;
    }
    Gen          g(tp, tool, op, fs, h);
    const double stl = std::max(0., op.stock_to_leave_radial);
    const int    nf  = std::max(0, op.finishing_passes);
    const double fst = std::max(0.01, op.finish_stepover);
    const bool   inside_side = op.side == ContourSide::Inside;
    g.short_link = std::max(op.stepover, 3 * op.lead_in_radius + 1);

    // Offset distance from the region edge for a pass `extra` further out.
    const auto pass = [&](double extra) {
        const double d = op.side == ContourSide::On ? extra : g.r + stl + extra;
        return offset_ex(region, float(scale_(inside_side ? -d : d)), ClipperLib::jtRound, kArcTol);
    };
    // Leads must keep the tool centre at least the final offset away from the material.
    const double     keep = (op.side == ContourSide::On ? 0 : g.r + stl) - 0.01;
    const ExPolygons keep_area =
        keep > 0 ? offset_ex(region, float(scale_(inside_side ? -keep : keep)), ClipperLib::jtRound, kArcTol) : ExPolygons{};
    const auto ok = [&](const Vec2d& q) {
        if (keep <= 0)
            return true;
        const bool in = inside(keep_area, p2(Vec3d(q.x(), q.y(), 0)));
        return inside_side ? in : !in;
    };
    // Outside/On: the region is material (on the left of its loops); only its outer contours
    // for Outside. Inside: the offset area is air (material on the right), every loop.
    const auto loops = [&](const ExPolygons& ex) {
        return inside_side ? loops_of(ex, true, true, false, op.climb)
                           : loops_of(ex, true, op.side == ContourSide::On, true, op.climb);
    };
    if (pass(0).empty()) {
        tp.error = fmt("The %g mm tool does not fit inside this shape. Use a smaller tool.", tool.diameter);
        return tp;
    }

    const std::vector<double> zs = z_levels(h.top, h.bottom + std::max(0., op.stock_to_leave_axial), op.stepdown);
    for (double z : zs) {
        g.cut_loops(loops(pass(nf * fst)), z, ok, nullptr, true);
        for (int j = nf - 1; j >= 0; --j)
            g.cut_loops(loops(pass(j * fst)), z, ok, nullptr, true);
    }
    append_retract(tp, h.clearance);
    return tp;
}

// ---- Face --------------------------------------------------------------------------------------

Toolpath face_region(const ExPolygons& outline, const CamTool& tool, const CamOperation& op, const FeedsSpeeds& fs,
                     const ResolvedHeights& h)
{
    Toolpath tp;
    if (std::string e = check_common(tool, op, h, true); !e.empty()) {
        tp.error = e;
        return tp;
    }
    Gen g(tp, tool, op, fs, h);
    // Grown by the tool radius so the passes clear the stock edge and start/end in air.
    const ExPolygons area = offset_ex(outline, float(scale_(g.r)), ClipperLib::jtMiter);
    if (area.empty()) {
        tp.error = "There is no stock outline to face.";
        return tp;
    }
    const BoundingBox bb = get_extents(area);
    const double      y0 = unscale<double>(bb.min.y()), y1 = unscale<double>(bb.max.y());
    const int         n  = std::max(1, int(std::ceil((y1 - y0 - 2 * g.r) / op.stepover - 1e-6)) + 1);
    Polylines         rows;
    for (int i = 0; i < n; ++i) {
        const double y = n == 1 ? 0.5 * (y0 + y1) : y0 + g.r + (y1 - y0 - 2 * g.r) * i / (n - 1);
        const coord_t ys = coord_t(scale_(y));
        for (Polyline& pl : intersection_pl(Polylines{Polyline(Point(bb.min.x() - 10, ys), Point(bb.max.x() + 10, ys))}, area)) {
            // One-way passes: rows advance +Y, so climb (material on the right) travels -X.
            if ((pl.first_point().x() < pl.last_point().x()) == op.climb)
                pl.reverse();
            rows.push_back(std::move(pl));
        }
    }
    g.short_link = 1e9;   // return strokes hop at the retract height, outside the stock
    const double bottom = h.bottom + std::max(0., op.stock_to_leave_axial);
    for (double z : z_levels(h.top, bottom, op.stepdown))
        for (const Polyline& row : rows) {
            g.link(p3(row.first_point(), z));
            for (size_t i = 1; i < row.points.size(); ++i)
                g.feed(p3(row.points[i], z));
        }
    append_retract(tp, h.clearance);
    return tp;
}

// ---- Trace / Engrave / Slot --------------------------------------------------------------------

Toolpath trace_chains(const Polylines& chains, const CamTool& tool, const CamOperation& op, const FeedsSpeeds& fs,
                      const ResolvedHeights& h)
{
    Toolpath tp;
    if (std::string e = check_common(tool, op, h, false); !e.empty()) {
        tp.error = e;
        return tp;
    }
    if (chains.empty()) {
        tp.error = "Select edges or sketch lines to follow.";
        return tp;
    }
    Gen g(tp, tool, op, fs, h);
    const bool ramp = op.type == OpType::Slot ? op.entry != EntryType::Plunge : op.entry == EntryType::Ramp;
    const std::vector<double> zs = z_levels(h.top, h.bottom, op.stepdown);

    std::vector<Polyline> todo(chains.begin(), chains.end());
    while (!todo.empty()) {
        // nearest chain end next (open chains may be reversed)
        const Point here = tp.moves.empty() ? todo.front().first_point() : p2(cur(tp));
        size_t      best = 0;
        bool        rev  = false;
        double      bd   = 1e300;
        for (size_t i = 0; i < todo.size(); ++i) {
            const double d0 = (todo[i].first_point() - here).cast<double>().squaredNorm();
            const double d1 = (todo[i].last_point() - here).cast<double>().squaredNorm();
            if (d0 < bd) { bd = d0; best = i; rev = false; }
            if (d1 < bd) { bd = d1; best = i; rev = true; }
        }
        Polyline pl = std::move(todo[best]);
        todo.erase(todo.begin() + best);
        if (pl.points.size() < 2)
            continue;
        const bool closed = pl.first_point() == pl.last_point();
        if (rev && !closed)
            pl.reverse();
        double z_prev = h.top;
        for (double z : zs) {
            std::vector<Vec2d> pts;
            for (const Point& p : pl.points)
                pts.push_back(v2(p));
            if (ramp) {
                g.link(Vec3d(pts[0].x(), pts[0].y(), z_prev));
                g.ramp_along(pts, z_prev, z);
            } else
                g.link(Vec3d(pts[0].x(), pts[0].y(), z));
            for (size_t i = 1; i < pts.size(); ++i)
                g.feed(Vec3d(pts[i].x(), pts[i].y(), z));
            // open chains alternate direction so the next level starts where this one ended
            if (!closed)
                pl.reverse();
            z_prev = z;
        }
    }
    append_retract(tp, h.clearance);
    return tp;
}

// ---- Adaptive ----------------------------------------------------------------------------------

void append_adaptive_level(Toolpath& tp, const Adaptive::Result& res, const CamOperation& op, const FeedsSpeeds& fs,
                           const ResolvedHeights& h, double z_from, double z)
{
    CamTool dummy;
    Gen     g(tp, dummy, op, fs, h);
    for (size_t ei = 0; ei < res.entries.size(); ++ei) {
        const Adaptive::Entry& e     = res.entries[ei];
        const size_t           first = e.path_index;
        const size_t           last  = ei + 1 < res.entries.size() ? res.entries[ei + 1].path_index : res.paths.size();
        const Vec2d            c = v2(e.center), s = v2(e.start);
        const double           rh = (s - c).norm();
        if (rh > 1e-3) {
            const Vec2d hs(c.x() + rh, c.y());
            g.link(Vec3d(hs.x(), hs.y(), z_from));
            g.helix(c, rh, z_from, z, op.helix_angle_deg);
            g.feed(Vec3d(s.x(), s.y(), z));
        } else {
            g.link(Vec3d(s.x(), s.y(), z_from));
            g.feed(Vec3d(s.x(), s.y(), z), K::Plunge);
        }
        for (size_t pi = first; pi < last; ++pi) {
            const Adaptive::Path& path = res.paths[pi];
            if (path.pts.points.empty())
                continue;
            const Vec3d start = p3(path.pts.points.front(), z);
            switch (path.type) {
            case Adaptive::MotionType::Cutting:
            case Adaptive::MotionType::LinkClear:
                g.feed(start);
                for (size_t i = 1; i < path.pts.points.size(); ++i)
                    g.feed(p3(path.pts.points[i], z));
                break;
            case Adaptive::MotionType::LinkNotClear: {
                const double saved = g.short_link;
                g.short_link = 1e9;   // hop at the retract height
                g.link(p3(path.pts.points.back(), z));
                g.short_link = saved;
                break;
            }
            case Adaptive::MotionType::LinkClearAtPrevPass: {
                const Vec3d c0 = cur(tp);
                g.feed(Vec3d(c0.x(), c0.y(), z_from));
                for (const Point& p : path.pts.points)
                    g.feed(p3(p, z_from));
                g.feed(p3(path.pts.points.back(), z));
                break;
            }
            }
        }
    }
}

static Toolpath adaptive_region(const ExPolygons& region, const CamTool& tool, const CamOperation& op, const FeedsSpeeds& fs,
                                const ResolvedHeights& h)
{
    Toolpath tp;
    if (std::string e = check_common(tool, op, h, false); !e.empty()) {
        tp.error = e;
        return tp;
    }
    Adaptive::Params p;
    p.tool_diameter     = tool.diameter;
    p.stepover_fraction = std::clamp(op.optimal_load / tool.diameter, 0.01, 1.0);
    p.helix_diameter    = op.helix_diameter;
    p.helix_angle_deg   = op.helix_angle_deg;
    p.tolerance         = std::max(0.001, op.tolerance);
    p.stock_to_leave    = std::max(0., op.stock_to_leave_radial);
    p.climb             = op.climb;
    // Every 2D level clears the same region: compute once, replay per level.
    const Adaptive::Result res = Adaptive::clear(region, {}, p);
    if (!res.ok) {
        tp.error = res.error;
        return tp;
    }
    for (const std::string& w : res.warnings)
        tp.warnings.push_back({-1, Vec3d::Zero(), w});
    double z_prev = h.top;
    for (double z : z_levels(h.top, h.bottom + std::max(0., op.stock_to_leave_axial), op.stepdown)) {
        append_adaptive_level(tp, res, op, fs, h, z_prev, z);
        z_prev = z;
    }
    append_retract(tp, h.clearance);
    return tp;
}

// ---- Chamfer -----------------------------------------------------------------------------------

// Chamfer mill / V-bit of included angle a and flat tip radius rt, chamfer width w (measured
// horizontally on the top face), tip_offset o (how far the tip goes below the chamfer bottom):
//   chamfer depth    d_v = w / tan(a/2)
//   tip Z            z   = edge_z - d_v - o
//   tool-axis offset D   = rt + o * tan(a/2)   (the cone touches the wall at the chamfer bottom)
// For a 90 deg tool: D = rt + o, z = edge_z - w - o. The cone meets the top face D + w from the
// axis, so the tool's widest radius must be at least D + w.
static Toolpath chamfer_region(const ExPolygons& region, const Polylines& chains, const CamTool& tool, const CamOperation& op,
                               const FeedsSpeeds& fs, const ResolvedHeights& h)
{
    Toolpath tp;
    if (tool.type != ToolType::ChamferMill && tool.type != ToolType::VBit && tool.type != ToolType::SpotDrill) {
        tp.error = "A chamfer needs a chamfer mill or a V-bit.";
        return tp;
    }
    const double ta = std::tan(0.5 * std::clamp(tool.tip_angle_deg, 10.0, 170.0) * M_PI / 180.0);
    const double w  = std::max(0., op.chamfer_width);
    const double D  = 0.5 * tool.tip_diameter + std::max(0., op.tip_offset) * ta;
    const double z  = h.bottom - w / ta - std::max(0., op.tip_offset);
    if (0.5 * tool.diameter < D + w - 1e-9) {
        tp.error = fmt("The chamfer is too wide for this tool: it needs a tool at least %.2f mm wide.", 2 * (D + w));
        return tp;
    }
    CamOperation one = op;
    one.stepdown     = 1e9;   // single pass
    ResolvedHeights hh = h;
    hh.top    = std::max(h.top, h.bottom);
    hh.bottom = z;
    if (hh.bottom >= hh.top) hh.top = hh.bottom + 1e-3;
    Gen g(tp, tool, one, fs, hh);
    if (!region.empty()) {
        // offset_ex(region, +D): the material (the face) is inside every loop, on its left.
        const ExPolygons path = offset_ex(region, float(scale_(D)), ClipperLib::jtRound, kArcTol);
        const ExPolygons keep = D > 0.02 ? offset_ex(region, float(scale_(D - 0.01)), ClipperLib::jtRound, kArcTol) : region;
        g.cut_loops(loops_of(path, true, true, true, op.climb), z, [&](const Vec2d& q) { return !inside(keep, p2(Vec3d(q.x(), q.y(), 0))); },
                    nullptr, true);
    }
    if (!chains.empty()) {
        Toolpath t2 = trace_chains(chains, tool, one, fs, hh);
        tp.moves.insert(tp.moves.end(), t2.moves.begin(), t2.moves.end());
    }
    if (tp.moves.empty())
        tp.error = "Select the faces or edges to chamfer.";
    append_retract(tp, h.clearance);
    return tp;
}

// ---- Selection + dispatch ----------------------------------------------------------------------

static CamSetupFrame frame_of(const CamDocument& doc, const CamModel& model, int setup)
{
    if (setup >= 0 && setup < int(model.setups.size()))
        return model.setups[setup];
    if (setup >= 0 && setup < int(doc.setups.size()))
        return compute_setup_frame(doc.setups[setup], model);
    return {};
}

// Joins polylines sharing end points (within 10 um) into longer chains.
static Polylines join_chains(Polylines in)
{
    const double tol2 = scale_(0.01) * scale_(0.01);
    auto near = [&](const Point& a, const Point& b) { return (a - b).cast<double>().squaredNorm() <= tol2; };
    Polylines out;
    while (!in.empty()) {
        Polyline cur_pl = std::move(in.back());
        in.pop_back();
        for (bool grew = true; grew && !near(cur_pl.first_point(), cur_pl.last_point());) {
            grew = false;
            for (size_t i = 0; i < in.size(); ++i) {
                Polyline& o = in[i];
                if (near(cur_pl.last_point(), o.last_point()))
                    o.reverse();
                if (near(cur_pl.last_point(), o.first_point())) {
                    cur_pl.points.insert(cur_pl.points.end(), o.points.begin() + 1, o.points.end());
                } else {
                    if (near(cur_pl.first_point(), o.first_point()))
                        o.reverse();
                    if (!near(cur_pl.first_point(), o.last_point()))
                        continue;
                    o.points.insert(o.points.end(), cur_pl.points.begin() + 1, cur_pl.points.end());
                    cur_pl = std::move(o);
                }
                in.erase(in.begin() + i);
                grew = true;
                break;
            }
        }
        if (cur_pl.points.size() > 2 && near(cur_pl.first_point(), cur_pl.last_point()))
            cur_pl.points.back() = cur_pl.points.front();
        out.push_back(std::move(cur_pl));
    }
    return out;
}

Region2D resolve_selection_2d(const CamDocument& doc, const CamOperation& op, const CamModel& model)
{
    Region2D            out;
    const CamSetupFrame frame = frame_of(doc, model, op.setup_index);
    double              zt = -1e30, zb = 1e30;
    auto                zext = [&](double z) { zt = std::max(zt, z); zb = std::min(zb, z); };
    ExPolygons          regions;
    for (const FaceRef& f : op.geom.faces) {
        const CamBody* b = model.body(f.body);
        if (!b || !b->shape) {
            out.error = "A selected face belongs to a body that no longer exists. Reselect the geometry.";
            return out;
        }
        double     z  = 0;
        ExPolygons ex = face_outline(*b->shape, f.face, frame.to_setup, z);
        if (ex.empty()) {
            out.error = "A selected face is not flat or does not face up. Pick flat faces for 2D operations.";
            return out;
        }
        zext(z);
        append(regions, std::move(ex));
    }
    Polylines chains;
    for (const EdgeRef& e : op.geom.edges) {
        const CamBody* b = model.body(e.body);
        if (!b || !b->shape) {
            out.error = "A selected edge belongs to a body that no longer exists. Reselect the geometry.";
            return out;
        }
        double   z  = 0;
        Polyline pl = edge_polyline(*b->shape, e.edge, frame.to_setup, std::max(0.001, op.tolerance), z);
        if (pl.points.size() < 2) {
            out.error = "A selected edge could not be read. Reselect the geometry.";
            return out;
        }
        zext(z);
        chains.push_back(std::move(pl));
    }
    if (op.geom.sketch_feature >= 0) {
        const CamSketch* s = model.sketch(op.geom.sketch_feature);
        if (!s) {
            out.error = "The selected sketch no longer exists. Reselect the geometry.";
            return out;
        }
        const Vec3d o = frame.to_setup * s->origin;
        const Vec3d x = frame.to_setup.linear() * s->x_axis, y = frame.to_setup.linear() * s->y_axis;
        if (std::abs(x.z()) > 1e-6 || std::abs(y.z()) > 1e-6) {
            out.error = "The sketch is not parallel to the setup's XY plane.";
            return out;
        }
        auto map = [&](const Point& p) {
            const Vec3d q = o + x * unscale<double>(p.x()) + y * unscale<double>(p.y());
            return p2(q);
        };
        for (ExPolygon ex : s->regions) {
            for (Point& p : ex.contour.points) p = map(p);
            for (Polygon& hp : ex.holes)
                for (Point& p : hp.points) p = map(p);
            regions.push_back(std::move(ex));
        }
        for (Polyline pl : s->chains) {
            for (Point& p : pl.points) p = map(p);
            chains.push_back(std::move(pl));
        }
        zext(o.z());
    }
    out.regions = union_ex(regions);   // re-orients mirrored sketches too
    out.chains  = join_chains(std::move(chains));
    if (zt < zb)
        zt = zb = frame.model.max.z();
    out.z_top    = zt;
    out.z_bottom = zb;
    if (out.regions.empty() && out.chains.empty())
        out.error = "Select faces, edges or a sketch for this operation.";
    return out;
}

Toolpath generate_2d(const CamDocument& doc, const CamOperation& op, const CamModel& model)
{
    Toolpath       tp;
    const CamTool* tool = doc.find_tool(op.tool_number);
    if (!tool) {
        tp.error = "The operation's tool (T" + std::to_string(op.tool_number) + ") is not in the tool library.";
        return tp;
    }
    if (op.setup_index < 0 || op.setup_index >= int(doc.setups.size())) {
        tp.error = "The operation's setup does not exist.";
        return tp;
    }
    const CamSetup&       setup = doc.setups[op.setup_index];
    const CamSetupFrame   frame = frame_of(doc, model, op.setup_index);
    const MachineProfile& mach  = find_machine(setup.machine);
    const FeedsSpeeds     fs    = effective_feeds(op, *tool, setup.material, mach);
    if (fs.feed <= 0 || fs.plunge_feed <= 0) {
        tp.error = "The feed rates must be greater than zero.";
        return tp;
    }

    if (op.type == OpType::Face) {
        const BoundingBoxf3& s = frame.stock;
        const ExPolygons     outline{ExPolygon(Polygon::new_scale({{s.min.x(), s.min.y()}, {s.max.x(), s.min.y()},
                                                                   {s.max.x(), s.max.y()}, {s.min.x(), s.max.y()}}))};
        return face_region(outline, *tool, op, fs, resolve_heights(op.heights, frame, frame.model.max.z(), frame.model.max.z()));
    }

    Region2D sel = resolve_selection_2d(doc, op, model);
    if (!sel.error.empty()) {
        tp.error = sel.error;
        return tp;
    }
    const ResolvedHeights h = resolve_heights(op.heights, frame, sel.z_top, sel.z_bottom);

    // Closed chains (e.g. a picked loop of edges) act as regions for area/profile ops.
    ExPolygons closed = sel.regions;
    Polylines  open;
    for (const Polyline& pl : sel.chains) {
        if (pl.points.size() > 3 && pl.first_point() == pl.last_point()) {
            Polygon poly(Points(pl.points.begin(), pl.points.end() - 1));
            append(closed, union_ex(Polygons{poly}));
        } else
            open.push_back(pl);
    }

    switch (op.type) {
    case OpType::Pocket2D:
    case OpType::Adaptive2D:
    case OpType::Contour2D:
        if (closed.empty()) {
            tp.error = "Select flat faces, a closed loop of edges or a closed sketch profile.";
            return tp;
        }
        closed = union_ex(closed);
        return op.type == OpType::Pocket2D   ? pocket_region(closed, *tool, op, fs, h)
             : op.type == OpType::Adaptive2D ? adaptive_region(closed, *tool, op, fs, h)
                                             : contour_region(closed, *tool, op, fs, h);
    case OpType::Chamfer2D: return chamfer_region(union_ex(sel.regions), sel.chains, *tool, op, fs, h);
    case OpType::Slot:
    case OpType::Engrave:
    case OpType::Trace: {
        // Faces: follow their outlines.
        Polylines chains = sel.chains;
        for (const ExPolygon& e : sel.regions)
            for (const Polygon& p : to_polygons(e))
                chains.push_back(p.split_at_first_point());
        return trace_chains(chains, *tool, op, fs, h);
    }
    default: tp.error = "This is not a 2D operation."; return tp;
    }
}

} // namespace Slic3r::CAM
