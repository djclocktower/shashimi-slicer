#include "libslic3r/CAM/Op3D.hpp"
#include "libslic3r/CAM/Adaptive.hpp"
#include "libslic3r/CAM/CamInternal.hpp"
#include "libslic3r/CAM/Op2D.hpp"

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/TriangleMeshSlicer.hpp"

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <functional>

namespace Slic3r::CAM {

// ---- Drop-cutter -------------------------------------------------------------------------------

namespace internal {

namespace {

// Max of a concave f over [a, b] (golden section).
template<class F> double max_concave(const F& f, double a, double b)
{
    const double g  = 0.5 * (std::sqrt(5.) - 1.);
    double       best = std::max(f(a), f(b));
    double       c = b - g * (b - a), d = a + g * (b - a), fc = f(c), fd = f(d);
    for (int i = 0; i < 40 && b - a > 1e-6; ++i) {
        if (fc < fd) {
            a = c; c = d; fc = fd; d = a + g * (b - a); fd = f(d);
        } else {
            b = d; d = c; fd = fc; c = b - g * (b - a); fc = f(c);
        }
    }
    return std::max({best, fc, fd});
}

bool inside_triangle_xy(const Vec2d& q, const Vec2d& a, const Vec2d& b, const Vec2d& c)
{
    const auto cr = [](const Vec2d& o, const Vec2d& p, const Vec2d& r) { return (p - o).x() * (r - o).y() - (p - o).y() * (r - o).x(); };
    const double d1 = cr(a, b, q), d2 = cr(b, c, q), d3 = cr(c, a, q);
    const double eps = 1e-12;
    const bool   neg = d1 < -eps || d2 < -eps || d3 < -eps;
    const bool   pos = d1 > eps || d2 > eps || d3 > eps;
    return !(neg && pos);
}

// Edge contact: the chord of the footprint along the edge; tip z = max over the chord of
// z_edge(t) - h(dist(p, edge(t))), a concave function (h convex non-decreasing).
void edge_test(const Vec3d& a, const Vec3d& b, const Vec2d& p, const Cutter& c, double& best)
{
    const Vec2d  u  = b.head<2>() - a.head<2>();
    const double L2 = u.squaredNorm();
    if (L2 < 1e-18)
        return; // vertical edge: its top vertex is tested
    const double tp  = (p - a.head<2>()).dot(u) / L2;
    const double dp2 = (p - (a.head<2>() + tp * u)).squaredNorm();
    const double R2  = c.R * c.R;
    if (dp2 >= R2)
        return;
    const double L    = std::sqrt(L2);
    const double half = std::sqrt(R2 - dp2) / L;
    const double t0 = std::max(0., tp - half), t1 = std::min(1., tp + half);
    if (t0 > t1)
        return;
    const double dz = b.z() - a.z();
    if (!c.cone() && c.rc <= 0) { // flat: z linear along the chord
        best = std::max(best, a.z() + (dz > 0 ? t1 : t0) * dz);
        return;
    }
    if (!c.cone() && c.rc >= c.R - 1e-12) { // ball: circle of radius s in the edge's vertical plane
        const double s = std::sqrt(R2 - dp2), m = dz / L, k = std::sqrt(1 + m * m);
        const double uc = tp * L + s * m / k;
        if (uc >= 0 && uc <= L)
            best = std::max(best, a.z() + m * tp * L + s * k - c.R);
        return;
    }
    const auto f = [&](double t) {
        const Vec2d q = a.head<2>() + t * u;
        return a.z() + t * dz - c.h(std::min(c.R, (q - p).norm()));
    };
    best = std::max(best, max_concave(f, t0, t1));
}

} // namespace

DropCutter::DropCutter(const indexed_triangle_set& its, const Cutter& cutter)
    : m_its(its), m_c(cutter), m_tree(AABBTreeIndirect::build_aabb_tree_over_indexed_triangle_set(its.vertices, its.indices))
{}

double DropCutter::at(double x, double y, double floor) const
{
    double      best = floor;
    const Vec2d p(x, y);
    const float lx = float(x - m_c.R), hx = float(x + m_c.R), ly = float(y - m_c.R), hy = float(y + m_c.R);
    const auto  pred = [&](const AABBTreeIndirect::Tree3f::Node& n) {
        const auto& bb = n.bbox;
        return double(bb.max().z()) > best && bb.min().x() <= hx && bb.max().x() >= lx && bb.min().y() <= hy &&
               bb.max().y() >= ly;
    };
    const double R2 = m_c.R * m_c.R;
    AABBTreeIndirect::traverse(m_tree, pred, [&](const AABBTreeIndirect::Tree3f::Node& n) {
        const Vec3i32& f = m_its.indices[n.idx];
        const Vec3d    v[3]{m_its.vertices[f(0)].cast<double>(), m_its.vertices[f(1)].cast<double>(),
                         m_its.vertices[f(2)].cast<double>()};
        // vertices
        for (const Vec3d& q : v) {
            const double d2 = (q.head<2>() - p).squaredNorm();
            if (d2 <= R2)
                best = std::max(best, q.z() - m_c.h(std::sqrt(d2)));
        }
        // facet
        const Vec3d n3 = (v[1] - v[0]).cross(v[2] - v[0]);
        if (std::abs(n3.z()) > 1e-9 * n3.norm()) {
            const Vec2d  g(-n3.x() / n3.z(), -n3.y() / n3.z());
            const double s = g.norm();
            const double d = m_c.facet_d(s);
            const Vec2d  q = s > 1e-12 ? Vec2d(p + g * (d / s)) : p;
            if (inside_triangle_xy(q, v[0].head<2>(), v[1].head<2>(), v[2].head<2>()))
                best = std::max(best, v[0].z() + g.dot(p - v[0].head<2>()) + s * d - m_c.h(d));
        }
        // edges
        edge_test(v[0], v[1], p, m_c, best);
        edge_test(v[1], v[2], p, m_c, best);
        edge_test(v[2], v[0], p, m_c, best);
        return true;
    });
    return best;
}

std::vector<double> horizontal_face_zs(const indexed_triangle_set& its)
{
    std::vector<double> zs;
    for (const Vec3i32& f : its.indices) {
        const Vec3d a = its.vertices[f(0)].cast<double>(), b = its.vertices[f(1)].cast<double>(), c = its.vertices[f(2)].cast<double>();
        const Vec3d n = (b - a).cross(c - a);
        const double len = n.norm();
        if (len > 0 && n.z() > 0.9999 * len && std::abs(a.z() - b.z()) < 1e-4 && std::abs(a.z() - c.z()) < 1e-4)
            zs.push_back(a.z());
    }
    std::sort(zs.begin(), zs.end(), std::greater<>());
    zs.erase(std::unique(zs.begin(), zs.end(), [](double l, double r) { return std::abs(l - r) < 1e-3; }), zs.end());
    return zs;
}

void sketch_to_setup_xy(const CamSketch& sk, const Transform3d& to_setup, ExPolygons& regions, Polylines& chains)
{
    const auto map = [&](const Point& p) {
        const Vec3d w = sk.origin + unscale<double>(p.x()) * sk.x_axis + unscale<double>(p.y()) * sk.y_axis;
        const Vec3d s = to_setup * w;
        return Point::new_scale(s.x(), s.y());
    };
    const auto map_poly = [&](const Polygon& in) {
        Polygon out;
        for (const Point& p : in.points)
            out.points.push_back(map(p));
        if (out.area() < 0)
            out.reverse(); // a sketch seen from below mirrors: normalise
        return out;
    };
    Polygons contours, holes;
    for (const ExPolygon& e : sk.regions) {
        contours.push_back(map_poly(e.contour));
        for (const Polygon& h : e.holes)
            holes.push_back(map_poly(h));
    }
    regions = diff_ex(contours, holes);
    for (const Polyline& pl : sk.chains) {
        Polyline out;
        for (const Point& p : pl.points)
            out.points.push_back(map(p));
        chains.push_back(std::move(out));
    }
}

ExPolygons silhouette(const indexed_triangle_set& its)
{
    Polygons polys;
    polys.reserve(its.indices.size() / 2);
    for (const Vec3i32& f : its.indices) {
        Polygon poly;
        for (int k = 0; k < 3; ++k)
            poly.points.push_back(Point::new_scale(its.vertices[f(k)].x(), its.vertices[f(k)].y()));
        const double a = poly.area();
        if (a > 0)
            polys.push_back(std::move(poly));
    }
    return union_ex(polys);
}

// Vertical Douglas-Peucker on a pass whose points are collinear in XY: drops points whose Z is
// within `tol` of the chord (so a chord never dips more than tol below a dropped point).
std::vector<Vec3d> simplify_pass(const std::vector<Vec3d>& pts, double tol)
{
    if (pts.size() < 3)
        return pts;
    std::vector<char> keep(pts.size(), 0);
    keep.front() = keep.back() = 1;
    std::vector<std::pair<size_t, size_t>> stack{{0, pts.size() - 1}};
    while (!stack.empty()) {
        auto [i0, i1] = stack.back();
        stack.pop_back();
        const Vec3d& a = pts[i0];
        const Vec3d& b = pts[i1];
        const double L = (b.head<2>() - a.head<2>()).norm();
        double       worst = tol;
        size_t       wi    = 0;
        for (size_t i = i0 + 1; i < i1; ++i) {
            const double t   = L > 0 ? (pts[i].head<2>() - a.head<2>()).norm() / L : 0.;
            const double err = std::abs(pts[i].z() - (a.z() + t * (b.z() - a.z())));
            if (err > worst) {
                worst = err;
                wi    = i;
            }
        }
        if (wi) {
            keep[wi] = 1;
            stack.push_back({i0, wi});
            stack.push_back({wi, i1});
        }
    }
    std::vector<Vec3d> out;
    for (size_t i = 0; i < pts.size(); ++i)
        if (keep[i])
            out.push_back(pts[i]);
    return out;
}

} // namespace internal

HeightMap drop_cutter(const TriangleMesh& mesh, const CamTool& tool, const BoundingBox3Base<Vec3d>& box, double resolution)
{
    HeightMap hm;
    const Vec3d size = box.size();
    hm.resolution    = resolution > 0 ? resolution : std::clamp(std::max(size.x(), size.y()) / 400., 0.05, 0.5);
    hm.x0            = box.min.x();
    hm.y0            = box.min.y();
    hm.nx            = int(std::floor(size.x() / hm.resolution + 1e-9)) + 1;
    hm.ny            = int(std::floor(size.y() / hm.resolution + 1e-9)) + 1;
    hm.z.assign(size_t(hm.nx) * size_t(hm.ny), float(box.min.z()));
    const internal::DropCutter dc(mesh.its, internal::make_cutter(tool));
    tbb::parallel_for(tbb::blocked_range<int>(0, hm.ny), [&](const tbb::blocked_range<int>& r) {
        for (int j = r.begin(); j < r.end(); ++j)
            for (int i = 0; i < hm.nx; ++i)
                hm.z[size_t(j) * hm.nx + i] = float(dc.at(hm.x0 + i * hm.resolution, hm.y0 + j * hm.resolution, box.min.z()));
    });
    return hm;
}

// ---- shared bits of the 3D generators ----------------------------------------------------------

namespace {

using internal::add_move;
using K = Move::Kind;

ExPolygon rect_mm(double x0, double y0, double x1, double y1)
{
    return ExPolygon(Polygon::new_scale({{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}));
}

Vec2d mm(const Point& p) { return unscale(p); }

// top, top - step, ... (> bottom), bottom, plus `extra` inside [bottom, top]; descending, deduplicated.
std::vector<double> levels_with(double top, double bottom, double step, const std::vector<double>& extra, bool include_top)
{
    std::vector<double> zs;
    for (double z = include_top ? top : top - step; z > bottom + 1e-6; z -= step)
        zs.push_back(z);
    zs.push_back(bottom);
    for (double z : extra)
        if (z >= bottom - 1e-9 && z <= top + 1e-9)
            zs.push_back(z);
    std::sort(zs.begin(), zs.end(), std::greater<>());
    zs.erase(std::unique(zs.begin(), zs.end(), [](double l, double r) { return std::abs(l - r) < 1e-3; }), zs.end());
    return zs;
}

// Model sections at `zs`, each the union of every section at or above it (the "shadow" a 3-axis
// tool from above must avoid). Returned in the order of `zs`.
// ponytail: the shadow is only sampled at the requested heights; an overhang that exists purely
// between two of them is missed. Fine for 3-axis parts without undercuts.
std::vector<ExPolygons> shadow_sections(const indexed_triangle_set& its, const std::vector<double>& zs)
{
    std::vector<float> sorted;
    for (double z : zs)
        sorted.push_back(float(z));
    std::sort(sorted.begin(), sorted.end());
    sorted.erase(std::unique(sorted.begin(), sorted.end()), sorted.end());
    std::vector<ExPolygons> secs = slice_mesh_ex(its, sorted);
    for (int k = int(sorted.size()) - 2; k >= 0; --k)
        if (!secs[k + 1].empty())
            secs[k] = union_ex(secs[k], secs[k + 1]);
    std::vector<ExPolygons> out;
    for (double z : zs)
        out.push_back(secs[std::lower_bound(sorted.begin(), sorted.end(), float(z)) - sorted.begin()]);
    return out;
}

bool same_area(const ExPolygons& a, const ExPolygons& b)
{
    const double tol = scale_(1.) * scale_(1.) * 1e-3; // 0.001 mm^2
    return std::abs(area(a) - area(b)) < tol && area(diff_ex(a, b)) < tol && area(diff_ex(b, a)) < tol;
}

void add_warning(Toolpath& tp, const std::string& text)
{
    for (const Warning& w : tp.warnings)
        if (w.text == text)
            return;
    Warning w;
    w.text = text;
    tp.warnings.push_back(w);
}

} // namespace

// ---- Parallel3D --------------------------------------------------------------------------------

Toolpath generate_parallel3d(const CamDocument& doc, const CamOperation& op, const CamModel& model)
{
    Toolpath           tp;
    internal::OpContext ctx = internal::op_context(doc, op, model);
    if (!ctx.error.empty()) {
        tp.error = ctx.error;
        return tp;
    }
    if (op.stepover <= 0) {
        tp.error = "Stepover must be greater than zero.";
        return tp;
    }
    const CamTool&       tool = *ctx.tool;
    const CamSetupFrame& fr   = *ctx.frame;
    const double         R    = tool.diameter / 2;

    // Where the tool centre may go (setup XY).
    ExPolygons boundary;
    if (op.boundary == Boundary::Selection) {
        if (const CamSketch* sk = model.sketch(op.geom.sketch_feature)) {
            Polylines unused;
            internal::sketch_to_setup_xy(*sk, fr.to_setup, boundary, unused);
        }
        if (boundary.empty())
            add_warning(tp, "No closed sketch profile was selected as the boundary; the model outline is used instead.");
    }
    if (op.boundary == Boundary::Stock)
        boundary = {rect_mm(fr.stock.min.x(), fr.stock.min.y(), fr.stock.max.x(), fr.stock.max.y())};
    else if (boundary.empty())
        boundary = offset_ex(internal::silhouette(ctx.mesh.its), float(scale_(R)), ClipperLib::jtRound);
    if (boundary.empty()) {
        tp.error = "The machining boundary is empty.";
        return tp;
    }

    // Work in a frame rotated so the passes run along +X.
    const double ang = op.angle_deg * M_PI / 180.;
    TriangleMesh mesh = ctx.mesh;
    if (ang != 0) {
        mesh.transform(Transform3d(Eigen::AngleAxisd(-ang, Vec3d::UnitZ())));
        for (ExPolygon& e : boundary)
            e.rotate(-ang);
    }
    const BoundingBox bb = get_extents(boundary);
    const double      x0 = unscale<double>(bb.min.x()), x1 = unscale<double>(bb.max.x());
    const double      y0 = unscale<double>(bb.min.y()), y1 = unscale<double>(bb.max.y());
    const double      res   = std::min(std::clamp(std::max(x1 - x0, y1 - y0) / 400., 0.05, 0.5), std::max(0.05, R / 2));
    const int         nrows = y1 - y0 > 1e-9 ? int(std::ceil((y1 - y0) / op.stepover)) + 1 : 1;
    const double      dy    = nrows > 1 ? (y1 - y0) / (nrows - 1) : 0.;
    const double      floor = ctx.h.bottom;
    const double      leave = op.stock_to_leave_axial;
    const double      tol   = std::max(op.tolerance * 0.5, 1e-4);

    // Stock to leave: drop an offset cutter (R + radial leave) and lift by the axial leave.
    const internal::DropCutter dc(mesh.its, internal::make_cutter(tool, op.stock_to_leave_radial));
    const auto height = [&](double x, double y) { return dc.at(x, y, floor - leave) + leave; };

    // Appends b to pts (ending at a), first inserting midpoints wherever the surface bulges above
    // the chord by more than tol / 2 (steep, convex spots such as a ball rolling over an edge).
    const std::function<void(std::vector<Vec3d>&, const Vec3d&, int)> refine = [&](std::vector<Vec3d>& pts, const Vec3d& b, int depth) {
        const Vec3d a = pts.back();
        if (depth < 8 && b.x() - a.x() > 0.01) {
            const double xm = 0.5 * (a.x() + b.x());
            const Vec3d  m(xm, a.y(), height(xm, a.y()));
            if (m.z() - 0.5 * (a.z() + b.z()) > 0.5 * tol) {
                refine(pts, m, depth + 1);
                refine(pts, b, depth + 1);
                return;
            }
        }
        pts.push_back(b);
    };

    std::vector<std::vector<std::vector<Vec3d>>> rows(nrows);
    tbb::parallel_for(tbb::blocked_range<int>(0, nrows), [&](const tbb::blocked_range<int>& r) {
        for (int j = r.begin(); j < r.end(); ++j) {
            const double y = y0 + j * dy;
            // nudge the first/last row inside so the boundary edge itself is kept
            const double yc = j == 0 ? y + 1e-4 : j == nrows - 1 ? y - 1e-4 : y;
            Polylines    segs = intersection_pl(Polylines{Polyline(Point::new_scale(x0 - 1, yc), Point::new_scale(x1 + 1, yc))}, boundary);
            std::vector<std::pair<double, double>> spans;
            for (const Polyline& s : segs) {
                const double a = unscale<double>(s.first_point().x()), b = unscale<double>(s.last_point().x());
                spans.emplace_back(std::min(a, b), std::max(a, b));
            }
            std::sort(spans.begin(), spans.end());
            for (auto [xa, xb] : spans) {
                const int          n = std::max(1, int(std::ceil((xb - xa) / res)));
                std::vector<Vec3d> pts{Vec3d(xa, yc, height(xa, yc))};
                for (int k = 1; k <= n; ++k) {
                    const double x = xa + (xb - xa) * k / n;
                    refine(pts, Vec3d(x, yc, height(x, yc)), 0);
                }
                rows[j].push_back(internal::simplify_pass(pts, tol));
            }
        }
    });

    std::vector<std::vector<Vec3d>> runs;
    for (auto& row : rows)
        for (auto& run : row)
            runs.push_back(std::move(run));
    if (runs.empty()) {
        tp.error = "Nothing to machine inside the boundary.";
        return tp;
    }

    // A straight feed link between passes is fine when it is short and never below the surface.
    const double link_max  = 2 * op.stepover;
    const auto   link_safe = [&](const Vec3d& a, const Vec3d& b) {
        const int n = std::max(1, int(std::ceil((b - a).head<2>().norm() / res)));
        for (int k = 1; k < n; ++k) {
            const Vec3d p = a + (b - a) * (double(k) / n);
            if (p.z() < height(p.x(), p.y()) - 1e-4)
                return false;
        }
        return true;
    };

    // Greedy nearest-run ordering (fewest retracts).
    // ponytail: O(runs^2); fine up to a few thousand runs.
    std::vector<char> done(runs.size(), 0);
    done[0] = 1;
    std::vector<Vec3d> out = runs[0];
    std::vector<std::pair<size_t, size_t>> run_spans{{0, out.size()}};
    for (size_t left = runs.size() - 1; left > 0; --left) {
        const Vec2d e = out.back().head<2>();
        double      best = std::numeric_limits<double>::max();
        size_t      bi = 0;
        bool        rev = false;
        for (size_t i = 0; i < runs.size(); ++i) {
            if (done[i])
                continue;
            const double ds = (runs[i].front().head<2>() - e).squaredNorm();
            if (ds < best) { best = ds; bi = i; rev = false; }
            if (op.bidirectional) {
                const double de = (runs[i].back().head<2>() - e).squaredNorm();
                if (de < best) { best = de; bi = i; rev = true; }
            }
        }
        done[bi] = 1;
        if (rev)
            std::reverse(runs[bi].begin(), runs[bi].end());
        run_spans.push_back({out.size(), out.size() + runs[bi].size()});
        out.insert(out.end(), runs[bi].begin(), runs[bi].end());
    }

    const ResolvedHeights& h  = ctx.h;
    const FeedsSpeeds&               fs = ctx.fs;
    for (size_t r = 0; r < run_spans.size(); ++r) {
        const auto [b, e] = run_spans[r];
        const Vec3d& s    = out[b];
        if (r > 0) {
            const Vec3d& prev = out[b - 1];
            if ((s - prev).head<2>().norm() <= link_max && link_safe(prev, s))
                add_move(tp, K::Feed, s, fs.feed);
            else
                append_link_move(tp, s, h, link_max, fs);
        } else
            append_link_move(tp, s, h, link_max, fs);
        for (size_t i = b + 1; i < e; ++i)
            add_move(tp, K::Feed, out[i], fs.feed);
    }
    append_retract(tp, h.clearance);

    if (ang != 0) {
        const Eigen::Rotation2Dd rot(ang);
        for (Move& m : tp.moves) {
            m.to.head<2>()     = rot * m.to.head<2>();
            m.center.head<2>() = rot * m.center.head<2>();
        }
    }
    return tp;
}

// ---- Contour3D ---------------------------------------------------------------------------------

Toolpath generate_contour3d(const CamDocument& doc, const CamOperation& op, const CamModel& model)
{
    Toolpath            tp;
    internal::OpContext ctx = internal::op_context(doc, op, model);
    if (!ctx.error.empty()) {
        tp.error = ctx.error;
        return tp;
    }
    if (op.stepdown <= 0) {
        tp.error = "Stepdown must be greater than zero.";
        return tp;
    }
    const CamTool&          tool = *ctx.tool;
    const internal::Cutter  cut  = internal::make_cutter(tool);
    const double            La = op.stock_to_leave_axial, Lr = op.stock_to_leave_radial;
    const ResolvedHeights&  h  = ctx.h;
    const FeedsSpeeds&      fs = ctx.fs;

    std::vector<double> faces = internal::horizontal_face_zs(ctx.mesh.its);
    for (double& z : faces)
        z += La;
    const std::vector<double> levels = levels_with(h.top, h.bottom + La, op.stepdown, faces, true);

    // The tool's centre must stay outside the union over heights t above the tip of the model
    // shadow at z + t grown by the cutter radius at t. Flat: t = 0 only. Ball/bull/cone: sampled.
    // ponytail: 5 samples of the tip profile (0, 1/4 .. 1 of its height); the corner between two
    // samples can under-cut a sloped wall by a few microns. More samples if it ever matters.
    std::vector<double> ts{0.};
    if (cut.profile_h() > 1e-9)
        for (int k = 1; k <= 4; ++k)
            ts.push_back(cut.profile_h() * k / 4);
    std::vector<double> zs;
    for (double z : levels)
        for (double t : ts)
            zs.push_back(z - La + t + 1e-4);
    const std::vector<ExPolygons> shadows = shadow_sections(ctx.mesh.its, zs);

    std::vector<ExPolygons> forbidden(levels.size());
    tbb::parallel_for(tbb::blocked_range<size_t>(0, levels.size()), [&](const tbb::blocked_range<size_t>& r) {
        for (size_t l = r.begin(); l < r.end(); ++l) {
            Polygons all;
            for (size_t k = 0; k < ts.size(); ++k) {
                const ExPolygons& s = shadows[l * ts.size() + k];
                if (!s.empty())
                    append(all, to_polygons(offset_ex(s, float(scale_(cut.radius_at(ts[k]) + Lr)), ClipperLib::jtRound)));
            }
            forbidden[l] = union_ex(all);
        }
    });

    const bool   climb = op.climb;
    const double rl    = std::max(0., op.lead_in_radius);
    const double hop   = 2 * tool.diameter;
    for (size_t l = 0; l < levels.size(); ++l) {
        const ExPolygons& fb = forbidden[l];
        if (fb.empty())
            continue;
        const double     z      = levels[l];
        const ExPolygons inner  = offset_ex(fb, -float(scale_(0.01)));
        const auto       in_fb  = [&](const Vec2d& p) {
            const Point pt = Point::new_scale(p.x(), p.y());
            for (const ExPolygon& e : fb)
                if (e.contains(pt))
                    return true;
            return false;
        };

        std::vector<std::vector<Vec2d>> loops;
        for (const Polygon& poly : to_polygons(fb)) {
            if (poly.size() < 3)
                continue;
            std::vector<Vec2d> pts;
            for (const Point& p : poly.points)
                pts.push_back(mm(p));
            if (climb) // climb: outer contours CW, holes CCW = reverse the native orientation
                std::reverse(pts.begin(), pts.end());
            loops.push_back(std::move(pts));
        }

        std::vector<char> done(loops.size(), 0);
        for (size_t left = loops.size(); left > 0; --left) {
            const Vec2d cur = tp.moves.empty() ? loops[0][0] : Vec2d(tp.moves.back().to.head<2>());
            // nearest loop (by its nearest vertex)
            double best = std::numeric_limits<double>::max();
            size_t bl = 0, bv = 0;
            for (size_t i = 0; i < loops.size(); ++i)
                if (!done[i])
                    for (size_t v = 0; v < loops[i].size(); ++v) {
                        const double d = (loops[i][v] - cur).squaredNorm();
                        if (d < best) { best = d; bl = i; bv = v; }
                    }
            done[bl] = 1;
            std::vector<Vec2d> pts = loops[bl];
            std::rotate(pts.begin(), pts.begin() + bv, pts.end());
            const size_t n = pts.size();

            // Tangential quarter-arc lead-in/out on the air side (left of travel when climbing).
            const auto   side = [&](const Vec2d& t) { return climb ? Vec2d(-t.y(), t.x()) : Vec2d(t.y(), -t.x()); };
            const Vec2d& S    = pts[0];
            const Vec2d  Tin  = (pts[1] - S).normalized();
            const Vec2d  Tout = (S - pts[n - 1]).normalized();
            const Vec2d  Nin = side(Tin), Nout = side(Tout);
            const Vec2d  P0 = S - Tin * rl + Nin * rl, Cin = S + Nin * rl;
            const Vec2d  E  = S + Tout * rl + Nout * rl, Cout = S + Nout * rl;
            const auto   arc_mid = [&](const Vec2d& c, const Vec2d& a, const Vec2d& b) {
                const Vec2d m = (a - c) + (b - c);
                return Vec2d(c + m.normalized() * rl);
            };
            const bool lead_in  = rl > 0 && !in_fb(P0) && !in_fb(arc_mid(Cin, P0, S));
            const bool lead_out = rl > 0 && !in_fb(E) && !in_fb(arc_mid(Cout, S, E));
            const Vec3d entry(lead_in ? P0.x() : S.x(), lead_in ? P0.y() : S.y(), z);

            // Link: stay down when the straight move is short and clear of the part at this level.
            if (tp.moves.empty())
                append_link_move(tp, entry, h, hop, fs);
            else {
                const Vec3d c3 = tp.moves.back().to;
                const bool  down_ok = c3.z() >= z - 1e-9 && (c3 - entry).head<2>().norm() <= hop &&
                                     intersection_pl(Polylines{Polyline(Point::new_scale(c3.x(), c3.y()),
                                                                        Point::new_scale(entry.x(), entry.y()))},
                                                     inner).empty();
                if (down_ok) {
                    if ((c3 - entry).head<2>().norm() > 1e-6)
                        add_move(tp, K::Feed, {entry.x(), entry.y(), c3.z()}, fs.feed);
                    if (c3.z() > z + 1e-9)
                        add_move(tp, K::Plunge, entry, fs.plunge_feed);
                } else
                    append_link_move(tp, entry, h, hop, fs);
            }
            const ArcDir dir = climb ? ArcDir::CCW : ArcDir::CW;
            if (lead_in) {
                Move m;
                m.kind   = K::LeadIn;
                m.to     = Vec3d(S.x(), S.y(), z);
                m.center = Vec3d(Cin.x(), Cin.y(), z);
                m.feed   = fs.feed;
                m.arc    = dir;
                tp.moves.push_back(m);
            }
            for (size_t i = 1; i <= n; ++i)
                add_move(tp, K::Feed, {pts[i % n].x(), pts[i % n].y(), z}, fs.feed);
            if (lead_out) {
                Move m;
                m.kind   = K::LeadOut;
                m.to     = Vec3d(E.x(), E.y(), z);
                m.center = Vec3d(Cout.x(), Cout.y(), z);
                m.feed   = fs.feed;
                m.arc    = dir;
                tp.moves.push_back(m);
            }
        }
    }
    if (tp.moves.empty()) {
        tp.error = "The model is outside the operation's height range.";
        return tp;
    }
    append_retract(tp, h.clearance);
    return tp;
}

// ---- Adaptive3D --------------------------------------------------------------------------------

Toolpath generate_adaptive3d(const CamDocument& doc, const CamOperation& op, const CamModel& model)
{
    Toolpath            tp;
    internal::OpContext ctx = internal::op_context(doc, op, model);
    if (!ctx.error.empty()) {
        tp.error = ctx.error;
        return tp;
    }
    if (op.stepdown <= 0) {
        tp.error = "Stepdown must be greater than zero.";
        return tp;
    }
    const CamTool&         tool = *ctx.tool;
    const CamSetupFrame&   fr   = *ctx.frame;
    const ResolvedHeights& h    = ctx.h;
    const FeedsSpeeds&     fs   = ctx.fs;
    const double           D    = tool.diameter;
    const double           La   = op.stock_to_leave_axial;
    const double           lowest = h.bottom + La;
    if (lowest >= h.top) {
        tp.error = "The bottom height is above the top height.";
        return tp;
    }

    std::vector<double> faces = internal::horizontal_face_zs(ctx.mesh.its);
    for (double& z : faces)
        z += La;
    const std::vector<double> levels = levels_with(h.top, lowest, op.stepdown, faces, false);

    // keep-out = the model shadow at the tip height (minus the axial leave). Exact for a flat
    // cutter; ball/bull/cone cutters sit inside the flat cylinder, so they only leave more.
    std::vector<double> zs;
    for (double z : levels)
        zs.push_back(z - La + 1e-4);
    const std::vector<ExPolygons> keep_out = shadow_sections(ctx.mesh.its, zs);

    const bool cylinder = doc.setups[op.setup_index].stock.kind == StockKind::Cylinder && fr.stock_radius > 0;
    const auto stock_section = [&](double z) {
        if (!cylinder)
            return ExPolygons{rect_mm(fr.stock.min.x(), fr.stock.min.y(), fr.stock.max.x(), fr.stock.max.y())};
        const double r = fr.stock_radius;
        const double w = z > 0 ? std::sqrt(std::max(0., r * r - z * z)) : r; // material at or above z
        return w > 1e-3 ? ExPolygons{rect_mm(fr.stock.min.x(), -w, fr.stock.max.x(), w)} : ExPolygons{};
    };

    Adaptive::Params ap;
    ap.tool_diameter     = D;
    ap.stepover_fraction = std::clamp(op.optimal_load / D, 0.02, 1.);
    ap.helix_diameter    = op.helix_diameter;
    ap.helix_angle_deg   = op.helix_angle_deg;
    // roughing: 0.02 mm is plenty (and Adaptive is 3x faster than at 0.01)
    ap.tolerance         = std::max(op.tolerance, 0.02);
    ap.stock_to_leave    = op.stock_to_leave_radial;
    ap.climb             = op.climb;
    // ponytail: pocket mode over the stock grown by D + 1 mm (the tool can still pass fully outside
    // the stock) instead of Adaptive's outside_is_air mode, which is ~100x slower at stepovers
    // below ~20 % of D (52 s vs 0.4 s on 40 x 40 mm). Entries helix into the stock instead of
    // plunging in the air, and the margin is cut as if it were stock (passing it as
    // `already_cleared` is as slow as air mode). Switch back once air mode is fast.
    ap.outside_is_air = false;
    const float grow  = float(scale_(D + 1));

    // A level whose stock and keep-out match the previous one clears the same XY: reuse it.
    // (op.rest_machining is not used across levels: nothing at a new Z is cleared yet.)
    std::vector<ExPolygons> stock(levels.size());
    std::vector<size_t>     src(levels.size());
    std::vector<size_t>     unique;
    for (size_t l = 0; l < levels.size(); ++l) {
        stock[l] = stock_section(levels[l]);
        if (l > 0 && same_area(stock[l], stock[src[l - 1]]) && same_area(keep_out[l], keep_out[src[l - 1]]))
            src[l] = src[l - 1];
        else {
            src[l] = l;
            unique.push_back(l);
        }
    }
    // Per unique level: Adaptive, or (material nowhere wider than the tool around a keep-out)
    // concentric passes around the keep-out, outermost first, `load` apart. Adaptive is slow and
    // pointless in such thin rings (57 s for the 0.5 mm skin around an 80 mm block).
    const double load = std::clamp(op.optimal_load, 0.02 * D, D);
    struct Plan {
        Adaptive::Result          res;
        std::vector<Polygons>     rings; // outermost first; tool-centre loops
    };
    std::vector<Plan> plans(levels.size());
    tbb::parallel_for(tbb::blocked_range<size_t>(0, unique.size(), 1), [&](const tbb::blocked_range<size_t>& r) {
        for (size_t u = r.begin(); u < r.end(); ++u) {
            const size_t l = unique[u];
            if (stock[l].empty())
                continue;
            const ExPolygons material = diff_ex(stock[l], offset_ex(keep_out[l], float(scale_(op.stock_to_leave_radial)), ClipperLib::jtRound));
            if (material.empty())
                continue;
            if (!keep_out[l].empty() && offset_ex(material, -float(scale_(D / 2))).empty()) {
                for (int k = 0; k < 1000; ++k) {
                    const double off = op.stock_to_leave_radial + k * load;
                    plans[l].rings.push_back(to_polygons(offset_ex(keep_out[l], float(scale_(off + D / 2)), ClipperLib::jtRound)));
                    if (diff_ex(material, offset_ex(keep_out[l], float(scale_(off + D / 2)), ClipperLib::jtRound)).empty())
                        break;
                }
                std::reverse(plans[l].rings.begin(), plans[l].rings.end());
            } else
                plans[l].res = Adaptive::clear(offset_ex(stock[l], grow), keep_out[l], ap);
        }
    });

    for (size_t l = 0; l < levels.size(); ++l) {
        const Plan&  plan   = plans[src[l]];
        const double z      = levels[l];
        const double z_prev = l == 0 ? h.top : levels[l - 1];
        for (const Polygons& ring : plan.rings)
            for (const Polygon& poly : ring) {
                std::vector<Vec2d> pts;
                for (const Point& p : poly.points)
                    pts.push_back(mm(p));
                if (op.climb) // climb: outer contours CW, holes CCW
                    std::reverse(pts.begin(), pts.end());
                size_t      bv  = 0;
                const Vec2d cur = tp.moves.empty() ? pts[0] : Vec2d(tp.moves.back().to.head<2>());
                for (size_t v = 1; v < pts.size(); ++v)
                    if ((pts[v] - cur).squaredNorm() < (pts[bv] - cur).squaredNorm())
                        bv = v;
                std::rotate(pts.begin(), pts.begin() + bv, pts.end());
                append_link_move(tp, {pts[0].x(), pts[0].y(), z}, h, 3 * D, fs);
                for (size_t i = 1; i <= pts.size(); ++i)
                    add_move(tp, K::Feed, {pts[i % pts.size()].x(), pts[i % pts.size()].y(), z}, fs.feed);
            }
        if (!plan.rings.empty())
            continue;
        const Adaptive::Result& res = plan.res;
        if (!res.ok) {
            if (!res.error.empty() && res.error.rfind("Nothing to clear", 0) != 0)
                add_warning(tp, "Level Z " + std::to_string(int(std::round(z))) + ": " + res.error);
            continue;
        }
        for (const std::string& w : res.warnings)
            add_warning(tp, w);
        append_adaptive_level(tp, res, op, fs, h, z_prev, z);
    }
    if (tp.moves.empty()) {
        tp.error = "Nothing to rough: the stock is already at the model shape.";
        return tp;
    }
    append_retract(tp, h.clearance);
    return tp;
}

} // namespace Slic3r::CAM
