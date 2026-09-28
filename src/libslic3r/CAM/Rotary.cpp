#include "libslic3r/CAM/Rotary.hpp"
#include "libslic3r/CAM/CamInternal.hpp"
#include "libslic3r/CAM/Op2D.hpp"

#include "libslic3r/ClipperUtils.hpp"

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <algorithm>
#include <cmath>

namespace Slic3r::CAM {

using internal::add_move;
using K = Move::Kind;

Transform3d apply_index(const Transform3d& frame, double a_deg)
{
    return Transform3d(Eigen::AngleAxisd(a_deg * M_PI / 180., Vec3d::UnitX())) * frame;
}

namespace {

// Feed for a move with an A component, so the surface speed at `radius` equals `feed`: the
// controller (without inverse time) meters sqrt(dx^2 + dy^2 + dz^2 + dA^2) with A in degrees.
double rotary_feed(double feed, const Move& from, const Move& to, double radius)
{
    const double da = to.a_deg - from.a_deg;
    if (std::abs(da) < 1e-9)
        return feed;
    const double lin2 = (to.to - from.to).squaredNorm();
    const double ls   = std::sqrt(lin2 + std::pow(da * M_PI / 180. * radius, 2));
    const double lc   = std::sqrt(lin2 + da * da);
    return ls > 1e-9 ? feed * lc / ls : feed;
}

} // namespace

Toolpath generate_rotary_wrap(const CamDocument& doc, const CamOperation& op, const CamModel& model, const ProgressFn& progress)
{
    Toolpath            tp;
    internal::OpContext ctx = internal::op_context(doc, op, model, false);
    if (!ctx.error.empty()) {
        tp.error = ctx.error;
        return tp;
    }
    const CamSetupFrame& fr = *ctx.frame;
    const double         r  = op.wrap_radius > 0 ? op.wrap_radius : fr.stock_radius;
    if (r <= 0) {
        tp.error = "Wrapping needs a cylinder stock or a wrap radius.";
        return tp;
    }
    const CamSketch* sk = model.sketch(op.geom.sketch_feature);
    if (!sk) {
        tp.error = "Select a sketch to wrap around the cylinder.";
        return tp;
    }
    // The sketch seen from above (setup XY) is the unrolled surface: X along the axis, Y = arc length.
    ExPolygons regions;
    Polylines  chains;
    internal::sketch_to_setup_xy(*sk, fr.to_setup, regions, chains);

    // Depth is radial: the selection is the wrap surface (Z = r above the axis).
    const ResolvedHeights h    = resolve_heights(op.heights, fr, r, r);
    const CamTool&        tool = *ctx.tool;
    Toolpath              flat;
    switch (op.wrap_strategy) {
    case OpType::Contour2D: flat = contour_region(regions, tool, op, ctx.fs, h, progress); break;
    // ponytail: Adaptive2D wraps as a pocket (A's adaptive region builder is private to Op2D.cpp).
    case OpType::Pocket2D:
    case OpType::Adaptive2D: flat = pocket_region(regions, tool, op, ctx.fs, h, progress); break;
    case OpType::Face: flat = face_region(regions, tool, op, ctx.fs, h, progress); break;
    default: { // Engrave, Trace, Slot, Chamfer2D: tool centre on the lines and profile outlines
        Polylines lines = chains;
        for (const Polygon& p : to_polygons(regions)) {
            Polyline pl(p.points);
            pl.points.push_back(p.points.front());
            lines.push_back(std::move(pl));
        }
        flat = trace_chains(lines, tool, op, ctx.fs, h, progress);
    }
    }
    if (!flat.ok()) {
        tp.error = flat.error;
        return tp;
    }
    tp.warnings = flat.warnings;
    // XY arcs have no meaning once Y becomes A: linearise them.
    for (const Move& m : flat.moves) {
        if (!is_arc(m) || tp.moves.empty()) {
            tp.moves.push_back(m);
            continue;
        }
        const Vec3d from = tp.moves.back().to;
        for (const Vec3d& p : arc_points(from, m, std::max(op.tolerance, 0.001))) {
            Move l = m;
            l.kind = m.kind == Move::Kind::ArcCW || m.kind == Move::Kind::ArcCCW ? Move::Kind::Feed : m.kind;
            l.arc  = ArcDir::None;
            l.to   = p;
            tp.moves.push_back(l);
        }
    }

    // Wrap: Y (arc length at r) -> A; the tool stays over the axis (Y = 0), Z is the tip radius.
    Move prev;
    for (size_t i = 0; i < tp.moves.size(); ++i) {
        Move& m   = tp.moves[i];
        m.a_deg   = wrap_y_to_a_deg(m.to.y(), r);
        const double unrolled_len = i > 0 ? (m.to - prev.to).norm() : 0.;
        const Move   unrolled     = m;
        m.to.y()  = 0;
        if (i > 0 && m.kind != K::Rapid && unrolled_len > 1e-9) {
            const double da   = m.a_deg - tp.moves[i - 1].a_deg;
            const double lin2 = (m.to - tp.moves[i - 1].to).squaredNorm();
            m.feed            = m.feed * std::sqrt(lin2 + da * da) / unrolled_len;
        }
        prev = unrolled;
    }
    return tp;
}

Toolpath generate_rotary_finish(const CamDocument& doc, const CamOperation& op, const CamModel& model, const ProgressFn& progress)
{
    Toolpath            tp;
    internal::OpContext ctx = internal::op_context(doc, op, model);
    if (!ctx.error.empty()) {
        tp.error = ctx.error;
        return tp;
    }
    const CamTool&         tool = *ctx.tool;
    const CamSetupFrame&   fr   = *ctx.frame;
    const ResolvedHeights& h    = ctx.h;
    const FeedsSpeeds&     fs   = ctx.fs;
    const double           R    = tool.diameter / 2;
    const double           x0   = std::max(fr.model.min.x(), fr.stock.min.x());
    const double           x1   = std::min(fr.model.max.x(), fr.stock.max.x());
    if (x1 <= x0) {
        tp.error = "The model is outside the stock.";
        return tp;
    }
    const double res   = std::min(std::clamp((x1 - x0) / 400., 0.05, 0.5), std::max(0.05, R / 2));
    const int    nx    = int(std::ceil((x1 - x0) / res)) + 1;
    const int    na    = std::max(1, int(std::lround(360. / std::max(op.a_stepover_deg, 0.05))));
    const double da    = 360. / na;
    const double floor = std::max(h.bottom, 0.);
    const double La    = op.stock_to_leave_axial;
    const internal::Cutter cutter = internal::make_cutter(tool, op.stock_to_leave_radial);

    // Radial drop-cutter: at A = a the part is rotated by a about X under the vertical tool at
    // Y = 0, so a plain drop-cutter over the rotated mesh along the X axis is exact.
    std::vector<std::vector<double>> zt(na, std::vector<double>(nx));
    ProgressCounter pc(progress, size_t(na), 0, 0.95);
    tbb::parallel_for(tbb::blocked_range<int>(0, na, 4), [&](const tbb::blocked_range<int>& rg) {
        for (int k = rg.begin(); k < rg.end(); ++k) {
            if (pc.step())
                return;
            TriangleMesh m = ctx.mesh;
            m.transform(apply_index(Transform3d::Identity(), k * da));
            const internal::DropCutter dc(m.its, cutter);
            for (int i = 0; i < nx; ++i)
                zt[k][i] = dc.at(x0 + (x1 - x0) * i / (nx - 1), 0., floor - La) + La;
        }
    });
    if (pc.cancelled()) {
        tp.error = "Cancelled";
        return tp;
    }
    const auto xi = [&](int i) { return x0 + (x1 - x0) * i / (nx - 1); };

    const auto feed_move = [&](K kind, const Vec3d& to, double a, double f) {
        Move m;
        m.kind  = kind;
        m.to    = to;
        m.a_deg = a;
        m.feed  = tp.moves.empty() ? f : rotary_feed(f, tp.moves.back(), m, std::max(to.z(), 1.));
        tp.moves.push_back(m);
    };
    const auto first_link = [&](const Vec3d& to, double a) {
        append_link_move(tp, to, h, 0, fs);
        for (Move& m : tp.moves)
            m.a_deg = a;
    };
    const double tol = std::max(op.tolerance * 0.5, 1e-4);

    if (op.rotary_spiral) {
        // ponytail: spiral heights are interpolated between the A columns of the grid (a_stepover_deg
        // apart); a fine a_stepover_deg keeps the chord error small.
        const double pitch = op.stepover > 0 ? op.stepover : R / 2;
        const double turns = std::ceil((x1 - x0) / pitch);
        const double step  = da / 2;
        const int    n     = int(std::ceil(turns * 360. / step));
        const auto   z_at  = [&](double a, double x) {
            const double fa = std::fmod(a, 360.) / da, fx = std::clamp((x - x0) / (x1 - x0) * (nx - 1), 0., double(nx - 1));
            const int    k0 = int(fa) % na, k1 = (k0 + 1) % na, i0 = std::min(int(fx), nx - 2), i1 = i0 + 1;
            const double tx = fx - i0;
            const auto   col = [&](int k) { return zt[k][i0] * (1 - tx) + zt[k][i1] * tx; };
            // max of the two neighbouring columns: never below either one's surface
            return std::max(col(k0), col(k1));
        };
        for (int s = 0; s <= n; ++s) {
            const double a = s * step, x = std::min(x1, x0 + pitch * a / 360.);
            const Vec3d  p(x, 0, z_at(a, x));
            if (s == 0)
                first_link(p, a);
            else
                feed_move(K::Feed, p, a, fs.feed);
        }
    } else {
        for (int k = 0; k < na; ++k) {
            const double       a = k * da;
            std::vector<Vec3d> pts;
            for (int i = 0; i < nx; ++i)
                pts.emplace_back(xi(i), 0., zt[k][i]);
            pts = internal::simplify_pass(pts, tol);
            if (op.bidirectional && k % 2 == 1)
                std::reverse(pts.begin(), pts.end());
            if (k == 0)
                first_link(pts[0], a);
            else if (op.bidirectional) {
                // lift clear of both passes, index A, come back down
                const Vec3d  e    = tp.moves.back().to;
                const double z_up = std::max(e.z(), pts[0].z()) + std::max(op.lift_height, 0.1);
                feed_move(K::Retract, {e.x(), 0, z_up}, a - da, fs.plunge_feed);
                feed_move(K::Feed, {e.x(), 0, z_up}, a, fs.feed);
                feed_move(K::Plunge, pts[0], a, fs.plunge_feed);
            } else {
                const Vec3d e = tp.moves.back().to;
                add_move(tp, K::Retract, {e.x(), 0, h.clearance}, fs.plunge_feed, a - da);
                add_move(tp, K::Rapid, {pts[0].x(), 0, h.clearance}, 0, a);
                add_move(tp, K::Rapid, {pts[0].x(), 0, std::max(h.retract, pts[0].z())}, 0, a);
                feed_move(K::Plunge, pts[0], a, fs.plunge_feed);
            }
            for (size_t i = 1; i < pts.size(); ++i)
                feed_move(K::Feed, pts[i], a, fs.feed);
        }
    }
    append_retract(tp, h.clearance);
    return tp;
}

} // namespace Slic3r::CAM
