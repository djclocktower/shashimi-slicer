#include <catch2/catch_all.hpp>

#include "libslic3r/CAM/CAM.hpp"
#include "libslic3r/ClipperUtils.hpp"

#include <set>

using namespace Slic3r;
using namespace Slic3r::CAM;
using Catch::Approx;

namespace {

Polygon rect(double x0, double y0, double x1, double y1) { return Polygon::new_scale({{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}); }

Polygon circle(double cx, double cy, double r, int n = 72)
{
    std::vector<Vec2d> pts;
    for (int i = 0; i < n; ++i)
        pts.emplace_back(cx + r * cos(2 * M_PI * i / n), cy + r * sin(2 * M_PI * i / n));
    return Polygon::new_scale(pts);
}

CamTool flat(double d)
{
    CamTool t;
    t.diameter = d;
    return t;
}

FeedsSpeeds feeds() { return FeedsSpeeds{}; }

ResolvedHeights heights(double top, double bottom)
{
    ResolvedHeights h;
    h.clearance = top + 10;
    h.retract   = top + 5;
    h.top       = top;
    h.bottom    = bottom;
    return h;
}

// Every move as a polyline of 3D points (arcs sampled), with the kind of the move that made it.
struct Seg { Vec3d a, b; Move::Kind kind; };
std::vector<Seg> segments(const Toolpath& tp)
{
    std::vector<Seg> out;
    for (size_t i = 1; i < tp.moves.size(); ++i) {
        Vec3d p = tp.moves[i - 1].to;
        for (const Vec3d& q : arc_points(p, tp.moves[i], 0.005)) {
            out.push_back({p, q, tp.moves[i].kind});
            p = q;
        }
    }
    return out;
}

bool cutting(Move::Kind k) { return k != Move::Kind::Rapid && k != Move::Kind::Retract; }

// Cells swept by the tool's disc along cutting moves at exactly z.
struct Raster {
    double x0, y0, res;
    int    nx, ny;
    std::vector<uint8_t> cut;
    Raster(const BoundingBox& bb, double res_) : res(res_)
    {
        x0 = unscale<double>(bb.min.x()) - 1; y0 = unscale<double>(bb.min.y()) - 1;
        nx = int((unscale<double>(bb.max.x()) + 1 - x0) / res) + 1;
        ny = int((unscale<double>(bb.max.y()) + 1 - y0) / res) + 1;
        cut.assign(size_t(nx) * ny, 0);
    }
    Vec2d center(int i, int j) const { return {x0 + (i + 0.5) * res, y0 + (j + 0.5) * res}; }
    void sweep(const Vec2d& a, const Vec2d& b, double r)
    {
        const Vec2d ab = b - a;
        const double l2 = ab.squaredNorm();
        const int i0 = std::max(0, int((std::min(a.x(), b.x()) - r - x0) / res));
        const int i1 = std::min(nx - 1, int((std::max(a.x(), b.x()) + r - x0) / res) + 1);
        const int j0 = std::max(0, int((std::min(a.y(), b.y()) - r - y0) / res));
        const int j1 = std::min(ny - 1, int((std::max(a.y(), b.y()) + r - y0) / res) + 1);
        for (int j = j0; j <= j1; ++j)
            for (int i = i0; i <= i1; ++i) {
                const Vec2d p = center(i, j);
                const double t = l2 > 0 ? std::clamp((p - a).dot(ab) / l2, 0., 1.) : 0.;
                if ((p - (a + t * ab)).squaredNorm() <= r * r)
                    cut[size_t(j) * nx + i] = 1;
            }
    }
    // fraction of `area` covered
    double coverage(const ExPolygons& area) const
    {
        size_t in = 0, hit = 0;
        for (int j = 0; j < ny; ++j)
            for (int i = 0; i < nx; ++i) {
                const Vec2d c = center(i, j);
                const Point p(coord_t(scale_(c.x())), coord_t(scale_(c.y())));
                bool inside = false;
                for (const ExPolygon& e : area) inside |= e.contains(p);
                if (!inside) continue;
                ++in;
                hit += cut[size_t(j) * nx + i];
            }
        return in ? double(hit) / in : 0;
    }
};

double coverage_at(const Toolpath& tp, const ExPolygons& area, double r, double z)
{
    Raster ras(get_extents(area), 0.2);
    for (const Seg& s : segments(tp))
        if (cutting(s.kind) && std::abs(s.a.z() - z) < 1e-6 && std::abs(s.b.z() - z) < 1e-6)
            ras.sweep(s.a.head<2>(), s.b.head<2>(), r);
    return ras.coverage(area);
}

std::set<double> feed_levels(const Toolpath& tp)
{
    std::set<double> zs;
    for (const Move& m : tp.moves)
        if (m.kind == Move::Kind::Feed)
            zs.insert(std::round(m.to.z() * 1e6) / 1e6);
    return zs;
}

double dist_to(const ExPolygons& ex, const Vec3d& p)
{
    const Point q(coord_t(scale_(p.x())), coord_t(scale_(p.y())));
    double best = 1e30;
    for (const ExPolygon& e : ex)
        best = std::min(best, (e.point_projection(q) - q).cast<double>().norm());
    return unscale<double>(best);
}

// No rapid below the retract plane, and plunges only straight down.
void check_links(const Toolpath& tp, const ResolvedHeights& h)
{
    REQUIRE(tp.moves.front().kind == Move::Kind::Rapid);
    REQUIRE(tp.moves.front().to.z() == Approx(h.clearance));
    for (size_t i = 1; i < tp.moves.size(); ++i) {
        const Move& m = tp.moves[i];
        if (m.kind == Move::Kind::Rapid)
            REQUIRE(m.to.z() >= h.retract - 1e-9);
        if (m.kind == Move::Kind::Plunge)
            REQUIRE((m.to - tp.moves[i - 1].to).head<2>().norm() < 1e-9);
    }
    REQUIRE(tp.moves.back().to.z() == Approx(h.clearance));
}

} // namespace

TEST_CASE("Pocket clears the region around an island", "[Cam2D]")
{
    const ExPolygons region = diff_ex(Polygons{rect(0, 0, 60, 40)}, Polygons{circle(30, 20, 6)});
    CamOperation op;
    op.stepover = 2.4;
    op.stepdown = 1.0;
    const ResolvedHeights h = heights(0, -3);
    const CamTool tool = flat(6);

    for (EntryType entry : {EntryType::Helix, EntryType::Ramp, EntryType::Plunge}) {
        op.entry = entry;
        const Toolpath tp = pocket_region(region, tool, op, feeds(), h);
        REQUIRE(tp.ok());
        check_links(tp, h);
        REQUIRE(feed_levels(tp) == std::set<double>{-1, -2, -3});
        // what a 6 mm tool can reach (corners keep the tool radius)
        const ExPolygons reachable = offset_ex(offset_ex(region, -scale_(3.)), scale_(3.));
        REQUIRE(coverage_at(tp, reachable, 3, -3) >= 0.97);
        // the tool centre never leaves the wall offset
        const ExPolygons wall = offset_ex(region, -scale_(3. - 0.02));
        for (const Seg& s : segments(tp))
            if (cutting(s.kind) && s.b.z() < 0)
                REQUIRE(wall.front().contains(Point(coord_t(scale_(s.b.x())), coord_t(scale_(s.b.y())))));
        if (entry == EntryType::Helix) {
            bool helix = false;
            for (const Move& m : tp.moves) helix |= m.kind == Move::Kind::Ramp && is_arc(m);
            REQUIRE(helix);
        }
        if (entry == EntryType::Ramp) {
            bool ramp = false;
            for (const Move& m : tp.moves) ramp |= m.kind == Move::Kind::Ramp && !is_arc(m);
            REQUIRE(ramp);
        }
    }
}

TEST_CASE("Pocket with finishing passes and stock to leave", "[Cam2D]")
{
    const ExPolygons region{ExPolygon(rect(0, 0, 30, 30))};
    CamOperation op;
    op.stepover = 2.4;
    op.stepdown = 5;
    op.finishing_passes = 1;
    op.finish_stepover  = 0.2;
    op.stock_to_leave_radial = 0.3;
    const Toolpath tp = pocket_region(region, flat(6), op, feeds(), heights(0, -2));
    REQUIRE(tp.ok());
    // the outermost pass is exactly tool radius + stock to leave from the wall
    double min_d = 1e9;
    for (const Move& m : tp.moves)
        if (m.kind == Move::Kind::Feed)
            min_d = std::min(min_d, dist_to(region, m.to));
    REQUIRE(min_d == Approx(3.3).margin(0.01));
}

TEST_CASE("Pocket too small for the tool says so", "[Cam2D]")
{
    const Toolpath tp = pocket_region({ExPolygon(rect(0, 0, 5, 5))}, flat(6), CamOperation{}, feeds(), heights(0, -2));
    REQUIRE_FALSE(tp.ok());
    REQUIRE(tp.error.find("larger than the pocket") != std::string::npos);
}

TEST_CASE("Contour offsets by tool radius + stock to leave with tangent leads", "[Cam2D]")
{
    const ExPolygons region{ExPolygon(rect(0, 0, 40, 30))};
    for (ContourSide side : {ContourSide::Outside, ContourSide::Inside})
        for (bool climb : {true, false}) {
            CamOperation op;
            op.side                  = side;
            op.climb                 = climb;
            op.stepdown              = 1.5;
            op.stock_to_leave_radial = 0.5;
            op.lead_in_radius        = 2;
            const ResolvedHeights h  = heights(0, -4.5);
            const Toolpath tp        = contour_region(region, flat(6), op, feeds(), h);
            REQUIRE(tp.ok());
            check_links(tp, h);
            REQUIRE(feed_levels(tp) == std::set<double>{-1.5, -3, -4.5});
            for (const Move& m : tp.moves)
                if (m.kind == Move::Kind::Feed)
                    REQUIRE(dist_to(region, m.to) == Approx(3.5).margin(0.01));
            // leads: tangent arcs of the lead radius
            int leads = 0;
            for (size_t i = 1; i + 1 < tp.moves.size(); ++i) {
                const Move& m = tp.moves[i];
                if (m.kind != Move::Kind::LeadIn) continue;
                ++leads;
                REQUIRE(is_arc(m));
                const Vec2d c = m.center.head<2>(), p = m.to.head<2>();
                REQUIRE((tp.moves[i - 1].to.head<2>() - c).norm() == Approx(2).margin(1e-6));
                REQUIRE((p - c).norm() == Approx(2).margin(1e-6));
                const Vec2d t = (tp.moves[i + 1].to.head<2>() - p).normalized();
                REQUIRE(std::abs((p - c).normalized().dot(t)) < 1e-6);
                // the lead starts away from the material
                REQUIRE(dist_to(region, tp.moves[i - 1].to) > 3.5 - 1e-6);
            }
            REQUIRE(leads == 3);
            // direction: climb = material on the right. Outside: clockwise travel.
            double area2 = 0;
            std::vector<Vec2d> loop;
            for (const Move& m : tp.moves)
                if (m.kind == Move::Kind::Feed && std::abs(m.to.z() + 1.5) < 1e-9) loop.push_back(m.to.head<2>());
            for (size_t i = 0; i < loop.size(); ++i) {
                const Vec2d& a = loop[i]; const Vec2d& b = loop[(i + 1) % loop.size()];
                area2 += a.x() * b.y() - b.x() * a.y();
            }
            const bool cw = area2 < 0;
            REQUIRE(cw == (climb == (side == ContourSide::Outside)));
        }
}

TEST_CASE("Face covers the stock outline", "[Cam2D]")
{
    const ExPolygons stock{ExPolygon(rect(0, 0, 100, 50))};
    CamOperation op;
    op.stepover = 14;
    op.stepdown = 1;
    const ResolvedHeights h = heights(0, -0.5);
    const Toolpath tp = face_region(stock, flat(20), op, feeds(), h);
    REQUIRE(tp.ok());
    check_links(tp, h);
    REQUIRE(coverage_at(tp, stock, 10, -0.5) == Approx(1.0));
    // climb: every pass runs -X
    for (size_t i = 1; i < tp.moves.size(); ++i)
        if (tp.moves[i].kind == Move::Kind::Feed)
            REQUIRE(tp.moves[i].to.x() < tp.moves[i - 1].to.x());
}

TEST_CASE("Slot follows the centreline at the tool width with ramped levels", "[Cam2D]")
{
    CamOperation op;
    op.type     = OpType::Slot;
    op.stepdown = 1;
    op.ramp_angle_deg = 5;
    const Polylines chains{Polyline(Point::new_scale(0, 10), Point::new_scale(50, 10))};
    const ResolvedHeights h = heights(0, -3);
    const Toolpath tp = trace_chains(chains, flat(4), op, feeds(), h);
    REQUIRE(tp.ok());
    check_links(tp, h);
    int ramps = 0;
    for (const Move& m : tp.moves) {
        if (cutting(m.kind)) {
            // the tool centre stays on the centreline: the slot is exactly the tool wide
            REQUIRE(m.to.y() == Approx(10));
            REQUIRE(m.to.x() >= -1e-9);
            REQUIRE(m.to.x() <= 50 + 1e-9);
        }
        ramps += m.kind == Move::Kind::Ramp;
    }
    REQUIRE(ramps == 3);   // 1 mm at 5 deg = 11.4 mm, fits the 50 mm slot: one ramp per level
    REQUIRE(feed_levels(tp) == std::set<double>{-1, -2, -3});
    Raster ras(BoundingBox(Point::new_scale(-5, 0), Point::new_scale(55, 20)), 0.1);
    for (const Seg& s : segments(tp))
        if (cutting(s.kind) && s.b.z() < -2.99) ras.sweep(s.a.head<2>(), s.b.head<2>(), 2);
    // cut width across the middle = tool diameter
    int across = 0;
    for (int j = 0; j < ras.ny; ++j) across += ras.cut[size_t(j) * ras.nx + ras.nx / 2];
    REQUIRE(across * 0.1 == Approx(4).margin(0.2));
}

TEST_CASE("Chamfer runs the tool at the chamfer offset and depth", "[Cam2D]")
{
    // A 90 deg chamfer mill, width 1, tip 0.5 below: axis 0.5 out from the edge, tip at -1.5.
    CamDocument doc;
    CamTool     t;
    t.type          = ToolType::ChamferMill;
    t.diameter      = 6;
    t.tip_angle_deg = 90;
    doc.add_tool(t);
    CamSetup setup;
    setup.wcs.custom = true;   // setup frame = world: the sketch's edges sit at Z 0
    doc.add_setup(setup);
    CamModel model;
    CamSketch sk;
    sk.feature = 3;
    sk.regions = {ExPolygon(rect(0, 0, 20, 20))};
    model.sketches.push_back(sk);
    update_setup_frames(model, doc);
    CamOperation op      = default_operation(OpType::Chamfer2D, &doc.tools[0]);
    op.geom.sketch_feature = 3;
    op.chamfer_width     = 1;
    op.tip_offset        = 0.5;
    op.lead_in_radius    = 0;
    doc.add_operation(op);
    const Toolpath tp = generate_toolpath(doc, 0, model);
    REQUIRE(tp.ok());
    int n = 0;
    for (const Move& m : tp.moves)
        if (m.kind == Move::Kind::Feed) {
            ++n;
            REQUIRE(m.to.z() == Approx(-1.5));
            REQUIRE(dist_to(sk.regions, m.to) == Approx(0.5).margin(0.01));
        }
    REQUIRE(n > 4);

    doc.tools[0].type = ToolType::FlatEndMill;
    REQUIRE(generate_toolpath(doc, 0, model).error.find("chamfer mill") != std::string::npos);
}

TEST_CASE("Adaptive 2D through a sketch selection", "[Cam2D]")
{
    CamDocument doc;
    doc.add_tool(flat(6));
    CamSetup setup;
    setup.wcs.custom = true;   // setup frame = world
    doc.add_setup(setup);
    CamModel model;
    CamSketch sk;
    sk.feature = 0;
    sk.origin  = Vec3d(0, 0, -4);   // the sketch sits on a pocket floor 4 mm down
    sk.regions = {ExPolygon(rect(0, 0, 40, 30))};
    model.sketches.push_back(sk);
    update_setup_frames(model, doc);

    CamOperation op = default_operation(OpType::Adaptive2D, &doc.tools[0]);
    op.geom.sketch_feature = 0;
    op.stepdown = 2;
    doc.add_operation(op);
    const Toolpath tp = generate_toolpath(doc, 0, model);
    REQUIRE(tp.ok());
    REQUIRE(tp.time_s > 0);
    REQUIRE(tp.cut_length > 100);
    // no bodies: the stock is the offsets around an empty model at the origin (top Z 1)
    const ResolvedHeights h = resolve_heights(op.heights, model.setups[0], -4, -4);
    check_links(tp, h);
    REQUIRE(feed_levels(tp).count(-4) == 1);
    bool helix = false;
    for (const Move& m : tp.moves) helix |= m.kind == Move::Kind::Ramp && is_arc(m);
    REQUIRE(helix);
    const ExPolygons reachable = offset_ex(offset_ex(sk.regions, -scale_(3.)), scale_(3.));
    REQUIRE(coverage_at(tp, reachable, 3, -4) >= 0.97);

    SECTION("plain-language errors") {
        doc.operations[0].geom.sketch_feature = 7;
        REQUIRE(generate_toolpath(doc, 0, model).error == "The selected sketch no longer exists. Reselect the geometry.");
        doc.operations[0].geom.sketch_feature = -1;
        REQUIRE(generate_toolpath(doc, 0, model).error == "Select faces, edges or a sketch for this operation.");
        doc.operations[0].tool_number = 9;
        REQUIRE(generate_toolpath(doc, 0, model).error.find("T9") != std::string::npos);
    }
}

TEST_CASE("Face op through generate_toolpath faces the setup stock", "[Cam2D]")
{
    CamDocument doc;
    doc.add_tool(flat(10));
    doc.add_setup(CamSetup{});
    CamModel model;
    CamBody  b;
    b.body_id = 0;
    b.mesh    = TriangleMesh(its_make_cube(50, 30, 10));
    model.bodies.push_back(std::move(b));
    update_setup_frames(model, doc);
    doc.add_operation(default_operation(OpType::Face, &doc.tools[0]));
    const Toolpath tp = generate_toolpath(doc, 0, model);
    REQUIRE(tp.ok());
    // stock top 0, model top -1: one level at -1, the whole 52 x 32 stock covered
    REQUIRE(feed_levels(tp) == std::set<double>{-1});
    const BoundingBoxf3& s = model.setups[0].stock;
    const ExPolygons stock{ExPolygon(rect(s.min.x(), s.min.y(), s.max.x(), s.max.y()))};
    REQUIRE(coverage_at(tp, stock, 5, -1) == Approx(1.0));
}

TEST_CASE("Toolpath stats and time estimate", "[Cam2D]")
{
    Toolpath tp;
    const ResolvedHeights h = heights(0, -1);
    FeedsSpeeds fs;
    fs.feed = 600;
    fs.plunge_feed = 60;
    append_link_move(tp, Vec3d(0, 0, -1), h, 1, fs);   // rapid 10 + 5 down... then plunge 6 mm
    Move m;
    m.kind = Move::Kind::Feed; m.to = Vec3d(60, 0, -1); m.feed = 600;
    tp.moves.push_back(m);
    Move arc;
    arc.kind = Move::Kind::ArcCCW; arc.to = Vec3d(60, 0, -1); arc.center = Vec3d(50, 0, 0); arc.feed = 600;   // full circle r=10
    tp.moves.push_back(arc);
    append_retract(tp, h.clearance);
    toolpath_stats(tp);
    REQUIRE(tp.cut_length == Approx(6 + 60 + 2 * M_PI * 10));
    REQUIRE(tp.rapid_length == Approx(5 + 11));
    MachineProfile mach;
    mach.rapid_feed = 6000;
    mach.max_feed_z = 30;   // plunge clamped
    const double t = estimate_time(tp, mach);
    REQUIRE(t == Approx(6 / 30. * 60 + (60 + 2 * M_PI * 10) / 600. * 60 + 16 / 6000. * 60));

    // A-axis motion counts as arc length at the tool's distance from the axis
    Move a0; a0.to = Vec3d(0, 0, 10);
    Move a1; a1.kind = Move::Kind::Feed; a1.to = Vec3d(0, 0, 10); a1.a_deg = 90;
    REQUIRE(move_length(a0.to, 0, a1) == Approx(M_PI / 2 * 10));
    // short link hops at retract height
    Toolpath t2;
    append_link_move(t2, Vec3d(0, 0, -1), h, 5, fs);
    append_link_move(t2, Vec3d(3, 0, -1), h, 5, fs);
    REQUIRE(t2.moves.size() == 6);
    REQUIRE(t2.moves[3].kind == Move::Kind::Retract);
    REQUIRE(t2.moves[3].to.z() == Approx(5));
}
