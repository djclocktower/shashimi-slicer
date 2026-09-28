#include "libslic3r/CAM/CamGeometry.hpp"

#include "libslic3r/CAD/GeometryEngine.hpp"
#include "libslic3r/ClipperUtils.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_TangentialDeflection.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Wire.hxx>

#include <algorithm>
#include <cmath>

namespace Slic3r::CAM {

namespace {

constexpr double kAngularDeflection = 0.1;   // rad, discretisation of curves

Vec3d to_vec(const gp_Pnt& p) { return Vec3d(p.X(), p.Y(), p.Z()); }

// Points along an edge in its own orientation (REVERSED edges run Last -> First), world frame.
std::vector<Vec3d> edge_points(const TopoDS_Edge& edge, double tol)
{
    std::vector<Vec3d> out;
    if (BRep_Tool::Degenerated(edge))
        return out;
    BRepAdaptor_Curve curve(edge);
    if (curve.GetType() == GeomAbs_Line) {
        out = {to_vec(curve.Value(curve.FirstParameter())), to_vec(curve.Value(curve.LastParameter()))};
    } else {
        GCPnts_TangentialDeflection d(curve, kAngularDeflection, std::max(tol, 1e-4), 2);
        for (int i = 1; i <= d.NbPoints(); ++i)
            out.push_back(to_vec(d.Value(i)));
    }
    if (edge.Orientation() == TopAbs_REVERSED)
        std::reverse(out.begin(), out.end());
    return out;
}

// A wire's edges joined in wire order, world frame, no repeated joint points.
std::vector<Vec3d> wire_points(const TopoDS_Wire& wire, const TopoDS_Face* face, double tol)
{
    std::vector<Vec3d> out;
    BRepTools_WireExplorer ex;
    if (face)
        ex.Init(wire, *face);
    else
        ex.Init(wire);
    for (; ex.More(); ex.Next()) {
        std::vector<Vec3d> pts = edge_points(ex.Current(), tol);
        for (const Vec3d& p : pts)
            if (out.empty() || (p - out.back()).norm() > 1e-7)
                out.push_back(p);
    }
    return out;
}

Polygon to_polygon_xy(const std::vector<Vec3d>& pts, const Transform3d& t)
{
    Polygon poly;
    poly.points.reserve(pts.size());
    for (const Vec3d& p : pts) {
        const Vec3d q = t * p;
        poly.points.emplace_back(coord_t(scale_(q.x())), coord_t(scale_(q.y())));
    }
    if (poly.points.size() > 1 && poly.points.front() == poly.points.back())
        poly.points.pop_back();
    return poly;
}

// True when `p` (world) is inside the body's material.
bool material_at(const TopoDS_Shape& shape, const Vec3d& p)
{
    BRepClass3d_SolidClassifier cls(shape, gp_Pnt(p.x(), p.y(), p.z()), 1e-7);
    return cls.State() == TopAbs_IN;
}

} // namespace

ExPolygons face_outline(const CamBodyShape& body, int face_id, const Transform3d& to_setup, double& z_out)
{
    try {
        const std::vector<TopoDS_Face> faces = GeometryEngine::faces_of(body.shape);
        if (face_id < 0 || face_id >= int(faces.size()))
            return {};
        const TopoDS_Face& face = faces[face_id];
        BRepAdaptor_Surface surf(face);
        if (surf.GetType() != GeomAbs_Plane)
            return {};
        const gp_Dir n   = surf.Plane().Axis().Direction();
        const Vec3d  n_s = (to_setup.linear() * Vec3d(n.X(), n.Y(), n.Z())).normalized();
        if (std::abs(n_s.z()) < 1 - 1e-6)
            return {};
        const TopoDS_Wire outer = BRepTools::OuterWire(face);
        Polygons          contour, holes;
        for (TopExp_Explorer ex(face, TopAbs_WIRE); ex.More(); ex.Next()) {
            const TopoDS_Wire  w   = TopoDS::Wire(ex.Current());
            std::vector<Vec3d> pts = wire_points(w, &face, 0.005);
            if (pts.size() < 3)
                continue;
            z_out        = (to_setup * pts.front()).z();
            Polygon poly = to_polygon_xy(pts, to_setup);
            // Orientation-independent: every loop is made CCW and the outer one is kept apart.
            poly.make_counter_clockwise();
            (w.IsSame(outer) ? contour : holes).push_back(std::move(poly));
        }
        return diff_ex(union_ex(contour), holes);
    } catch (const Standard_Failure&) {
        return {};
    }
}

Polyline edge_polyline(const CamBodyShape& body, int edge_id, const Transform3d& to_setup, double tolerance, double& z_out)
{
    Polyline out;
    try {
        const std::vector<TopoDS_Edge> edges = GeometryEngine::edges_of(body.shape);
        if (edge_id < 0 || edge_id >= int(edges.size()))
            return out;
        TopoDS_Edge e = edges[edge_id];
        e.Orientation(TopAbs_FORWARD);
        std::vector<Vec3d> pts = edge_points(e, tolerance);
        z_out = 1e30;
        for (const Vec3d& p : pts) {
            const Vec3d q = to_setup * p;
            z_out         = std::min(z_out, q.z());
            out.points.emplace_back(coord_t(scale_(q.x())), coord_t(scale_(q.y())));
        }
        if (out.points.size() > 2 && BRep_Tool::IsClosed(e))
            out.points.back() = out.points.front();
        if (out.points.empty())
            z_out = 0;
    } catch (const Standard_Failure&) {
        out.clear();
    }
    return out;
}

std::vector<HoleFeature> find_holes(const CamBodyShape& body, const Transform3d& to_setup)
{
    std::vector<HoleFeature> holes;
    try {
        const std::vector<TopoDS_Face> faces = GeometryEngine::faces_of(body.shape);
        // Bore faces grouped by axis line + radius (a bore may be split into several faces).
        struct Bore { Vec3d p, dir; double r, t0, t1; int face; };
        std::vector<Bore> bores;
        for (int i = 0; i < int(faces.size()); ++i) {
            const GeometryEngine::CylinderFace cf = GeometryEngine::cylinder_of_face(faces[i]);
            if (!cf.ok || !cf.internal || cf.height <= 1e-6)
                continue;
            bool merged = false;
            for (Bore& b : bores) {
                const Vec3d d = cf.base - b.p;
                if (std::abs(cf.axis.dot(b.dir)) > 1 - 1e-6 && (d - b.dir * d.dot(b.dir)).norm() < 1e-4 &&
                    std::abs(cf.radius - b.r) < 1e-4) {
                    const double t0 = d.dot(b.dir), t1 = t0 + cf.height * cf.axis.dot(b.dir);
                    b.t0 = std::min(b.t0, std::min(t0, t1));
                    b.t1 = std::max(b.t1, std::max(t0, t1));
                    merged = true;
                    break;
                }
            }
            if (!merged)
                bores.push_back({cf.base, cf.axis, cf.radius, 0, cf.height, i});
        }
        for (const Bore& b : bores) {
            // An end is open (a mouth) when there is no material just past it. Probed near the
            // wall (0.95 r) and 0.1 r + 0.05 mm past the end, so a 118 deg drill-point bottom
            // (recess 0.03 r deep there) still reads as closed.
            const Vec3d  side  = b.dir.unitOrthogonal() * 0.95 * b.r;
            const double probe = 0.1 * b.r + 0.05;
            const Vec3d  e0 = b.p + b.dir * b.t0, e1 = b.p + b.dir * b.t1;
            const bool   open0 = !material_at(body.shape, e0 - b.dir * probe + side);
            const bool   open1 = !material_at(body.shape, e1 + b.dir * probe + side);
            if (!open0 && !open1)
                continue;   // a closed void, not machinable as a hole
            const Vec3d up_s = to_setup.linear() * b.dir;
            // The mouth: the open end; both open (through): the end facing setup +Z.
            const bool  top1 = open1 && (!open0 || up_s.z() >= 0);
            HoleFeature h;
            h.center   = to_setup * (top1 ? e1 : e0);
            h.axis     = (to_setup.linear() * (top1 ? b.dir : Vec3d(-b.dir))).normalized();
            h.diameter = 2 * b.r;
            h.depth    = b.t1 - b.t0;
            h.through  = open0 && open1;
            h.face     = b.face;
            holes.push_back(h);
        }
        // Circles bounding a planar face as a hole in it, with no bore behind them: spot positions.
        for (const TopoDS_Face& f : faces) {
            if (BRepAdaptor_Surface(f).GetType() != GeomAbs_Plane)
                continue;
            const TopoDS_Wire outer = BRepTools::OuterWire(f);
            for (TopExp_Explorer wx(f, TopAbs_WIRE); wx.More(); wx.Next()) {
                if (wx.Current().IsSame(outer))
                    continue;
                for (TopExp_Explorer ex(wx.Current(), TopAbs_EDGE); ex.More(); ex.Next()) {
                    const GeometryEngine::CylinderFace c = GeometryEngine::circle_of_edge(TopoDS::Edge(ex.Current()));
                    if (!c.ok)
                        continue;
                    const Vec3d center = to_setup * c.base;
                    bool        known  = false;   // coaxial with a bore (its mouth, a countersink, ...)
                    for (const HoleFeature& h : holes) {
                        const Vec3d d = center - h.center;
                        known |= (d - h.axis * d.dot(h.axis)).norm() < 1e-4;
                    }
                    const gp_Dir n0 = BRepAdaptor_Surface(f).Plane().Axis().Direction();
                    const Vec3d  n(n0.X(), n0.Y(), n0.Z());
                    // Material on either side of the circle's centre: a boss or a plug, not a hole.
                    if (known || material_at(body.shape, c.base + n * 0.05) || material_at(body.shape, c.base - n * 0.05))
                        continue;
                    Vec3d a = to_setup.linear() * n;
                    if (f.Orientation() == TopAbs_REVERSED)
                        a = -a;
                    HoleFeature h;
                    h.center   = center;
                    h.axis     = a.normalized();
                    h.diameter = 2 * c.radius;
                    holes.push_back(h);
                }
            }
        }
    } catch (const Standard_Failure&) {
    }
    return holes;
}

CamModel build_cam_model(const std::vector<TopoDS_Shape>& bodies, double linear_deflection, double angular_deflection)
{
    CamModel model;
    for (size_t i = 0; i < bodies.size(); ++i) {
        CamBody b;
        b.body_id = int(i);
        try {
            b.mesh = SketchEngine::tessellate(bodies[i], linear_deflection, angular_deflection);
        } catch (const Standard_Failure&) {
        }
        b.shape = std::make_shared<CamBodyShape>(CamBodyShape{bodies[i]});
        model.bodies.push_back(std::move(b));
    }
    return model;
}

void add_cam_sketch(CamModel& model, int feature_index, const std::vector<SketchEntity>& entities, const SketchPlane& plane)
{
    CamSketch s;
    s.feature = feature_index;
    s.origin  = plane.origin;
    s.x_axis  = plane.x_axis;
    s.y_axis  = plane.y_axis;
    std::vector<SketchEntity> curves;
    for (const SketchEntity& e : entities) {
        if (e.construction)
            continue;
        if (e.type == SketchEntity::Type::Point)
            s.points.push_back(e.p0);
        else
            curves.push_back(e);
    }
    // Wires come back in world coordinates; map them back into (u, v) of the plane.
    auto uv = [&](const Vec3d& p) {
        const Vec3d d = p - plane.origin;
        return Point(coord_t(scale_(d.dot(plane.x_axis))), coord_t(scale_(d.dot(plane.y_axis))));
    };
    try {
        Polygons loops;
        for (const TopoDS_Wire& w : SketchEngine::entities_to_wires(curves, plane, false)) {
            const std::vector<Vec3d> pts = wire_points(w, nullptr, 0.005);
            if (pts.size() < 2)
                continue;
            const bool closed = BRep_Tool::IsClosed(w) || (pts.size() > 2 && (pts.front() - pts.back()).norm() < kSketchJoinTol);
            if (closed) {
                Polygon poly;
                for (const Vec3d& p : pts)
                    poly.points.push_back(uv(p));
                if (poly.points.size() > 1 && poly.points.front() == poly.points.back())
                    poly.points.pop_back();
                if (poly.points.size() >= 3)
                    loops.push_back(std::move(poly));
            } else {
                Polyline pl;
                for (const Vec3d& p : pts)
                    pl.points.push_back(uv(p));
                s.chains.push_back(std::move(pl));
            }
        }
        // Nested loops alternate between material and hole.
        s.regions = union_ex(loops, ClipperLib::pftEvenOdd);
    } catch (const Standard_Failure&) {
    } catch (const std::exception&) {
    }
    model.sketches.push_back(std::move(s));
}

} // namespace Slic3r::CAM
