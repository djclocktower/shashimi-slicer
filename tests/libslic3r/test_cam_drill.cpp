#include <catch2/catch_all.hpp>

#include "libslic3r/CAM/CAM.hpp"
#include "libslic3r/CAM/CamGeometry.hpp"
#include "libslic3r/CAD/GeometryEngine.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <gp_Ax2.hxx>

using namespace Slic3r;
using namespace Slic3r::CAM;
using Catch::Matchers::WithinAbs;

namespace {

// 60 x 40 x 20 box at the origin with a vertical 8 mm hole at (20, 20): through, or 12 mm deep
// from the top.
TopoDS_Shape plate_with_hole(bool through, double d = 8)
{
    TopoDS_Shape box = BRepPrimAPI_MakeBox(60, 40, 20).Shape();
    const double z0  = through ? -1 : 8;
    TopoDS_Shape cyl = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(20, 20, z0), gp_Dir(0, 0, 1)), 0.5 * d, 30).Shape();
    return BRepAlgoAPI_Cut(box, cyl).Shape();
}

int face_index_of(const TopoDS_Shape& s, GeomAbs_SurfaceType type, double z = 1e9)
{
    const std::vector<TopoDS_Face> faces = GeometryEngine::faces_of(s);
    for (int i = 0; i < int(faces.size()); ++i) {
        BRepAdaptor_Surface surf(faces[i]);
        if (surf.GetType() != type) continue;
        if (z < 1e8 && std::abs(GeometryEngine::face_centroid_world(faces[i]).z() - z) > 1e-6) continue;
        return i;
    }
    return -1;
}

CamTool drill(double d)
{
    CamTool t;
    t.type          = ToolType::Drill;
    t.diameter      = d;
    t.tip_angle_deg = 118;
    return t;
}

// Doc with one drill, one setup (WCS: front-left top of the model, no stock offsets), one op.
struct Fixture {
    CamDocument doc;
    CamModel    model;
    Fixture(const TopoDS_Shape& shape, const CamTool& tool, OpType type = OpType::Drill)
    {
        model = build_cam_model({shape}, 0.05);
        doc.add_tool(tool);
        CamSetup s;
        s.stock.offset_neg = s.stock.offset_pos = Vec3d::Zero();
        doc.add_setup(s);
        update_setup_frames(model, doc);
        CamOperation op = default_operation(type, &doc.tools[0]);
        op.geom.whole_model = true;
        doc.add_operation(op);
    }
    CamOperation& op() { return doc.operations[0]; }
};

std::vector<double> z_sequence(const Toolpath& tp)
{
    std::vector<double> zs;
    for (const Move& m : tp.moves)
        if (m.cycle >= 0) zs.push_back(m.to.z());
    return zs;
}

} // namespace

TEST_CASE("Face outline of a box face is its rectangle", "[CamDrill]")
{
    const TopoDS_Shape box = BRepPrimAPI_MakeBox(gp_Pnt(5, 10, 0), 30, 20, 8).Shape();
    const CamBodyShape body{box};
    const int top = face_index_of(box, GeomAbs_Plane, 8);
    REQUIRE(top >= 0);
    double z = 0;
    const ExPolygons ex = face_outline(body, top, Transform3d::Identity(), z);
    REQUIRE(ex.size() == 1);
    REQUIRE(ex[0].holes.empty());
    REQUIRE_THAT(z, WithinAbs(8, 1e-9));
    REQUIRE_THAT(unscale<double>(unscale<double>(ex[0].area())), WithinAbs(600, 1e-6));
    const BoundingBox bb = get_extents(ex);
    REQUIRE_THAT(unscale<double>(bb.min.x()), WithinAbs(5, 1e-6));
    REQUIRE_THAT(unscale<double>(bb.max.y()), WithinAbs(30, 1e-6));
    // a side face is not a 2D face
    const int side = face_index_of(box, GeomAbs_Plane, 4);
    REQUIRE(face_outline(body, side, Transform3d::Identity(), z).empty());

    SECTION("a face with a hole keeps the hole") {
        const TopoDS_Shape plate = plate_with_hole(true);
        const int pt = face_index_of(plate, GeomAbs_Plane, 20);
        const ExPolygons ex2 = face_outline(CamBodyShape{plate}, pt, Transform3d::Identity(), z);
        REQUIRE(ex2.size() == 1);
        REQUIRE(ex2[0].holes.size() == 1);
        REQUIRE_THAT(unscale<double>(unscale<double>(ex2[0].area())), WithinAbs(2400 - M_PI * 16, 0.5));
    }
}

TEST_CASE("find_holes recognises a drilled hole", "[CamDrill]")
{
    for (bool through : {false, true}) {
        const std::vector<HoleFeature> holes = find_holes(CamBodyShape{plate_with_hole(through)}, Transform3d::Identity());
        REQUIRE(holes.size() == 1);
        const HoleFeature& h = holes[0];
        REQUIRE_THAT(h.diameter, WithinAbs(8, 1e-6));
        REQUIRE_THAT(h.depth, WithinAbs(through ? 20 : 12, 1e-6));
        REQUIRE(h.through == through);
        REQUIRE_THAT(h.axis.z(), WithinAbs(1, 1e-9));
        REQUIRE_THAT((h.center - Vec3d(20, 20, 20)).norm(), WithinAbs(0, 1e-6));
    }
    // flipped part: the hole now opens downward
    Transform3d flip = Transform3d::Identity();
    flip.rotate(Eigen::AngleAxisd(M_PI, Vec3d::UnitX()));
    const std::vector<HoleFeature> down = find_holes(CamBodyShape{plate_with_hole(false)}, flip);
    REQUIRE(down.size() == 1);
    REQUIRE_THAT(down[0].axis.z(), WithinAbs(-1, 1e-9));
}

TEST_CASE("Drill cycles produce the expected Z sequence", "[CamDrill]")
{
    Fixture f(plate_with_hole(false), drill(8));
    // WCS at the stock's front-left top: hole top at Z 0, 12 deep, R plane at 2
    f.op().heights.retract = {HeightRef::StockTop, 2};

    SECTION("drill (G81)") {
        f.op().cycle = DrillCycle::Drill;
        const Toolpath tp = generate_toolpath(f.doc, 0, f.model);
        REQUIRE(tp.ok());
        REQUIRE(z_sequence(tp) == std::vector<double>{-12, 2});
        // positioned over the hole at the R plane first
        const auto it = std::find_if(tp.moves.begin(), tp.moves.end(), [](const Move& m) { return m.cycle >= 0; });
        REQUIRE_THAT((it - 1)->to.z(), WithinAbs(2, 1e-9));
        REQUIRE_THAT((it - 1)->to.x(), WithinAbs(20, 1e-6));
        REQUIRE(tp.moves.back().to.z() == 10);
    }
    SECTION("peck (G83): 12 mm in 4 mm pecks is 3 pecks with full retracts") {
        f.op().cycle      = DrillCycle::Peck;
        f.op().peck_depth = 4;
        const Toolpath tp = generate_toolpath(f.doc, 0, f.model);
        REQUIRE(tp.ok());
        const std::vector<double> zs = z_sequence(tp);
        REQUIRE(zs.size() == 8);
        const std::vector<double> want{-4, 2, -3.5, -8, 2, -7.5, -12, 2};
        for (size_t i = 0; i < zs.size(); ++i) REQUIRE_THAT(zs[i], WithinAbs(want[i], 1e-9));
        int plunges = 0;
        for (const Move& m : tp.moves) plunges += m.kind == Move::Kind::Plunge;
        REQUIRE(plunges == 3);
    }
    SECTION("chip break (G73): short retracts") {
        f.op().cycle      = DrillCycle::ChipBreak;
        f.op().peck_depth = 5;
        const std::vector<double> zs = z_sequence(generate_toolpath(f.doc, 0, f.model));
        const std::vector<double> want{-5, -4.8, -10, -9.8, -12, 2};
        REQUIRE(zs.size() == want.size());
        for (size_t i = 0; i < zs.size(); ++i) REQUIRE_THAT(zs[i], WithinAbs(want[i], 1e-9));
    }
    SECTION("tap (G84): feed = rpm x pitch, in and out") {
        CamTool tap;
        tap.type = ToolType::Tap; tap.diameter = 8; tap.thread_pitch = 1.25;
        f.doc.tools[0] = tap;
        f.doc.tools[0].number = 1;
        f.op().cycle = DrillCycle::Tap;
        const Toolpath tp = generate_toolpath(f.doc, 0, f.model);
        REQUIRE(tp.ok());
        const FeedsSpeeds fs = recommend_feeds(tap, Material::Aluminum, find_machine("Generic 3-axis"));
        for (const Move& m : tp.moves)
            if (m.cycle >= 0) REQUIRE_THAT(m.feed, WithinAbs(fs.rpm * 1.25, 1e-6));
    }
    SECTION("a through hole gets the break-through and the drill point") {
        Fixture t(plate_with_hole(true), drill(8));
        t.op().cycle = DrillCycle::Drill;
        t.op().break_through = 0.5;
        const Toolpath tp = generate_toolpath(t.doc, 0, t.model);
        REQUIRE_THAT(z_sequence(tp)[0], WithinAbs(-20 - 0.5 - 4 / std::tan(59 * M_PI / 180), 1e-6));
    }
    SECTION("a spot drill cuts the chamfer depth of the hole") {
        CamTool spot;
        spot.type = ToolType::SpotDrill; spot.diameter = 10; spot.tip_angle_deg = 90;
        f.doc.tools[0] = spot;
        f.doc.tools[0].number = 1;
        f.op().cycle         = DrillCycle::Drill;
        f.op().chamfer_width = 0.5;
        const std::vector<double> zs = z_sequence(generate_toolpath(f.doc, 0, f.model));
        REQUIRE_THAT(zs[0], WithinAbs(-4.5, 1e-9));   // (8 / 2 + 0.5) / tan(45 deg)
    }
    SECTION("the hole diameter filter") {
        f.op().hole_diameter_min = 9;
        REQUIRE(generate_toolpath(f.doc, 0, f.model).error.find("diameter filter") != std::string::npos);
        f.op().hole_diameter_min = 0;
        f.op().hole_diameter_max = 7;
        REQUIRE(holes_for_op(f.doc, f.op(), f.model).empty());
        f.op().hole_diameter_min = 7.9;
        f.op().hole_diameter_max = 8.1;
        REQUIRE(holes_for_op(f.doc, f.op(), f.model).size() == 1);
    }
    SECTION("a drill larger than the hole is refused") {
        f.doc.tools[0].diameter = 10;
        const Toolpath tp = generate_toolpath(f.doc, 0, f.model);
        REQUIRE_FALSE(tp.ok());
        REQUIRE(tp.error.find("larger than the 8 mm hole") != std::string::npos);
    }
}

TEST_CASE("Holes come from faces, edges and points, nearest first", "[CamDrill]")
{
    Fixture f(plate_with_hole(false), drill(6));
    CamOperation& op = f.op();
    op.geom.whole_model = false;
    // the bore face selects its hole
    op.geom.faces = {{0, face_index_of(f.model.bodies[0].shape->shape, GeomAbs_Cylinder)}};
    REQUIRE(holes_for_op(f.doc, op, f.model).size() == 1);
    // the top face selects every hole opening in it
    op.geom.faces = {{0, face_index_of(f.model.bodies[0].shape->shape, GeomAbs_Plane, 20)}};
    std::vector<HoleFeature> holes = holes_for_op(f.doc, op, f.model);
    REQUIRE(holes.size() == 1);
    REQUIRE_THAT(holes[0].depth, WithinAbs(12, 1e-6));
    // picked points, sorted by nearest neighbour from the origin
    op.geom.faces.clear();
    op.geom.points = {Vec3d(50, 30, 0), Vec3d(5, 5, 0), Vec3d(45, 5, 0)};
    holes = holes_for_op(f.doc, op, f.model);
    REQUIRE(holes.size() == 3);
    REQUIRE_THAT(holes[0].center.x(), WithinAbs(5, 1e-9));
    REQUIRE_THAT(holes[1].center.x(), WithinAbs(45, 1e-9));
    // points need a depth from the Bottom height
    op.heights.bottom = {HeightRef::Absolute, -3};
    const Toolpath tp = generate_toolpath(f.doc, 0, f.model);
    REQUIRE(tp.ok());
    int cycles = 0;
    for (const Move& m : tp.moves) cycles += m.kind == Move::Kind::Plunge && m.cycle >= 0 && std::abs(m.to.z() + 3) < 1e-9;
    REQUIRE(cycles == 3);
}

TEST_CASE("Helical bore stays inside the hole", "[CamDrill]")
{
    CamTool em;
    em.diameter = 6;
    Fixture f(plate_with_hole(false, 12), em, OpType::Bore);
    f.op().stepdown = 2;
    const Toolpath tp = generate_toolpath(f.doc, 0, f.model);
    REQUIRE(tp.ok());
    int arcs = 0;
    double zmin = 0;
    for (size_t i = 1; i < tp.moves.size(); ++i) {
        const Move& m = tp.moves[i];
        if (!is_arc(m)) continue;
        ++arcs;
        REQUIRE_THAT((m.to.head<2>() - Vec2d(20, 20)).norm(), WithinAbs(3, 1e-6));   // (12 - 6) / 2
        REQUIRE(m.to.z() <= tp.moves[i - 1].to.z() + 1e-9);
        zmin = std::min(zmin, m.to.z());
    }
    REQUIRE(arcs == 12 + 2);   // 12 mm at 2 mm per turn = 12 half turns, then one flat turn
    REQUIRE_THAT(zmin, WithinAbs(-12, 1e-9));
}
