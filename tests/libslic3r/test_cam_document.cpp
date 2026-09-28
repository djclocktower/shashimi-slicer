#include <catch2/catch_all.hpp>

#include "libslic3r/CAM/CAM.hpp"

#include "test_utils.hpp"

using namespace Slic3r;
using namespace Slic3r::CAM;
using Catch::Approx;

namespace {

CamModel box_model(double x, double y, double z, Vec3d at = Vec3d::Zero())
{
    CamModel m;
    CamBody  b;
    b.body_id = 0;
    b.mesh    = TriangleMesh(its_make_cube(x, y, z));
    b.mesh.translate(at.cast<float>());
    m.bodies.push_back(std::move(b));
    return m;
}

CamDocument sample_doc()
{
    CamDocument doc;
    CamTool t;
    t.number = 3;
    t.name   = "6 mm flat";
    t.flutes = 3;
    doc.add_tool(t);
    t.name = "dup number";
    doc.add_tool(t);   // renumbered to 4
    CamSetup s;
    s.material        = Material::MDF;
    s.stock.kind      = StockKind::Cylinder;
    s.wcs.point       = WcsPoint::Center;
    s.body_ids        = {0, 2};
    s.a_index_deg     = 90;
    doc.add_setup(s);
    CamOperation op = default_operation(OpType::Contour2D, &doc.tools[0]);
    op.geom.faces   = {{0, 5}, {0, 7}};
    op.geom.points  = {Vec3d(1, 2, 3)};
    op.side         = ContourSide::Inside;
    op.heights.bottom = {HeightRef::Absolute, -4.5};
    op.hole_diameter_min = 3;
    op.hole_diameter_max = 8.5;
    doc.add_operation(op);
    return doc;
}

} // namespace

TEST_CASE("CamDocument round-trips through its blob", "[CamDocument]")
{
    const CamDocument doc = sample_doc();
    REQUIRE(doc.tools[1].number == 4);

    CamDocument back;
    REQUIRE(back.deserialize(doc.serialize()));
    REQUIRE(back.tools.size() == 2);
    REQUIRE(back.tools[0].name == "6 mm flat");
    REQUIRE(back.tools[0].flutes == 3);
    REQUIRE(back.setups.size() == 1);
    REQUIRE(back.setups[0].material == Material::MDF);
    REQUIRE(back.setups[0].stock.kind == StockKind::Cylinder);
    REQUIRE(back.setups[0].body_ids == std::vector<int>{0, 2});
    REQUIRE(back.setups[0].a_index_deg == 90);
    REQUIRE(back.operations.size() == 1);
    REQUIRE(back.operations[0].side == ContourSide::Inside);
    REQUIRE(back.operations[0].geom.faces.size() == 2);
    REQUIRE(back.operations[0].geom.faces[1].face == 7);
    REQUIRE(back.operations[0].geom.points[0] == Vec3d(1, 2, 3));
    REQUIRE(back.operations[0].heights.bottom.offset == -4.5);
    REQUIRE(back.operations[0].hole_diameter_min == 3);
    REQUIRE(back.operations[0].hole_diameter_max == 8.5);
    REQUIRE(back.paths.size() == 1);
    REQUIRE(back.warnings.empty());
    REQUIRE(back.serialize() == doc.serialize());

    SECTION("empty blob is an empty document") {
        REQUIRE(back.deserialize(""));
        REQUIRE(back.operations.empty());
    }
    SECTION("garbage is rejected") {
        REQUIRE_FALSE(back.deserialize(std::string("\x01\0\0\0\xff\xff\xff\x7f", 8)));
        REQUIRE(back.tools.empty());
    }
    SECTION("an item with trailing bytes (a newer build) still reads") {
        std::vector<CamTool> tools = doc.tools;
        std::string          block = cam_encode_items(tools);
        std::vector<CamTool> got;
        REQUIRE(cam_decode_items(block, got));
        REQUIRE(got.size() == 2);
    }
    SECTION("an op naming a missing tool is reported") {
        CamDocument d = doc;
        d.operations[0].tool_number = 42;
        REQUIRE(back.deserialize(d.serialize()));
        REQUIRE(back.warnings.size() == 1);
    }
}

TEST_CASE("CamDocument keeps operations grouped by setup", "[CamDocument]")
{
    CamDocument doc;
    doc.add_setup(CamSetup{});
    doc.add_setup(CamSetup{});
    CamOperation a;  a.name = "a";  a.setup_index = 1;
    CamOperation b;  b.name = "b";  b.setup_index = 0;
    CamOperation c;  c.name = "c";  c.setup_index = 1;
    REQUIRE(doc.add_operation(a) == 0);
    REQUIRE(doc.add_operation(b) == 0);   // setup 0 goes before setup 1
    REQUIRE(doc.add_operation(c) == 2);
    CamOperation bad; bad.setup_index = 5;
    REQUIRE(doc.add_operation(bad) == -1);
    REQUIRE(doc.paths.size() == 3);

    REQUIRE(doc.move_operation(2, 0) == 1);   // clamped into setup 1's group
    REQUIRE(doc.operations[1].name == "c");
    REQUIRE(doc.duplicate_operation(1) == 2);
    REQUIRE(doc.operations[2].name == "c copy");

    REQUIRE(doc.remove_setup(0));
    REQUIRE(doc.operations.size() == 3);
    for (const CamOperation& op : doc.operations)
        REQUIRE(op.setup_index == 0);
    REQUIRE(doc.remove_operation(0));
    REQUIRE(doc.paths.size() == 2);
}

TEST_CASE("Setup frame: box stock from the model bounds, WCS anchors", "[CamDocument]")
{
    CamModel model = box_model(100, 60, 20, Vec3d(10, 20, 5));
    CamSetup s;   // offsets (1,1,0)/(1,1,1), WCS front-left top of stock
    CamSetupFrame f = compute_setup_frame(s, model);
    REQUIRE(f.stock.min.x() == Approx(0));
    REQUIRE(f.stock.min.y() == Approx(0));
    REQUIRE(f.stock.max.z() == Approx(0));
    REQUIRE(f.stock.max.x() == Approx(102));
    REQUIRE(f.stock.max.y() == Approx(62));
    REQUIRE(f.stock.min.z() == Approx(-21));
    REQUIRE(f.model.max.z() == Approx(-1));
    REQUIRE(f.model.min.x() == Approx(1));
    // world (10,20,5) is the model's min corner
    REQUIRE((f.to_setup * Vec3d(10, 20, 5) - Vec3d(1, 1, -21)).norm() == Approx(0).margin(1e-9));

    s.wcs.point = WcsPoint::Center;
    s.wcs.z     = WcsZ::Bottom;
    s.wcs.box   = WcsBox::Model;
    f = compute_setup_frame(s, model);
    REQUIRE(f.model.min.z() == Approx(0));
    REQUIRE(f.model.min.x() == Approx(-50));
    REQUIRE(f.model.max.y() == Approx(30));

    s.wcs.custom       = true;
    s.wcs.custom_point = Vec3d(10, 20, 5);
    f = compute_setup_frame(s, model);
    REQUIRE(f.model.min.norm() == Approx(0).margin(1e-9));

    SECTION("flipped (a_index 180): the model bottom faces up") {
        s.wcs        = WcsOrigin{};
        s.a_index_deg = 180;
        f = compute_setup_frame(s, model);
        // the part is the same size; the world bottom (z=5) is now the top
        REQUIRE(f.model.size().z() == Approx(20));
        const Vec3d bottom_pt = f.to_setup * Vec3d(60, 50, 5);
        REQUIRE(bottom_pt.z() == Approx(f.model.max.z()));
    }

    SECTION("setup mesh is in the setup frame") {
        CamDocument doc;
        doc.add_setup(CamSetup{});
        update_setup_frames(model, doc);
        TriangleMesh m = setup_mesh(doc, model, 0);
        REQUIRE(m.bounding_box().max.z() == Approx(-1));
    }
}

TEST_CASE("Setup frame: cylinder stock sits on the A axis", "[CamDocument]")
{
    CamModel model = box_model(80, 20, 20, Vec3d(0, -10, 40));
    CamSetup s;
    s.stock.kind       = StockKind::Cylinder;
    s.stock.offset_pos = Vec3d(2, 1, 1);
    s.stock.offset_neg = Vec3d(3, 0, 0);
    s.wcs.point        = WcsPoint::Left;
    CamSetupFrame f = compute_setup_frame(s, model);
    const double r = std::sqrt(200.) + 1;
    REQUIRE(f.stock_radius == Approx(r));
    REQUIRE(f.stock.min.y() == Approx(-r));
    REQUIRE(f.stock.max.z() == Approx(r));
    REQUIRE(f.stock.min.x() == Approx(0));
    REQUIRE(f.stock.max.x() == Approx(85));
    // world axis (y=0, z=50) maps to setup Y=Z=0
    const Vec3d p = f.to_setup * Vec3d(40, 0, 50);
    REQUIRE(p.y() == Approx(0).margin(1e-9));
    REQUIRE(p.z() == Approx(0).margin(1e-9));
}

TEST_CASE("Heights resolve against stock, model and selection", "[CamDocument]")
{
    CamSetupFrame f;
    f.stock = BoundingBoxf3(Vec3d(0, 0, -20), Vec3d(100, 100, 0));
    f.model = BoundingBoxf3(Vec3d(1, 1, -20), Vec3d(99, 99, -1));
    Heights h;
    h.bottom = {HeightRef::SelectionBottom, -0.5};
    ResolvedHeights r = resolve_heights(h, f, -2, -8);
    REQUIRE(r.clearance == Approx(10));
    REQUIRE(r.retract == Approx(5));
    REQUIRE(r.top == Approx(0));
    REQUIRE(r.bottom == Approx(-8.5));
    h.top     = {HeightRef::ModelTop, 0};
    h.retract = {HeightRef::Absolute, -5};   // below the top: lifted to it
    r = resolve_heights(h, f, -2, -8);
    REQUIRE(r.top == Approx(-1));
    REQUIRE(r.retract == Approx(-1));
}

TEST_CASE("Default tools and the JSON tool library", "[CamDocument]")
{
    const std::vector<CamTool> tools = default_tools();
    REQUIRE(tools.size() == 18);
    for (size_t i = 0; i < tools.size(); ++i)
        REQUIRE(tools[i].number == int(i) + 1);
    REQUIRE(tools[0].diameter == Approx(3.175));
    REQUIRE(tools[3].name == "6 mm flat end mill");
    REQUIRE(tools.back().type == ToolType::Drill);
    REQUIRE(tools.back().diameter == 10);

    ScopedTemporaryFile file(".json");
    std::vector<CamTool> saved = tools;
    saved[2].override_feeds = true;
    saved[2].feeds.rpm      = 1234;
    REQUIRE(save_tool_library(file.string(), saved));
    std::vector<CamTool> back;
    REQUIRE(load_tool_library(file.string(), back));
    REQUIRE(back.size() == saved.size());
    REQUIRE(back[9].type == ToolType::VBit);
    REQUIRE(back[9].tip_angle_deg == 60);
    REQUIRE(back[2].override_feeds);
    REQUIRE(back[2].feeds.rpm == 1234);
    REQUIRE(back[11].material == ToolMaterial::HSS);

    std::string err;
    REQUIRE_FALSE(load_tool_library(file.string() + ".missing", back, &err));
    REQUIRE_FALSE(err.empty());
}

TEST_CASE("Machine profiles: built-ins match resources/cam/machines.json", "[CamDocument]")
{
    const std::vector<MachineProfile> builtin = builtin_machines();
    REQUIRE(builtin.size() == 6);
    const std::vector<MachineProfile> loaded = load_machines(std::string(PROFILES_DIR) + "/..");
    REQUIRE(loaded.size() == builtin.size());
    for (size_t i = 0; i < builtin.size(); ++i) {
        REQUIRE(loaded[i].name == builtin[i].name);
        REQUIRE(loaded[i].max_rpm == builtin[i].max_rpm);
        REQUIRE(loaded[i].min_rpm == builtin[i].min_rpm);
        REQUIRE(loaded[i].max_feed_xy == builtin[i].max_feed_xy);
        REQUIRE(loaded[i].rapid_feed == builtin[i].rapid_feed);
        REQUIRE(loaded[i].travel == builtin[i].travel);
        REQUIRE(loaded[i].post == builtin[i].post);
        REQUIRE(loaded[i].tool_change == builtin[i].tool_change);
        REQUIRE(loaded[i].has_a_axis == builtin[i].has_a_axis);
        REQUIRE(loaded[i].coolant == builtin[i].coolant);
    }
    REQUIRE(load_machines("/nonexistent").size() == builtin.size());
    REQUIRE(find_machine("LinuxCNC mill").post == PostDialect::LinuxCNC);
    REQUIRE(find_machine("no such machine").name == "Generic 3-axis");
}

TEST_CASE("Feeds and speeds stay inside the machine and scale with the tool", "[CamDocument]")
{
    for (const MachineProfile& mach : builtin_machines())
        for (int mi = 0; mi < kMaterialCount; ++mi) {
            const Material mat = Material(mi);
            REQUIRE(std::string(material_name(mat)).size() > 2);
            double prev_rpm = 1e9, prev_chip = 0;
            for (double d : {1.0, 2.0, 3.175, 4.0, 6.0, 8.0, 10.0, 12.0, 16.0, 20.0}) {
                CamTool t;
                t.diameter = d;
                t.flutes   = 2;
                const FeedsSpeeds fs = recommend_feeds(t, mat, mach);
                REQUIRE(fs.rpm <= mach.max_rpm);
                REQUIRE(fs.rpm >= mach.min_rpm);
                REQUIRE(fs.feed > 0);
                REQUIRE(fs.feed <= mach.max_feed_xy);
                REQUIRE(fs.plunge_feed <= mach.max_feed_z);
                REQUIRE(fs.plunge_feed <= fs.feed);
                REQUIRE(fs.plunge_feed >= 0.29 * std::min(fs.feed, mach.max_feed_z / 0.5));
                REQUIRE(fs.ramp_feed == Approx(0.5 * fs.feed).margin(1));
                // bigger tools: never faster spindle, never smaller chipload
                REQUIRE(fs.rpm <= prev_rpm);
                REQUIRE(fs.chipload >= prev_chip);
                prev_rpm  = fs.rpm;
                prev_chip = fs.chipload;
            }
        }
    MachineProfile big;   // unlimited-ish machine: pure formula
    big.max_rpm = 1e6; big.max_feed_xy = 1e6; big.max_feed_z = 1e6;
    CamTool t;   // 6 mm 2-flute carbide in aluminium: 200 m/min, 0.04 mm/tooth
    const FeedsSpeeds fs = recommend_feeds(t, Material::Aluminum, big);
    REQUIRE(fs.rpm == Approx(200000 / (M_PI * 6)).margin(1));
    REQUIRE(fs.feed == Approx(fs.rpm * 2 * 0.04).margin(1));
    CamTool tap;
    tap.type = ToolType::Tap; tap.diameter = 6; tap.thread_pitch = 1.0;
    const FeedsSpeeds tf = recommend_feeds(tap, Material::Aluminum, big);
    REQUIRE(tf.feed == Approx(tf.rpm * 1.0));

    CamOperation op;
    t.override_feeds = true;
    t.feeds.rpm      = 777;
    REQUIRE(effective_feeds(op, t, Material::MDF, big).rpm == 777);
    op.feeds_auto = false;
    op.feeds.rpm  = 555;
    REQUIRE(effective_feeds(op, t, Material::MDF, big).rpm == 555);
}

TEST_CASE("Toolpaths go stale when the model or the operation changes", "[CamDocument]")
{
    CamDocument doc = sample_doc();
    CamModel    model;
    CamSketch   sk;
    sk.feature = 1;
    sk.regions = {ExPolygon(Polygon::new_scale({{0, 0}, {30, 0}, {30, 20}, {0, 20}}))};
    model.sketches.push_back(sk);
    doc.setups[0].stock.kind = StockKind::Box;
    doc.setups[0].a_index_deg = 0;
    doc.operations[0].geom = GeometrySelection{};
    doc.operations[0].geom.sketch_feature = 1;
    doc.operations[0].heights.bottom = {HeightRef::StockTop, -2};
    doc.operations[0].stepdown = 0.5;   // 4 levels: 4 progress reports + the final 1
    update_setup_frames(model, doc);

    REQUIRE(doc.is_stale(0));
    doc.paths[0] = generate_toolpath(doc, 0, model);
    REQUIRE(doc.paths[0].ok());
    REQUIRE_FALSE(doc.is_stale(0));
    doc.mark_model_changed();
    REQUIRE(doc.is_stale(0));
    doc.paths[0] = generate_toolpath(doc, 0, model);
    REQUIRE_FALSE(doc.is_stale(0));
    doc.invalidate(0);
    REQUIRE(doc.is_stale(0));
    REQUIRE(doc.is_stale(7));

    SECTION("progress is monotone and ends at 1; cancelling empties the result") {
        std::vector<double> seen;
        const Toolpath tp = generate_toolpath(doc, 0, model, [&](double f) { seen.push_back(f); return false; });
        REQUIRE(tp.ok());
        REQUIRE(seen.size() >= 2);
        REQUIRE(std::is_sorted(seen.begin(), seen.end()));
        REQUIRE(seen.back() == 1.0);
        int calls = 0;
        const Toolpath cancelled = generate_toolpath(doc, 0, model, [&](double) { return ++calls >= 2; });
        REQUIRE(cancelled.error == "Cancelled");
        REQUIRE(cancelled.moves.empty());
        REQUIRE(calls == 2);
    }
    SECTION("adaptive and 3D generators honour cancel") {
        doc.operations[0].type = OpType::Adaptive2D;
        REQUIRE(generate_toolpath(doc, 0, model, [](double) { return true; }).error == "Cancelled");
        CamBody b;
        b.body_id = 0;
        b.mesh    = TriangleMesh(its_make_cube(30, 20, 10));
        model.bodies.push_back(std::move(b));
        update_setup_frames(model, doc);
        doc.operations[0] = default_operation(OpType::Parallel3D, &doc.tools[0]);
        REQUIRE(generate_toolpath(doc, 0, model).ok());
        REQUIRE(generate_toolpath(doc, 0, model, [](double) { return true; }).error == "Cancelled");
    }
}

TEST_CASE("World points map to the part frame of an indexed setup", "[CamDocument]")
{
    CamModel model = box_model(100, 60, 20, Vec3d(10, 20, 5));
    CamSetup s;
    s.a_index_deg      = 90;
    const Vec3d world(30, 40, 25);   // a point picked on the model
    s.wcs.custom       = true;
    s.wcs.custom_point = world_to_part_frame(s, model, world);
    const CamSetupFrame f = compute_setup_frame(s, model);
    REQUIRE((f.to_setup * world).norm() == Approx(0).margin(1e-9));
    s.a_index_deg = 0;
    REQUIRE((world_to_part_frame(s, model, world) - world).norm() == Approx(0).margin(1e-12));
}
