#include <catch2/catch_all.hpp>

#include <algorithm>

#include "libslic3r/Laser/LaserDocument.hpp"
#include "libslic3r/Laser/Materials.hpp"

#include "test_utils.hpp"

using namespace Slic3r;
using namespace Slic3r::Laser;
using Catch::Approx;

namespace {

LaserShape make(ShapeType t)
{
    LaserShape s;
    s.type = t;
    return s;
}

// One of every shape type, a group holding the rect and the ellipse.
LaserDocument sample_doc()
{
    LaserDocument doc;
    LaserShape path = make(ShapeType::Path);
    path.paths.push_back({Polyline(Points{{0, 0}, {Point::new_scale(10, 0)}, {Point::new_scale(10, 5)}}), false});
    path.layer = 3;
    doc.add_shape(path);
    LaserShape rect = make(ShapeType::Rect);
    rect.width = 20; rect.height = 10; rect.corner_radius = 2;
    rect.xform.translation() = Vec2d(50, 50);
    doc.add_shape(rect);
    LaserShape ell = make(ShapeType::Ellipse);
    ell.rx = 8; ell.ry = 4;
    doc.add_shape(ell);
    LaserShape poly = make(ShapeType::Polygon);
    poly.sides = 5;
    doc.add_shape(poly);
    LaserShape text = make(ShapeType::Text);
    text.text = "Hi\nthere";
    text.font = "/nonexistent.ttf";
    text.text_outlines.push_back(Polygon(Points{{0, 0}, Point::new_scale(3, 0), Point::new_scale(3, 7)}));
    text.bold = true;
    text.text_align = TextAlign::Center;
    doc.add_shape(text);
    LaserShape img = make(ShapeType::Image);
    img.image_w = 3; img.image_h = 2; img.gray = {0, 50, 100, 150, 200, 255};
    img.width_mm = 30; img.height_mm = 20; img.contrast = 12; img.dither_override = true; img.dither = DitherMode::Stucki;
    doc.add_shape(img);
    doc.group({1, 2});
    doc.layers[5].mode = LayerMode::Fill;
    doc.layers[5].speed_mm_s = 321;
    doc.device_name = "My laser";
    doc.job.start_from = JobSettings::StartFrom::UserOrigin;
    doc.job.job_origin = 4;
    doc.job.optimize.order_by = JobSettings::OrderBy::Priority;
    return doc;
}

BoundingBoxf bbox(const LaserPaths& paths)
{
    BoundingBoxf bb;
    for (const LaserPath& p : paths)
        for (const Point& pt : p.pts.points) bb.merge(unscale(pt));
    return bb;
}

} // namespace

TEST_CASE("Laser document round-trips every shape type", "[LaserDoc]")
{
    const LaserDocument doc = sample_doc();
    const std::string   blob = doc.serialize();
    LaserDocument       back;
    REQUIRE(back.deserialize(blob));
    REQUIRE(back.warnings.empty());
    REQUIRE(back.shapes.size() == doc.shapes.size());
    for (size_t i = 0; i < doc.shapes.size(); ++i) {
        const LaserShape &a = doc.shapes[i], &b = back.shapes[i];
        CHECK(a.type == b.type);
        CHECK(a.id == b.id);
        CHECK(a.parent == b.parent);
        CHECK(a.layer == b.layer);
        CHECK(a.xform.matrix().isApprox(b.xform.matrix()));
        CHECK(a.width == b.width);
        CHECK(a.corner_radius == b.corner_radius);
        CHECK(a.rx == b.rx);
        CHECK(a.sides == b.sides);
        CHECK(a.text == b.text);
        CHECK(a.bold == b.bold);
        CHECK(a.text_align == b.text_align);
        CHECK(a.text_outlines == b.text_outlines);
        CHECK(a.gray == b.gray);
        CHECK(a.width_mm == b.width_mm);
        CHECK(a.contrast == b.contrast);
        CHECK(a.dither == b.dither);
        REQUIRE(a.paths.size() == b.paths.size());
        for (size_t k = 0; k < a.paths.size(); ++k) {
            CHECK(a.paths[k].pts.points == b.paths[k].pts.points);
            CHECK(a.paths[k].closed == b.paths[k].closed);
        }
    }
    CHECK(back.layers[5].mode == LayerMode::Fill);
    CHECK(back.layers[5].speed_mm_s == 321);
    CHECK(back.layers[7].name == "C07");
    CHECK(back.device_name == "My laser");
    CHECK(back.job.start_from == JobSettings::StartFrom::UserOrigin);
    CHECK(back.job.job_origin == 4);
    CHECK(back.job.optimize.order_by == JobSettings::OrderBy::Priority);
    // Ids continue after the loaded ones.
    const int n = back.add_shape(make(ShapeType::Rect));
    CHECK(back.shapes[n].id > doc.shapes.back().id);

    SECTION("empty and corrupt blobs")
    {
        LaserDocument d;
        CHECK(d.deserialize(""));
        CHECK(d.empty());
        CHECK_FALSE(d.deserialize(std::string("\x01\x00\x00\x00\xff\xff\xff\x7f", 8)));
        CHECK(d.empty());
        // Truncated blob.
        CHECK_FALSE(d.deserialize(blob.substr(0, blob.size() / 2)));
        CHECK(d.empty());
    }
    SECTION("an image whose pixels do not match its size is emptied, a huge polygon is capped")
    {
        LaserDocument bad;
        LaserShape    img = make(ShapeType::Image);
        img.image_w = img.image_h = 1000;   // 1 Mpx claimed, 4 bytes stored
        img.gray.assign(4, 0);
        img.width_mm = img.height_mm = 10;
        bad.add_shape(img);
        LaserShape poly = make(ShapeType::Polygon);
        poly.sides      = 2000000000;
        bad.add_shape(poly);
        LaserDocument d;
        REQUIRE(d.deserialize(bad.serialize()));
        CHECK(d.shapes[0].image_w == 0);
        CHECK(d.shapes[0].gray.empty());
        CHECK_FALSE(d.warnings.empty());
        CHECK(d.flatten(1)[0].pts.size() == 1000);
    }
    SECTION("trailing blocks from a newer build are ignored")
    {
        LaserDocument d;
        CHECK(d.deserialize(blob + std::string("\x04\x00\x00\x00" "abcd", 8)));
        CHECK(d.shapes.size() == doc.shapes.size());
    }
    SECTION("bad parent index is repaired with a warning")
    {
        LaserDocument bad = doc;
        bad.shapes[0].parent = 99;
        LaserDocument d;
        REQUIRE(d.deserialize(bad.serialize()));
        CHECK(d.shapes[0].parent == -1);
        CHECK(d.warnings.size() == 1);
    }
}

TEST_CASE("Laser document grouping and group transforms", "[LaserDoc]")
{
    LaserDocument doc = sample_doc();
    const int g = int(doc.shapes.size()) - 1;
    REQUIRE(doc.shapes[g].type == ShapeType::Group);
    CHECK(doc.children(g) == std::vector<int>{1, 2});
    CHECK(doc.root_of(2) == g);

    // Rect 20x10 at (50,50), ellipse 16x8 at the origin.
    BoundingBoxf bb = doc.bounds({g});
    CHECK(bb.min.x() == Approx(-8));
    CHECK(bb.max.x() == Approx(60));
    CHECK(bb.max.y() == Approx(55));

    Transform2d t = Transform2d::Identity();
    t.translation() = Vec2d(5, -3);
    doc.transform(g, t);
    CHECK(doc.shapes[1].xform.translation().isApprox(Vec2d(55, 47)));
    CHECK(doc.shapes[2].xform.translation().isApprox(Vec2d(5, -3)));
    CHECK(doc.shapes[0].xform.translation().isApprox(Vec2d(0, 0)));   // not a member
    bb = doc.bounds({g});
    CHECK(bb.min.x() == Approx(-3));
    CHECK(bb.max.y() == Approx(52));

    // Scale about the origin applies to members' shapes as well as positions.
    Transform2d s = Transform2d::Identity();
    s.linear() *= 2;
    doc.transform(g, s);
    bb = bbox(doc.flatten(1));
    CHECK(bb.size().x() == Approx(40));

    // Nested group, duplicate deep copies, ungroup restores the members.
    const int outer = doc.group({g, 0});
    REQUIRE(outer >= 0);
    CHECK(doc.root_of(1) == outer);
    const size_t before = doc.shapes.size();
    const std::vector<int> dup = doc.duplicate({outer});
    REQUIRE(dup.size() == 1);
    CHECK(doc.shapes.size() == before + 5);   // outer, path, inner group, rect, ellipse
    CHECK(doc.shapes[dup[0]].type == ShapeType::Group);
    CHECK(doc.shapes[dup[0]].id != doc.shapes[outer].id);
    CHECK(bbox(doc.flatten(dup[0])).size().isApprox(bbox(doc.flatten(outer)).size()));

    const std::vector<int> members = doc.ungroup(outer);
    CHECK(members.size() == 2);
    for (int m : members) CHECK(doc.shapes[m].parent == -1);

    // Removing a group removes its members; removing the last member removes the group.
    const size_t n = doc.shapes.size();
    const int inner = doc.find(doc.shapes[1].parent >= 0 ? doc.shapes[doc.shapes[1].parent].id : 0);
    REQUIRE(inner >= 0);
    doc.remove_shapes({inner});
    CHECK(doc.shapes.size() == n - 3);
    for (const LaserShape& sh : doc.shapes) {
        const bool valid_parent = sh.parent == -1 || (sh.parent < int(doc.shapes.size()) && doc.shapes[sh.parent].type == ShapeType::Group);
        CHECK(valid_parent);
    }

    // Fewer than two shapes or mixed parents -> no group.
    CHECK(doc.group({0}) == -1);
}

TEST_CASE("Laser document flattens every shape type", "[LaserDoc]")
{
    LaserDocument doc = sample_doc();
    const double  tol = 0.01;
    // Rounded rect: 20x10 box, area slightly less than 200 by (4 - pi) r^2.
    LaserPaths r = doc.flatten(1, tol);
    REQUIRE(r.size() == 1);
    CHECK(r[0].closed);
    CHECK(bbox(r).size().isApprox(Vec2d(20, 10), 1e-6));
    const double area = Polygon(r[0].pts.points).area() * SCALING_FACTOR * SCALING_FACTOR;
    CHECK(area == Approx(200 - (4 - M_PI) * 4).margin(0.1));
    // Ellipse: area pi*a*b, chord error bounded.
    LaserPaths e = doc.flatten(2, tol);
    CHECK(Polygon(e[0].pts.points).area() * SCALING_FACTOR * SCALING_FACTOR == Approx(M_PI * 32).epsilon(0.005));
    // Pentagon: 5 vertices, first at the top.
    LaserPaths p = doc.flatten(3);
    REQUIRE(p[0].pts.size() == 5);
    CHECK(unscale(p[0].pts.points[0]).isApprox(Vec2d(0, 5), 1e-6));
    // Text: its cached outlines. Image: its border.
    CHECK(doc.flatten(4).size() == 1);
    CHECK(bbox(doc.flatten(5)).size().isApprox(Vec2d(30, 20), 1e-6));
    // Path: open stays open.
    CHECK_FALSE(doc.flatten(0)[0].closed);
    // Group: both members.
    CHECK(doc.flatten(int(doc.shapes.size()) - 1).size() == 2);
}

TEST_CASE("Material library and device profiles round-trip through JSON", "[LaserDoc]")
{
    const std::vector<MaterialEntry> diode = default_materials(LaserSource::Diode), co2 = default_materials(LaserSource::CO2);
    auto has = [](const std::vector<MaterialEntry>& v, const std::string& m) {
        return std::any_of(v.begin(), v.end(), [&](const MaterialEntry& e) { return e.material == m; });
    };
    for (const char* m : {"Plywood", "MDF", "Cardboard", "Leather", "Anodized aluminum", "Slate"}) {
        CHECK(has(diode, m));
        CHECK(has(co2, m));
    }
    CHECK_FALSE(has(diode, "Acrylic"));   // a diode does not cut clear acrylic
    CHECK(has(co2, "Acrylic"));

    ScopedTemporaryFile f(".json");
    std::string err;
    std::vector<MaterialEntry> mats = co2;
    mats[0].settings.crosshatch = true;
    mats[0].settings.image_dither = DitherMode::Halftone;
    REQUIRE(save_materials(f.string(), mats, &err));
    std::vector<MaterialEntry> back;
    REQUIRE(load_materials(f.string(), back, &err));
    REQUIRE(back.size() == mats.size());
    CHECK(back[0].material == mats[0].material);
    CHECK(back[0].settings.crosshatch);
    CHECK(back[0].settings.image_dither == DitherMode::Halftone);
    CHECK(back[1].settings.speed_mm_s == mats[1].settings.speed_mm_s);

    const std::vector<LaserDevice> devs = default_devices();
    REQUIRE(devs.size() >= 6);
    CHECK(devs[0].name == "Generic GRBL diode");
    ScopedTemporaryFile d(".json");
    std::vector<LaserDevice> dv = devs;
    dv[0].rotary.enabled = true;
    dv[0].rotary.object_diameter = 77;
    REQUIRE(save_devices(d.string(), dv, &err));
    std::vector<LaserDevice> dback;
    REQUIRE(load_devices(d.string(), dback, &err));
    REQUIRE(dback.size() == dv.size());
    CHECK(dback[0].rotary.enabled);
    CHECK(dback[0].rotary.object_diameter == 77);
    CHECK(dback[4].type == DeviceType::Marlin);

    // Built-in profiles are valid; a hand-edited one is repaired field by field, with a message each.
    for (LaserDevice dd : devs) CHECK(validate_device(dd).empty());
    std::vector<LaserDevice> broken(1);
    broken[0].name              = "Edited";
    broken[0].bed_w             = 0;
    broken[0].bed_h             = 5000;
    broken[0].s_max             = 0;
    broken[0].frame_power_pct   = 50;
    broken[0].travel_speed_mm_s = -1;
    REQUIRE(save_devices(d.string(), broken, &err));
    std::vector<std::string> fixes;
    REQUIRE(load_devices(d.string(), dback, &err, &fixes));
    CHECK(dback[0].bed_w == 10);
    CHECK(dback[0].bed_h == 3000);
    CHECK(dback[0].s_max == 1000);
    CHECK(dback[0].frame_power_pct == 20);
    CHECK(dback[0].travel_speed_mm_s == LaserDevice{}.travel_speed_mm_s);
    CHECK(dback[0].max_speed_mm_s == LaserDevice{}.max_speed_mm_s);   // untouched: valid
    REQUIRE(fixes.size() == 5);
    for (const char* field : {"bed_w", "bed_h", "s_max", "frame_power_pct", "travel_speed_mm_s"})
        CHECK(std::any_of(fixes.begin(), fixes.end(), [&](const std::string& m) { return m.find(field) != std::string::npos; }));
    dback = dv;

    // Missing file: false with a message, output untouched.
    CHECK_FALSE(load_devices("/nonexistent/laser_devices.json", dback, &err));
    CHECK_FALSE(err.empty());
    CHECK(dback.size() == dv.size());
}
