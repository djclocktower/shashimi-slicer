#include <catch2/catch_all.hpp>

#include "libslic3r/Laser/VectorOps.hpp"
#include "libslic3r/Laser/TextLayout.hpp"
#include "libslic3r/Utils.hpp"
#include "libslic3r/ClipperUtils.hpp"

#include <boost/filesystem/path.hpp>

using namespace Slic3r;
using namespace Slic3r::Laser;
using Catch::Approx;

namespace {

LaserPath square(double x0, double y0, double size, bool ccw = true)
{
    Points pts{Point::new_scale(x0, y0), Point::new_scale(x0 + size, y0), Point::new_scale(x0 + size, y0 + size),
               Point::new_scale(x0, y0 + size)};
    if (!ccw) std::reverse(pts.begin(), pts.end());
    return {Polyline(pts), true};
}

double total_length_mm(const Polylines& pls)
{
    double l = 0;
    for (const Polyline& p : pls) l += unscale<double>(p.length());
    return l;
}

double area_mm2(const ExPolygons& ex)
{
    double a = 0;
    for (const ExPolygon& e : ex) a += e.area();
    return a * SCALING_FACTOR * SCALING_FACTOR;
}

std::string repo_font() { return (boost::filesystem::path(PROFILES_DIR).parent_path() / "fonts" / "NotoSansKR-Regular.ttf").string(); }

} // namespace

TEST_CASE("Hatch fill covers the area", "[LaserVec]")
{
    // 20 mm square with a 6 mm hole plus a separate circle-ish octagon.
    LaserPaths paths{square(0, 0, 20), square(7, 7, 6, false), square(30, 0, 10)};
    const ExPolygons regions = to_expolygons(paths);
    REQUIRE(regions.size() == 2);
    const double area = area_mm2(regions);
    CHECK(area == Approx(400 - 36 + 100));

    for (double angle : {0., 30., 90., 135.}) {
        INFO("angle " << angle);
        const double interval = 0.1;
        const Polylines lines = hatch_fill(regions, interval, angle, false, true);
        CHECK(total_length_mm(lines) * interval == Approx(area).epsilon(0.05));
        // Every line runs along the scan angle.
        const Vec2d d(std::cos(angle * M_PI / 180), std::sin(angle * M_PI / 180));
        for (const Polyline& l : lines) {
            const Vec2d v = unscale(l.last_point()) - unscale(l.first_point());
            REQUIRE(std::abs(std::abs(v.normalized().dot(d)) - 1) < 1e-6);
        }
        // Crosshatch doubles the burnt length.
        CHECK(total_length_mm(hatch_fill(regions, interval, angle, true, true)) == Approx(2 * total_length_mm(lines)).epsilon(0.02));
    }

    SECTION("rows run top to bottom, bidirectional alternates, unidirectional does not")
    {
        const ExPolygons sq = to_expolygons({square(0, 0, 1)});
        const Polylines bi = hatch_fill(sq, 0.25, 0, false, true);
        REQUIRE(bi.size() == 4);
        CHECK(unscale(bi[0].first_point()).y() == Approx(0.875));
        CHECK(unscale(bi[3].first_point()).y() == Approx(0.125));
        CHECK(bi[0].first_point().x() < bi[0].last_point().x());
        CHECK(bi[1].first_point().x() > bi[1].last_point().x());
        const Polylines uni = hatch_fill(sq, 0.25, 0, false, false);
        for (const Polyline& l : uni) CHECK(l.first_point().x() < l.last_point().x());
    }
}

TEST_CASE("Offsets, booleans, kerf and offset fill", "[LaserVec]")
{
    const LaserPaths sq{square(0, 0, 10)};
    auto bbox = [](const LaserPaths& ps) {
        BoundingBoxf bb;
        for (const LaserPath& p : ps)
            for (const Point& pt : p.pts.points) bb.merge(unscale(pt));
        return bb;
    };
    LaserPaths out = offset_paths(sq, 1, OffsetDir::Outward, CornerStyle::Miter);
    REQUIRE(out.size() == 1);
    CHECK(bbox(out).size().x() == Approx(12));
    out = offset_paths(sq, 1, OffsetDir::Inward, CornerStyle::Round);
    CHECK(bbox(out).size().x() == Approx(8));
    out = offset_paths(sq, 1, OffsetDir::Both, CornerStyle::Round);
    CHECK(out.size() == 2);
    // Round corners: area of the Minkowski sum 100 + 4*10 + pi.
    out = offset_paths(sq, 1, OffsetDir::Outward, CornerStyle::Round);
    CHECK(area_mm2(to_expolygons(out)) == Approx(140 + M_PI).epsilon(0.002));
    // Open path -> closed outline around it.
    const LaserPaths line{{Polyline(Points{Point::new_scale(0, 0), Point::new_scale(10, 0)}), false}};
    out = offset_paths(line, 0.5, OffsetDir::Outward, CornerStyle::Miter);
    REQUIRE(out.size() == 1);
    CHECK(out[0].closed);
    CHECK(bbox(out).size().x() == Approx(11));
    CHECK(bbox(out).size().y() == Approx(1));

    // Booleans.
    const LaserPaths other{square(5, 0, 10)};
    CHECK(area_mm2(to_expolygons(boolean_op(sq, other, BooleanOp::Union))) == Approx(150));
    CHECK(area_mm2(to_expolygons(boolean_op(sq, other, BooleanOp::Subtract))) == Approx(50));
    CHECK(area_mm2(to_expolygons(boolean_op(sq, other, BooleanOp::Intersect))) == Approx(50));
    CHECK(area_mm2(to_expolygons(weld({sq, other}))) == Approx(150));

    // Kerf: outer grows, hole shrinks.
    const LaserPaths framed{square(0, 0, 20), square(5, 5, 10, false)};
    out = kerf_offset(framed, 0.1);
    CHECK(area_mm2(to_expolygons(out)) == Approx(20.2 * 20.2 - 9.8 * 9.8).epsilon(1e-4));
    CHECK(kerf_offset(line, 0.1).size() == 1);

    // Offset fill: 10 mm square at 1 mm -> rings at 0.5, 1.5, ... 4.5 inside: 5 rings, innermost first.
    const LaserPaths rings = offset_fill(to_expolygons(sq), 1);
    REQUIRE(rings.size() == 5);
    CHECK(bbox({rings.front()}).size().x() == Approx(1).margin(0.01));
    CHECK(bbox({rings.back()}).size().x() == Approx(9).margin(0.01));
}

TEST_CASE("Tabs leave gaps of the requested size", "[LaserVec]")
{
    const LaserPaths sq{square(0, 0, 10)};   // 40 mm perimeter
    LaserPaths out = insert_tabs(sq, 4, 0, 0.5);
    REQUIRE(out.size() == 4);
    double cut = 0;
    for (const LaserPath& p : out) {
        CHECK_FALSE(p.closed);
        cut += unscale<double>(p.pts.length());
    }
    CHECK(cut == Approx(40 - 4 * 0.5).margin(1e-4));
    // Gap between the end of one piece and the start of the next is the tab size.
    for (size_t i = 0; i < out.size(); ++i) {
        const Vec2d end = unscale(out[i].pts.last_point()), next = unscale(out[(i + 1) % out.size()].pts.first_point());
        CHECK((next - end).norm() == Approx(0.5).margin(1e-4));
    }
    // By spacing: one every 15 mm -> floor(40 / 15) = 2 tabs.
    CHECK(insert_tabs(sq, 0, 15, 1).size() == 2);
    // Tabs that would eat the whole path are skipped.
    CHECK(insert_tabs(sq, 100, 0, 1).front().closed);
}

TEST_CASE("Lead-in approaches from outside", "[LaserVec]")
{
    const LaserPaths framed{square(0, 0, 20), square(5, 5, 10, true)};
    const LaserPaths out = add_lead_in(framed, 2);
    REQUIRE(out.size() == 2);
    const Polygon outer(framed[0].pts.points), hole(framed[1].pts.points);
    // Outer contour: lead-in starts outside the part; hole: starts inside the hole.
    CHECK_FALSE(outer.contains(out[0].pts.first_point()));
    CHECK(hole.contains(out[1].pts.first_point()));
    CHECK((unscale(out[0].pts.first_point()) - unscale(out[0].pts.points[1])).norm() == Approx(2));
    CHECK(unscale(out[0].pts.points[1]).isApprox(Vec2d(10, 0)));   // mid-way along the first edge
    CHECK(out[0].pts.last_point() == out[0].pts.points[1]);        // closes back at the start
    CHECK(unscale<double>(out[0].pts.length()) == Approx(2 + 80));
}

TEST_CASE("Cut order: inner paths first, nearest neighbour", "[LaserVec]")
{
    // Two nested pairs, listed outer first.
    LaserPaths paths{square(0, 0, 20), square(100, 0, 20), square(5, 5, 10), square(105, 5, 10)};
    JobSettings::Optimize opts;
    opts.cut_inner_first = true;
    opts.reduce_travel   = true;
    const std::vector<Points> orig{paths[0].pts.points, paths[1].pts.points, paths[2].pts.points, paths[3].pts.points};
    optimize_order(paths, Vec2d(0, 0), opts);
    auto index_of = [&](const LaserPath& p) {
        for (size_t i = 0; i < orig.size(); ++i) {
            Points a = orig[i], b = p.pts.points;
            std::sort(a.begin(), a.end());
            std::sort(b.begin(), b.end());
            if (a == b) return int(i);
        }
        return -1;
    };
    std::vector<int> order;
    for (const LaserPath& p : paths) order.push_back(index_of(p));
    CHECK(order == std::vector<int>{2, 0, 3, 1});
    // The first path starts at its vertex nearest the start point.
    CHECK(unscale(paths[0].pts.first_point()).isApprox(Vec2d(5, 5)));

    // Without inner-first the nearest path wins: the outer square at the origin.
    LaserPaths p2{square(5, 5, 10), square(0, 0, 20)};
    opts.cut_inner_first = false;
    optimize_order(p2, Vec2d(0, 0), opts);
    CHECK(unscale(p2[0].pts.first_point()).isApprox(Vec2d(0, 0)));

    // Open paths are reversed when their end is nearer.
    LaserPaths p3{{Polyline(Points{Point::new_scale(50, 0), Point::new_scale(1, 0)}), false}};
    const Vec2d end = optimize_order(p3, Vec2d(0, 0), opts);
    CHECK(end.isApprox(Vec2d(50, 0)));

    // reduce_direction_changes: every closed path gets the first one's winding.
    LaserPaths p4{square(0, 0, 5, true), square(10, 0, 5, false)};
    opts.reduce_direction_changes = true;
    optimize_order(p4, Vec2d(0, 0), opts);
    CHECK(Polygon(p4[0].pts.points).is_counter_clockwise() == Polygon(p4[1].pts.points).is_counter_clockwise());
}

TEST_CASE("Text layout through Emboss", "[LaserVec]")
{
    const std::string font = repo_font();
    std::string err;
    const Polygons ab = text_outlines("Ab", font, 10, 0, false, false, TextAlign::Left, &err);
    INFO(err);
    REQUIRE(ab.size() >= 2);   // A (+ its counter), b (+ its counter)
    BoundingBox bb = get_extents(ab);
    const Vec2d size = unscale(bb.size());
    CHECK(size.x() > 5);
    CHECK(size.x() < 20);
    CHECK(size.y() > 5);
    CHECK(size.y() < 12);
    CHECK(unscale<double>(bb.min.y()) > -1);   // sits on the baseline (no descenders in "Ab")
    CHECK(unscale<double>(bb.min.x()) >= -0.5);

    // Linear in the height.
    const BoundingBox bb2 = get_extents(text_outlines("Ab", font, 20));
    CHECK(unscale<double>(bb2.size().x()) == Approx(2 * size.x()).epsilon(0.02));
    CHECK(unscale<double>(bb2.size().y()) == Approx(2 * size.y()).epsilon(0.02));

    // Centred and right aligned.
    CHECK(std::abs(unscale<double>(get_extents(text_outlines("Ab", font, 10, 0, false, false, TextAlign::Center)).center().x())) < 0.6);
    CHECK(unscale<double>(get_extents(text_outlines("Ab", font, 10, 0, false, false, TextAlign::Right)).max.x()) < 0.6);
    // Second line below the first.
    CHECK(get_extents(text_outlines("A\nA", font, 10)).min.y() < scale_(-5));
    // Spacing widens, bold grows, italic leans.
    CHECK(get_extents(text_outlines("Ab", font, 10, 2)).size().x() > bb.size().x() + scale_(1.5));
    CHECK(area_mm2(union_ex(text_outlines("A", font, 10, 0, true))) > area_mm2(union_ex(text_outlines("A", font, 10))));
    CHECK(get_extents(text_outlines("I", font, 10, 0, false, true)).size().x() > get_extents(text_outlines("I", font, 10)).size().x() + scale_(1));

    // A family name that does not exist falls back to the bundled font.
    const std::string saved = resources_dir();
    set_resources_dir((boost::filesystem::path(PROFILES_DIR).parent_path()).string());
    CHECK(resolve_font_path("No Such Font Family 123") == fallback_font_path());
    CHECK_FALSE(fallback_font_path().empty());
    set_resources_dir(saved);

    // update_text_outlines keeps the cache when the font fails.
    LaserShape t;
    t.type = ShapeType::Text;
    t.text = "Hi";
    t.font = font;
    REQUIRE(update_text_outlines(t));
    const size_t n = t.text_outlines.size();
    t.text_height_mm = -1;
    CHECK_FALSE(update_text_outlines(t, &err));
    CHECK(t.text_outlines.size() == n);
}
