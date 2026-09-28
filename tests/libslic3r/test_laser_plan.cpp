#include <catch2/catch_all.hpp>

#include "libslic3r/Laser/LaserPlan.hpp"

#include <cmath>

using namespace Slic3r;
using namespace Slic3r::Laser;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

// A size x size rect centred at (cx, cy) on layer `layer`.
LaserShape rect(double cx, double cy, double size, int layer = 0)
{
    LaserShape s;
    s.type   = ShapeType::Rect;
    s.width  = s.height = size;
    s.layer  = layer;
    s.xform.translation() = Vec2d(cx, cy);
    return s;
}

LaserDevice fast_device()
{
    LaserDevice d;
    d.accel_mm_s2       = 1e9;   // no acceleration: time = length / speed
    d.max_speed_mm_s    = 1000;
    d.travel_speed_mm_s = 200;
    return d;
}

size_t count(const LaserJob& job, Segment::Kind k)
{
    size_t n = 0;
    for (const Segment& s : job.segments) n += s.kind == k;
    return n;
}

double lit_length(const LaserJob& job)
{
    double l = 0;
    for (const Segment& s : job.segments)
        if (s.kind == Segment::Kind::Cut) l += (s.to - s.from).norm();
    return l;
}

} // namespace

TEST_CASE("A 20 mm square on a Line layer plans four cuts and one travel", "[LaserPlan]")
{
    LaserDocument doc;
    doc.add_shape(rect(50, 50, 20));
    doc.layers[0].speed_mm_s = 40;
    const LaserDevice dev = fast_device();
    const LaserJob    job = plan(doc, dev);
    REQUIRE(job.ok());
    CHECK(count(job, Segment::Kind::Cut) == 4);
    CHECK(count(job, Segment::Kind::Travel) == 1);
    CHECK(job.segments.front().from.isApprox(Vec2d(0, 0)));
    CHECK_THAT(lit_length(job), WithinAbs(80, 1e-6));
    const double travel = Vec2d(40, 40).norm();
    CHECK_THAT(job.estimated_time_s, WithinRel(80. / 40 + travel / 200, 1e-3));
    CHECK_THAT(job.layer_time_s[0], WithinRel(job.estimated_time_s, 1e-9));
    CHECK_THAT(job.bounds.min.x(), WithinAbs(40, 1e-6));
    CHECK_THAT(job.bounds.max.y(), WithinAbs(60, 1e-6));

    SECTION("acceleration adds time")
    {
        LaserDevice slow = dev;
        slow.accel_mm_s2 = 1000;
        const LaserJob j2 = plan(doc, slow);
        // Each 20 mm edge: 20/40 + 40/1000 s.
        CHECK(j2.estimated_time_s > job.estimated_time_s + 4 * 0.039);
    }
    SECTION("passes duplicate the cuts, Z steps down per pass")
    {
        doc.layers[0].passes          = 3;
        doc.layers[0].z_step_per_pass = 0.5;
        const LaserJob j3 = plan(doc, dev);
        CHECK(count(j3, Segment::Kind::Cut) == 12);
        CHECK_THAT(lit_length(j3), WithinAbs(240, 1e-6));
        CHECK_THAT(j3.segments.back().z, WithinAbs(-1.0, 1e-9));
    }
    SECTION("output off, hidden shape or empty selection: nothing to burn")
    {
        LaserDocument d2 = doc;
        d2.layers[0].output = false;
        CHECK_FALSE(plan(d2, dev).ok());
        d2 = doc;
        d2.shapes[0].visible = false;
        CHECK_FALSE(plan(d2, dev).ok());
        d2 = doc;
        d2.job.cut_selected_only = true;
        CHECK_FALSE(plan(d2, dev, {}).ok());
        CHECK(plan(d2, dev, {0}).ok());
    }
    SECTION("off the bed with absolute coordinates is an error")
    {
        LaserDocument d2 = doc;
        d2.shapes[0].xform.translation() = Vec2d(395, 50);
        CHECK_FALSE(plan(d2, dev).ok());
    }
    SECTION("Ruida is refused")
    {
        LaserDevice r = dev;
        r.type = DeviceType::Ruida;
        CHECK_FALSE(plan(doc, r).ok());
    }
    SECTION("cancel")
    {
        const LaserJob c = plan(doc, dev, {}, [](double) { return true; });
        CHECK(c.error == "Cancelled");
        CHECK(c.segments.empty());
    }
}

TEST_CASE("A Fill square scans one line per interval", "[LaserPlan]")
{
    LaserDocument doc;
    doc.add_shape(rect(50, 50, 20));
    LaserLayer& L  = doc.layers[0];
    L.mode         = LayerMode::Fill;
    L.interval_mm  = 0.1;
    L.overscan_mm  = 1;
    const LaserJob job = plan(doc, fast_device());
    REQUIRE(job.ok());
    // 20 mm / 0.1 mm = 200 rows.
    CHECK(job.scans.size() == 200);
    CHECK(count(job, Segment::Kind::Scan) == 200);
    CHECK(count(job, Segment::Kind::Cut) == 0);
    for (const ScanLine& sl : job.scans) {
        REQUIRE(sl.runs.size() == 1);
        CHECK_THAT((sl.end - sl.start).norm(), WithinAbs(22, 1e-3));   // 20 + 2 x 1 mm overscan
        CHECK_THAT(sl.runs[0].x0, WithinAbs(1, 1e-4));
        CHECK_THAT(sl.runs[0].x1, WithinAbs(21, 1e-3));
    }
    // Bidirectional: consecutive rows run in opposite directions.
    CHECK((job.scans[0].end - job.scans[0].start).x() * (job.scans[1].end - job.scans[1].start).x() < 0);
    // Bounds include the overscan.
    CHECK_THAT(job.bounds.min.x(), WithinAbs(39, 1e-3));

    SECTION("FillLine: fill first, then the outline")
    {
        doc.layers[0].mode = LayerMode::FillLine;
        const LaserJob j2 = plan(doc, fast_device());
        CHECK(count(j2, Segment::Kind::Cut) == 4);
        CHECK(j2.segments.back().kind == Segment::Kind::Cut);
        CHECK(j2.segments[1].kind == Segment::Kind::Scan);
    }
}

TEST_CASE("Nested squares cut the inner one first", "[LaserPlan]")
{
    LaserDocument doc;
    doc.add_shape(rect(50, 50, 40));   // outer first in the document
    doc.add_shape(rect(50, 50, 10));
    const LaserJob job = plan(doc, fast_device());
    REQUIRE(job.ok());
    REQUIRE(count(job, Segment::Kind::Cut) == 8);
    const Segment* first_cut = nullptr;
    for (const Segment& s : job.segments)
        if (s.kind == Segment::Kind::Cut) { first_cut = &s; break; }
    REQUIRE(first_cut);
    CHECK(std::abs(first_cut->from.x() - 50) <= 5 + 1e-6);   // on the 10 mm square
    CHECK(std::abs(first_cut->from.y() - 50) <= 5 + 1e-6);
}

TEST_CASE("Layers run in priority order", "[LaserPlan]")
{
    LaserDocument doc;
    doc.add_shape(rect(20, 20, 10, 0));
    doc.add_shape(rect(60, 20, 10, 1));
    doc.layers[0].priority = 5;
    doc.layers[1].priority = 1;
    doc.job.optimize.order_by = JobSettings::OrderBy::Priority;
    LaserJob job = plan(doc, fast_device());
    REQUIRE(job.ok());
    CHECK(job.segments.front().layer == 1);
    doc.job.optimize.order_by = JobSettings::OrderBy::Layer;
    job = plan(doc, fast_device());
    CHECK(job.segments.front().layer == 0);
}

TEST_CASE("Job origin moves the anchor to zero, rotary scales Y", "[LaserPlan]")
{
    LaserDocument doc;
    doc.add_shape(rect(100, 80, 20));   // bounds 90..110 x 70..90
    doc.job.start_from = JobSettings::StartFrom::UserOrigin;
    doc.job.job_origin = 6;   // front-left
    LaserJob job = plan(doc, fast_device());
    REQUIRE(job.ok());
    CHECK_THAT(job.bounds.min.x(), WithinAbs(0, 1e-6));
    CHECK_THAT(job.bounds.min.y(), WithinAbs(0, 1e-6));
    CHECK_THAT(job.bounds.max.x(), WithinAbs(20, 1e-6));
    CHECK(job.start_from == JobSettings::StartFrom::UserOrigin);

    doc.job.job_origin = 4;   // centre
    job = plan(doc, fast_device());
    CHECK_THAT(job.bounds.min.x(), WithinAbs(-10, 1e-6));
    CHECK_THAT(job.bounds.max.y(), WithinAbs(10, 1e-6));
    doc.job.job_origin = 2;   // rear-right
    job = plan(doc, fast_device());
    CHECK_THAT(job.bounds.max.x(), WithinAbs(0, 1e-6));
    CHECK_THAT(job.bounds.max.y(), WithinAbs(0, 1e-6));

    // Rotary, roller: Y * mm_per_rotation / (pi * roller_diameter).
    LaserDevice dev = fast_device();
    dev.rotary.enabled         = true;
    dev.rotary.type            = RotaryType::Roller;
    dev.rotary.mm_per_rotation = 40;
    dev.rotary.roller_diameter = 20;
    doc.job.job_origin         = 6;
    job = plan(doc, dev);
    REQUIRE(job.ok());
    const double k = 40 / (M_PI * 20);
    CHECK_THAT(job.bounds.max.y(), WithinAbs(20 * k, 1e-6));
    CHECK_THAT(job.bounds.max.x(), WithinAbs(20, 1e-6));
    dev.rotary.type            = RotaryType::Chuck;
    dev.rotary.object_diameter = 50;
    job = plan(doc, dev);
    CHECK_THAT(job.bounds.max.y(), WithinAbs(20 * 40 / (M_PI * 50), 1e-6));
}

TEST_CASE("Tabs, dot mode and images reach the plan", "[LaserPlan]")
{
    LaserDocument doc;
    doc.add_shape(rect(50, 50, 20));
    doc.layers[0].tabs        = true;
    doc.layers[0].tab_count   = 4;
    doc.layers[0].tab_size_mm = 1;
    LaserJob job = plan(doc, fast_device());
    REQUIRE(job.ok());
    CHECK_THAT(lit_length(job), WithinAbs(76, 1e-3));

    doc.layers[0].tabs           = false;
    doc.layers[0].dot_mode       = true;
    doc.layers[0].dot_spacing_mm = 1;
    job = plan(doc, fast_device());
    CHECK(count(job, Segment::Kind::Dwell) == 80);   // every mm of the 80 mm outline, the start once

    LaserShape img;
    img.type     = ShapeType::Image;
    img.image_w  = 4;
    img.image_h  = 4;
    img.gray.assign(16, 0);   // black: burns everywhere
    img.width_mm = img.height_mm = 4;
    img.layer    = 2;
    img.xform.translation() = Vec2d(10, 10);
    LaserDocument d2;
    d2.add_shape(img);
    d2.layers[2].image_dpi = 25.4;   // 1 mm pixels
    job = plan(d2, fast_device());
    REQUIRE(job.ok());
    CHECK(job.scans.size() == 4);
    CHECK(count(job, Segment::Kind::Cut) == 0);
}
