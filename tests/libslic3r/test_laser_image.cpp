#include <catch2/catch_all.hpp>

#include "libslic3r/Laser/ImageEngrave.hpp"
#include "libslic3r/Laser/Trace.hpp"

#include <cmath>

using namespace Slic3r;
using namespace Slic3r::Laser;
using Catch::Approx;

namespace {

LaserShape image_shape(int w, int h, double w_mm, double h_mm, const std::function<uint8_t(int, int)>& px)
{
    LaserShape s;
    s.type    = ShapeType::Image;
    s.image_w = w;
    s.image_h = h;
    s.width_mm  = w_mm;
    s.height_mm = h_mm;
    s.gray.resize(size_t(w) * h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) s.gray[size_t(y) * w + x] = px(x, y);
    return s;
}

LaserLayer plain_layer()
{
    LaserLayer l;
    l.image_dpi     = 254;   // 0.1 mm pitch
    l.overscan_pct  = 0;
    l.overscan_mm   = 0;
    l.bidirectional = false;
    l.power_min     = 10;
    l.power_max     = 60;
    return l;
}

Raster raster_rows(const std::vector<std::vector<uint8_t>>& rows)
{
    Raster r;
    r.h = int(rows.size());
    r.w = int(rows.front().size());
    for (const auto& row : rows) r.burn.insert(r.burn.end(), row.begin(), row.end());
    r.pixel_mm = r.line_mm = 0.1;
    r.to_workspace.linear() << 0.1, 0, 0, -0.1;   // columns right, rows down
    return r;
}

double path_area_mm2(const LaserPath& p) { return Polygon(p.pts.points).area() * SCALING_FACTOR * SCALING_FACTOR; }

} // namespace

TEST_CASE("Dithers keep the mean darkness of a gradient", "[LaserImage]")
{
    const int w = 256, h = 512, band = 64;   // gray = row / 2: black at the top, white at the bottom
    std::vector<uint8_t> gray(size_t(w) * h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) gray[size_t(y) * w + x] = uint8_t(y / 2);

    for (DitherMode mode : {DitherMode::Threshold, DitherMode::Ordered, DitherMode::FloydSteinberg, DitherMode::Jarvis,
                            DitherMode::Stucki, DitherMode::Atkinson, DitherMode::Newsprint, DitherMode::Halftone,
                            DitherMode::Grayscale}) {
        const std::vector<uint8_t> burn = dither(gray, w, h, mode, 254, 25, 22.5);
        REQUIRE(burn.size() == gray.size());
        for (int b = 0; b < h / band; ++b) {
            double got = 0, want = 0;
            for (int y = b * band; y < (b + 1) * band; ++y)
                for (int x = 0; x < w; ++x) {
                    got  += burn[size_t(y) * w + x] / 255.;
                    want += (255 - gray[size_t(y) * w + x]) / 255.;
                }
            got /= band * w;
            want /= band * w;
            INFO("mode " << int(mode) << " band " << b << " want " << want << " got " << got);
            if (mode == DitherMode::Threshold)
                CHECK_THAT(got, Catch::Matchers::WithinAbs(want > 0.5 ? 1. : 0., 1e-12));   // bands never straddle 128
            else if (mode == DitherMode::Atkinson && (b < 2 || b >= h / band - 2))
                CHECK(std::abs(got - want) < 0.15);     // Atkinson drops 1/4 of the error: pushes the outer quarters to the extremes
            else
                CHECK(std::abs(got - want) < 0.05);
        }
    }
}

TEST_CASE("Threshold checkerboard gives exact runs", "[LaserImage]")
{
    // 16 x 16 px, 2 px squares, 0.1 mm/px: the raster is the image itself.
    const LaserShape img = image_shape(16, 16, 1.6, 1.6, [](int x, int y) { return ((x / 2 + y / 2) % 2) ? 255 : 0; });
    LaserLayer layer = plain_layer();
    layer.image_dither = DitherMode::Threshold;
    const Raster r = prepare_image(img, layer);
    REQUIRE(r.w == 16);
    REQUIRE(r.h == 16);
    CHECK((r.to_workspace * Vec2d(0, 0) - Vec2d(-0.75, 0.75)).norm() < 1e-9);   // top-left pixel centre

    const std::vector<ScanLine> lines = raster_to_scanlines(r, layer, LaserDevice{});
    REQUIRE(lines.size() == 16);
    for (int row = 0; row < 16; ++row) {
        const ScanLine& sl = lines[row];
        const double first = (row / 2) % 2 ? 2 : 0;
        CHECK(sl.start.x() == Approx(-0.8 + first * 0.1));
        CHECK(sl.end.x() == Approx(-0.8 + (first + 14) * 0.1));
        CHECK(sl.start.y() == Approx(0.75 - row * 0.1));
        REQUIRE(sl.runs.size() == 4);
        for (int k = 0; k < 4; ++k) {
            CHECK(sl.runs[k].x0 == Approx(0.4 * k).margin(1e-5));
            CHECK(sl.runs[k].x1 == Approx(0.4 * k + 0.2).margin(1e-5));
            CHECK(sl.runs[k].power_pct == Approx(60));
        }
    }
}

TEST_CASE("Grayscale maps burn to power_min..power_max", "[LaserImage]")
{
    const LaserLayer layer = plain_layer();
    const auto lines = raster_to_scanlines(raster_rows({{255, 255, 128, 0, 1}}), layer, LaserDevice{});
    REQUIRE(lines.size() == 1);
    const auto& runs = lines[0].runs;
    REQUIRE(runs.size() == 3);
    CHECK(runs[0].x0 == Approx(0));
    CHECK(runs[0].x1 == Approx(0.2));
    CHECK(runs[0].power_pct == Approx(60));
    CHECK(runs[1].power_pct == Approx(35));   // 10 + 128/255 * 50, 1 % steps
    CHECK(runs[2].x0 == Approx(0.4));
    CHECK(runs[2].power_pct == Approx(10));

    // Through prepare_image: black -> max, white -> nothing.
    LaserLayer gl = plain_layer();
    gl.image_dither = DitherMode::Grayscale;
    const Raster r = prepare_image(image_shape(4, 1, 0.4, 0.1, [](int x, int) { return x < 2 ? 0 : 255; }), gl);
    const auto gl_lines = raster_to_scanlines(r, gl, LaserDevice{});
    REQUIRE(gl_lines.size() == 1);
    REQUIRE(gl_lines[0].runs.size() == 1);
    CHECK(gl_lines[0].runs[0].power_pct == Approx(60));
    CHECK(gl_lines[0].runs[0].x1 == Approx(0.2).margin(1e-5));
}

TEST_CASE("Overscan extends each line", "[LaserImage]")
{
    LaserLayer layer = plain_layer();
    layer.overscan_mm = 1;
    const Raster r = raster_rows({{0, 0, 255, 255, 255, 0}});
    auto lines = raster_to_scanlines(r, layer, LaserDevice{});
    REQUIRE(lines.size() == 1);
    CHECK((lines[0].end - lines[0].start).norm() == Approx(2.3));
    CHECK(lines[0].start.x() == Approx(0.15 - 1));
    REQUIRE(lines[0].runs.size() == 1);
    CHECK(lines[0].runs[0].x0 == Approx(1));
    CHECK(lines[0].runs[0].x1 == Approx(1.3));

    layer.overscan_mm  = 0;
    layer.overscan_pct = 10;
    layer.speed_mm_s   = 50;   // 5 mm
    lines = raster_to_scanlines(r, layer, LaserDevice{});
    CHECK((lines[0].end - lines[0].start).norm() == Approx(10.3));
}

TEST_CASE("Bidirectional alternates, blank rows are skipped", "[LaserImage]")
{
    const Raster r = raster_rows({{0, 255, 0, 0}, {0, 0, 0, 0}, {255, 255, 0, 0}, {0, 0, 0, 0}, {0, 255, 255, 255}});
    LaserLayer layer = plain_layer();
    auto lines = raster_to_scanlines(r, layer, LaserDevice{});
    REQUIRE(lines.size() == 3);
    for (const ScanLine& sl : lines) CHECK(sl.end.x() > sl.start.x());
    CHECK(lines[0].start.y() > lines[1].start.y());   // top -> bottom
    CHECK(lines[1].start.y() > lines[2].start.y());

    layer.bidirectional = true;
    lines = raster_to_scanlines(r, layer, LaserDevice{});
    REQUIRE(lines.size() == 3);
    CHECK(lines[0].end.x() > lines[0].start.x());
    CHECK(lines[1].end.x() < lines[1].start.x());
    CHECK(lines[2].end.x() > lines[2].start.x());
    CHECK(lines[1].start.x() == Approx(0.15));   // row 2 lit cols 0..1, reversed: starts at the right edge
    REQUIRE(lines[1].runs.size() == 1);
    CHECK(lines[1].runs[0].x0 == Approx(0));
    CHECK(lines[1].runs[0].x1 == Approx(0.2));
}

TEST_CASE("Rotated raster maps back onto the image", "[LaserImage]")
{
    LaserShape img = image_shape(100, 200, 10, 20, [](int, int) { return 0; });
    img.xform = Transform2d::Identity();
    img.xform.translate(Vec2d(50, 50)).rotate(Eigen::Rotation2Dd(30 * M_PI / 180));
    LaserLayer layer = plain_layer();
    layer.image_dither = DitherMode::Threshold;

    // Scan angle = image rotation: the raster is the image, corners map exactly.
    layer.angle_deg = 30;
    Raster r = prepare_image(img, layer);
    REQUIRE(r.w == 100);
    REQUIRE(r.h == 200);
    CHECK((r.to_workspace * Vec2d(-0.5, -0.5) - img.xform * Vec2d(-5, 10)).norm() < 1e-6);   // top-left
    CHECK((r.to_workspace * Vec2d(r.w - 0.5, r.h - 0.5) - img.xform * Vec2d(5, -10)).norm() < 1e-6);
    size_t lit = std::count(r.burn.begin(), r.burn.end(), 255);
    CHECK(double(lit) / (r.w * r.h) > 0.99);

    // Scan angle 0: axis-aligned raster over the rotated image; burnt area = image area.
    layer.angle_deg = 0;
    r = prepare_image(img, layer);
    lit = std::count(r.burn.begin(), r.burn.end(), 255);
    CHECK(lit * 0.01 == Approx(200).epsilon(0.03));
    BoundingBoxf quad;
    for (const Vec2d c : {Vec2d(-5, -10), Vec2d(5, -10), Vec2d(5, 10), Vec2d(-5, 10)}) quad.merge(img.xform * c);
    const Vec2d tl = r.to_workspace * Vec2d(-0.5, -0.5), br = r.to_workspace * Vec2d(r.w - 0.5, r.h - 0.5);
    CHECK(tl.x() == Approx(quad.min.x()));
    CHECK(tl.y() == Approx(quad.max.y()));
    CHECK(br.x() == Approx(quad.max.x()).margin(0.1));
    CHECK(br.y() == Approx(quad.min.y()).margin(0.1));
    // Every lit pixel centre is inside the rotated image.
    const Transform2d inv = img.xform.inverse();
    int outside = 0;
    for (int y = 0; y < r.h; ++y)
        for (int x = 0; x < r.w; ++x)
            if (r.burn[size_t(y) * r.w + x]) {
                const Vec2d p = inv * (r.to_workspace * Vec2d(x, y));
                outside += std::abs(p.x()) > 5.05 || std::abs(p.y()) > 10.05;
            }
    CHECK(outside == 0);
}

TEST_CASE("Trace a filled circle and a ring", "[LaserImage]")
{
    const double r_px = 150, r_in = 80;   // 0.1 mm/px
    auto disc = [&](double inner) {
        return image_shape(400, 400, 40, 40, [=](int x, int y) {
            const double d = std::hypot(x + 0.5 - 200, y + 0.5 - 200);
            return (d < r_px && d >= inner) ? 0 : 255;
        });
    };
    TraceOptions opts;

    LaserShape circle = trace_image(disc(0), opts);
    CHECK(circle.type == ShapeType::Path);
    REQUIRE(circle.paths.size() == 1);
    CHECK(circle.paths[0].closed);
    CHECK(path_area_mm2(circle.paths[0]) == Approx(M_PI * 15 * 15).epsilon(0.02));

    LaserShape ring = trace_image(disc(r_in), opts);
    REQUIRE(ring.paths.size() == 2);
    const double a0 = path_area_mm2(ring.paths[0]), a1 = path_area_mm2(ring.paths[1]);
    CHECK(a0 > 0);   // outer CCW
    CHECK(a1 < 0);   // hole CW
    CHECK(a0 + a1 == Approx(M_PI * (15 * 15 - 8 * 8)).epsilon(0.02));

    // Specks below ignore_less_than_px are dropped.
    opts.ignore_less_than_px = 400;
    CHECK(trace_image(disc(0), opts).paths.empty());
}
