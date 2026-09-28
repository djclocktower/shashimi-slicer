#include <catch2/catch_all.hpp>

#include "libslic3r/Laser/Import.hpp"
#include "libslic3r/Laser/LaserDocument.hpp"

#include "test_utils.hpp"

#include <boost/beast/core/detail/base64.hpp>
#include <boost/nowide/fstream.hpp>

using namespace Slic3r;
using namespace Slic3r::Laser;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

void write(const std::string& path, const std::string& data)
{
    boost::nowide::ofstream f(path, std::ios::binary);
    f << data;
}

std::string unbase64(const std::string& s)
{
    std::string out(boost::beast::detail::base64::decoded_size(s.size()), '\0');
    out.resize(boost::beast::detail::base64::decode(&out[0], s.data(), s.size()).first);
    return out;
}

// 3 x 2 gray: 0 128 255 / 255 255 0.
const char* kGrayPng = "iVBORw0KGgoAAAANSUhEUgAAAAMAAAACCAAAAAC4HznGAAAAEElEQVR4nGNgaPjP8P8/AwAM/gN+gu/1iAAAAABJRU5ErkJggg==";
// 2 x 1 RGBA: opaque black, transparent black.
const char* kRgbaPng = "iVBORw0KGgoAAAANSUhEUgAAAAIAAAABCAYAAAD0In+KAAAAD0lEQVR4nGNgYGD4D8QMAAUEAQCwBUiSAAAAAElFTkSuQmCC";

// Workspace-frame outline bounds of result shape i (through a document, so every type flattens).
BoundingBoxf shape_bounds(const ImportResult& r, int i)
{
    LaserDocument doc;
    doc.add_shapes(r.shapes);
    return doc.bounds({i});
}

} // namespace

TEST_CASE("SVG import: units, colours, fills and strokes, groups", "[LaserImport]")
{
    ScopedTemporaryFile f(".svg");
    write(f.string(), R"(<svg xmlns="http://www.w3.org/2000/svg" width="100mm" height="50mm" viewBox="0 0 100 50">
  <rect x="10" y="10" width="20" height="10" fill="#ff0000"/>
  <path d="M 50 40 L 90 40" stroke="#0000ff" stroke-width="0.5" fill="none"/>
  <circle cx="70" cy="20" r="5" stroke="#000000" fill="none"/>
  <g id="pair"><rect x="0" y="0" width="1" height="1" fill="#000"/><rect x="2" y="0" width="1" height="1" fill="#000"/></g>
</svg>)");
    const ImportResult r = import_svg(f.string());
    REQUIRE(r.ok());
    REQUIRE(r.shapes.size() == 6);   // rect, line, circle, group + 2 members
    // Red fill -> C02, closed; flipped to Y up: SVG y 10..20 of a 50 tall canvas -> 30..40.
    CHECK(r.shapes[0].layer == 2);
    CHECK(r.shapes[0].paths[0].closed);
    const BoundingBoxf b0 = shape_bounds(r, 0);
    CHECK_THAT(b0.min.x(), WithinAbs(10, 1e-3));
    CHECK_THAT(b0.min.y(), WithinAbs(30, 1e-3));
    CHECK_THAT(b0.max.y(), WithinAbs(40, 1e-3));
    // Blue stroke-only open line -> C01, open.
    CHECK(r.shapes[1].layer == 1);
    CHECK_FALSE(r.shapes[1].paths[0].closed);
    // Stroked circle keeps the closed flag, flattened to 0.02 mm: its area is close to pi r^2.
    CHECK(r.shapes[2].paths[0].closed);
    CHECK_THAT(std::abs(Polygon(r.shapes[2].paths[0].pts.points).area()) * SCALING_FACTOR * SCALING_FACTOR, WithinRel(M_PI * 25, 0.01));
    // Group.
    CHECK(r.shapes[3].type == ShapeType::Group);
    CHECK(r.shapes[4].parent == 3);
    CHECK(r.shapes[5].parent == 3);

    SECTION("px at 96 dpi")
    {
        ScopedTemporaryFile p(".svg");
        write(p.string(), R"(<svg xmlns="http://www.w3.org/2000/svg" width="96" height="96"><rect x="0" y="0" width="96" height="48" fill="#000"/></svg>)");
        const ImportResult rp = import_svg(p.string());
        REQUIRE(rp.ok());
        const BoundingBoxf b = shape_bounds(rp, 0);
        CHECK_THAT(b.size().x(), WithinAbs(25.4, 1e-3));
        CHECK_THAT(b.size().y(), WithinAbs(12.7, 1e-3));
    }
    SECTION("missing file")
    {
        CHECK_FALSE(import_svg("/nonexistent/file.svg").ok());
    }
}

TEST_CASE("DXF import: entities, bulges, blocks, units and layers", "[LaserImport]")
{
    // Inches; layer "CUT" has ACI 1 (red) -> C02; layer "5" -> C05.
    const std::string dxf = R"(0
SECTION
2
HEADER
9
$INSUNITS
70
1
0
ENDSEC
0
SECTION
2
TABLES
0
TABLE
2
LAYER
0
LAYER
2
CUT
62
1
0
ENDTAB
0
ENDSEC
0
SECTION
2
BLOCKS
0
BLOCK
2
SQ
10
0
20
0
0
LWPOLYLINE
8
0
90
4
70
1
10
0
20
0
10
1
20
0
10
1
20
1
10
0
20
1
0
ENDBLK
0
ENDSEC
0
SECTION
2
ENTITIES
0
LINE
8
CUT
62
256
10
0
20
0
11
1
21
0
0
CIRCLE
8
5
10
2
20
2
40
0.5
0
ARC
8
0
10
0
20
0
40
1
50
0
51
90
0
LWPOLYLINE
8
0
90
2
70
0
10
0
20
0
42
1
10
2
20
0
0
INSERT
8
0
2
SQ
10
3
20
0
41
2
42
2
0
SPLINE
8
0
70
8
71
3
72
8
73
4
40
0
40
0
40
0
40
0
40
1
40
1
40
1
40
1
10
0
20
0
10
1
20
1
10
2
20
1
10
3
20
0
0
HATCH
8
0
0
ENDSEC
0
EOF
)";
    ScopedTemporaryFile f(".dxf");
    write(f.string(), dxf);
    const ImportResult r = import_dxf(f.string());
    INFO(r.error);
    REQUIRE(r.ok());
    // LINE, CIRCLE, ARC, LWPOLYLINE, INSERT group + its polyline, SPLINE.
    REQUIRE(r.shapes.size() == 7);
    // LINE: 1 inch long, BYLAYER red -> C02.
    CHECK(r.shapes[0].layer == 2);
    CHECK_THAT(shape_bounds(r, 0).size().x(), WithinAbs(25.4, 1e-3));
    // CIRCLE on layer "5" -> C05, closed, diameter 1 in.
    CHECK(r.shapes[1].layer == 5);
    CHECK(r.shapes[1].paths[0].closed);
    CHECK_THAT(shape_bounds(r, 1).size().x(), WithinAbs(25.4, 0.05));
    // ARC 0..90 deg: ends at (0, 1) in.
    CHECK_THAT(unscale(r.shapes[2].paths[0].pts.last_point()).y(), WithinAbs(25.4, 1e-3));
    // Bulge 1 = half circle CCW from (0,0) to (2,0): dips to y = -1 in (centre (1,0), CCW goes below).
    const BoundingBoxf bb = shape_bounds(r, 3);
    CHECK_THAT(bb.min.y(), WithinAbs(-25.4, 0.05));
    CHECK_THAT(bb.max.y(), WithinAbs(0, 0.05));
    // INSERT: a group holding the 1 in square scaled by 2 at x = 3 in.
    CHECK(r.shapes[4].type == ShapeType::Group);
    CHECK(r.shapes[5].parent == 4);
    const BoundingBoxf bi = shape_bounds(r, 5);
    CHECK_THAT(bi.min.x(), WithinAbs(3 * 25.4, 1e-3));
    CHECK_THAT(bi.size().x(), WithinAbs(2 * 25.4, 1e-3));
    // SPLINE (clamped cubic Bezier): starts and ends on its end control points, peaks at y = 0.75 in.
    const LaserPath& sp = r.shapes[6].paths[0];
    CHECK(unscale(sp.pts.first_point()).isApprox(Vec2d(0, 0)));
    CHECK_THAT(unscale(sp.pts.last_point()).x(), WithinAbs(3 * 25.4, 1e-3));
    CHECK_THAT(shape_bounds(r, 6).max.y(), WithinAbs(0.75 * 25.4, 0.01));
    // HATCH is reported, not imported.
    REQUIRE(r.warnings.size() == 1);
    CHECK_THAT(r.warnings[0], Catch::Matchers::ContainsSubstring("HATCH"));

    SECTION("binary or garbage DXF is refused")
    {
        ScopedTemporaryFile g(".dxf");
        write(g.string(), "AutoCAD Binary DXF\r\n\x1a");
        CHECK_FALSE(import_dxf(g.string()).ok());
    }
}

TEST_CASE("DXF import survives malformed entities", "[LaserImport]")
{
    // An ARC with an absurd angle (a += 2 pi loop would hang), a cubic SPLINE with two control
    // points (de Boor would read past them), a NaN LINE and a block inserting itself ten times.
    std::string dxf = "0\nSECTION\n2\nBLOCKS\n0\nBLOCK\n2\nB\n10\n0\n20\n0\n";
    for (int i = 0; i < 10; ++i) dxf += "0\nINSERT\n2\nB\n10\n1\n20\n1\n";
    dxf += "0\nLINE\n10\n0\n20\n0\n11\n1\n21\n1\n0\nENDBLK\n0\nENDSEC\n"
           "0\nSECTION\n2\nENTITIES\n"
           "0\nARC\n10\n0\n20\n0\n40\n5\n50\n1e300\n51\n-1e300\n"
           "0\nARC\n10\n0\n20\n0\n40\n5\n50\n0\n51\ninf\n"
           "0\nSPLINE\n71\n3\n10\n0\n20\n0\n10\n5\n20\n5\n"
           "0\nLINE\n10\nnan\n20\n0\n11\n1\n21\n1\n"
           "0\nINSERT\n2\nB\n10\n0\n20\n0\n0\nENDSEC\n0\nEOF\n";
    ScopedTemporaryFile f(".dxf");
    write(f.string(), dxf);
    const ImportResult r = import_dxf(f.string());
    REQUIRE(r.ok());
    CHECK(r.shapes.size() <= 250000);
    bool capped = false;
    for (const std::string& w : r.warnings) capped |= w.find("250000") != std::string::npos;
    CHECK(capped);
    bool sane = true;   // no NaN / overflowed coordinates
    for (const LaserShape& s : r.shapes)
        for (const LaserPath& p : s.paths)
            for (const Point& pt : p.pts.points) sane &= std::abs(pt.x()) < scale_(1e7) && std::abs(pt.y()) < scale_(1e7);
    CHECK(sane);
}

TEST_CASE("Image import: grayscale, alpha on white, size from dpi", "[LaserImport]")
{
    ScopedTemporaryFile f(".png");
    write(f.string(), unbase64(kGrayPng));
    ImportResult r = import_image(f.string(), 25.4);   // 1 mm pixels
    REQUIRE(r.ok());
    REQUIRE(r.shapes.size() == 1);
    const LaserShape& s = r.shapes[0];
    CHECK(s.type == ShapeType::Image);
    CHECK(s.image_w == 3);
    CHECK(s.image_h == 2);
    CHECK(s.gray == std::vector<uint8_t>{0, 128, 255, 255, 255, 0});
    CHECK_THAT(s.width_mm, WithinAbs(3, 1e-9));
    CHECK_THAT(s.height_mm, WithinAbs(2, 1e-9));
    CHECK(s.xform.translation().isApprox(Vec2d(1.5, 1)));

    ScopedTemporaryFile a(".png");
    write(a.string(), unbase64(kRgbaPng));
    r = import_image(a.string());
    REQUIRE(r.ok());
    CHECK(r.shapes[0].gray == std::vector<uint8_t>{0, 255});   // transparent -> white
    CHECK_THAT(r.shapes[0].width_mm, WithinAbs(0.2, 1e-9));    // 254 dpi default

    CHECK_FALSE(import_image("/nonexistent.png").ok());
    CHECK_FALSE(import_file("thing.xyz").ok());
}

TEST_CASE("LightBurn import: cut settings and every shape type", "[LaserImport]")
{
    const std::string xml = std::string(R"(<?xml version="1.0" encoding="UTF-8"?>
<LightBurnProject AppVersion="1.4.03" FormatVersion="1">
  <CutSetting type="Scan">
    <index Value="3"/>
    <name Value="Engrave"/>
    <speed Value="250"/>
    <maxPower Value="35"/>
    <minPower Value="5"/>
    <numPasses Value="2"/>
    <priority Value="1"/>
    <interval Value="0.08"/>
    <angle Value="45"/>
    <crossHatch Value="1"/>
  </CutSetting>
  <CutSetting_Img type="Image">
    <index Value="4"/>
    <ditherMode Value="stucki"/>
    <dpi Value="318"/>
  </CutSetting_Img>
  <Shape Type="Rect" CutIndex="3" W="20" H="10" Cr="2">
    <XForm>1 0 0 1 50 60</XForm>
  </Shape>
  <Shape Type="Ellipse" CutIndex="0" Rx="5" Ry="3">
    <XForm>2 0 0 1 10 10</XForm>
  </Shape>
  <Shape Type="Path" CutIndex="1">
    <XForm>1 0 0 1 0 0</XForm>
    <VertList>V0 0c0x0c0y5V10 0c1x10c1y5V10 -5</VertList>
    <PrimList>B0 1L1 2L2 0</PrimList>
  </Shape>
  <Shape Type="Group">
    <XForm>1 0 0 1 100 0</XForm>
    <Children>
      <Shape Type="Rect" CutIndex="0" W="4" H="4"><XForm>1 0 0 1 0 0</XForm></Shape>
      <Shape Type="Rect" CutIndex="0" W="4" H="4"><XForm>1 0 0 1 10 0</XForm></Shape>
    </Children>
  </Shape>
  <Shape Type="Bitmap" CutIndex="4" W="30" H="20" Data=")") + kGrayPng + R"(">
    <XForm>1 0 0 1 5 5</XForm>
  </Shape>
  <Shape Type="Unknown"/>
</LightBurnProject>)";
    ScopedTemporaryFile f(".lbrn2");
    write(f.string(), xml);
    const ImportResult r = import_file(f.string());
    INFO(r.error);
    REQUIRE(r.ok());
    REQUIRE(r.cut_settings.size() == 2);
    const LaserLayer& l = r.cut_settings[0].second;
    CHECK(r.cut_settings[0].first == 3);
    CHECK(l.name == "Engrave");
    CHECK(l.mode == LayerMode::Fill);
    CHECK(l.speed_mm_s == 250);
    CHECK(l.power_max == 35);
    CHECK(l.power_min == 5);
    CHECK(l.passes == 2);
    CHECK(l.priority == 1);
    CHECK(l.interval_mm == 0.08);
    CHECK(l.angle_deg == 45);
    CHECK(l.crosshatch);
    CHECK(r.cut_settings[1].second.image_dither == DitherMode::Stucki);
    CHECK(r.cut_settings[1].second.image_dpi == 318);

    REQUIRE(r.shapes.size() == 7);   // rect, ellipse, path, group + 2, bitmap
    CHECK(r.shapes[0].type == ShapeType::Rect);
    CHECK(r.shapes[0].layer == 3);
    CHECK(r.shapes[0].corner_radius == 2);
    CHECK(r.shapes[0].xform.translation().isApprox(Vec2d(50, 60)));
    CHECK_THAT(shape_bounds(r, 1).size().x(), WithinAbs(20, 1e-3));   // rx 5 scaled x2
    // Path: a Bezier bulging up to y = 3.75, then two lines back: closed.
    REQUIRE(r.shapes[2].paths.size() == 1);
    CHECK(r.shapes[2].paths[0].closed);
    const BoundingBoxf bp = shape_bounds(r, 2);
    CHECK_THAT(bp.max.y(), WithinAbs(3.75, 0.03));
    CHECK_THAT(bp.min.y(), WithinAbs(-5, 1e-6));
    // Group children compose with the group's XForm.
    CHECK(r.shapes[3].type == ShapeType::Group);
    CHECK(r.shapes[5].parent == 3);
    CHECK(r.shapes[5].xform.translation().isApprox(Vec2d(110, 0)));
    // Bitmap.
    CHECK(r.shapes[6].type == ShapeType::Image);
    CHECK(r.shapes[6].image_w == 3);
    CHECK(r.shapes[6].width_mm == 30);
    CHECK(r.warnings.size() == 1);   // the Unknown shape
}
