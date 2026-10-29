#include <catch2/catch_all.hpp>

#include "libslic3r/Metrology/Metrology.hpp"

#include <cmath>
#include <random>

using namespace Slic3r::Metrology;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

double region_area(const std::vector<Region>& regions)
{
    double a = 0;
    for (const Region& r : regions) {
        a += signed_area(r.outer);
        for (const Polygon2& h : r.holes) a += signed_area(h);
    }
    return a;
}

// A camera with no edge bias, so silhouettes sit on the true edges.
SimCamera clean_camera()
{
    SimCamera cam;
    cam.edge_bias_mm = 0;
    return cam;
}

// Light-on minus light-off frame of a scene.
GrayImage lit_frame(const SimScene& scene, const SimCamera& cam, double offset, unsigned seed)
{
    return subtract(sim_render(scene, cam, offset, true, seed), sim_render(scene, cam, offset, false, seed + 1));
}

CameraModel calibrated_camera(const SimScene& scene, const SimCamera& sc)
{
    const PlateCalibration ref = calibrate_mat(lit_frame(scene, sc, 0, 11), scene.mat);
    CameraModel            cam = camera_from(ref);
    const PlateCalibration up  = calibrate_mat(lit_frame(scene, sc, 10, 21), scene.mat);
    std::string            err;
    calibrate_camera_center(cam, {0.0, 10.0}, {ref, up}, &err);
    return cam;
}

} // namespace

TEST_CASE("A homography fitted to exact correspondences maps every point back", "[Metrology]")
{
    Mat3 truth;
    truth.m = {1.7, 0.2, 310, -0.1, -1.6, 420, 0.0004, -0.0002, 1};
    std::vector<P2> src, dst;
    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 4; ++j) {
            src.emplace_back(i * 40.0, j * 30.0);
            dst.push_back(truth.apply(src.back()));
        }
    Mat3 H;
    REQUIRE(fit_homography(src, dst, H));
    for (size_t i = 0; i < src.size(); ++i) {
        CHECK_THAT(H.apply(src[i]).x, WithinAbs(dst[i].x, 1e-6));
        CHECK_THAT(H.apply(src[i]).y, WithinAbs(dst[i].y, 1e-6));
    }
}

TEST_CASE("A circle fit recovers centre and radius from noisy edge points", "[Metrology]")
{
    std::mt19937                     rng(3);
    std::normal_distribution<double> n(0, 0.01);
    std::vector<P2>                  pts;
    for (int i = 0; i < 60; ++i) {
        const double a = 2 * kPi * i / 60;
        pts.emplace_back(12.5 + 4.2 * std::cos(a) + n(rng), -3 + 4.2 * std::sin(a) + n(rng));
    }
    pts.emplace_back(30, 30);   // an outlier
    const CircleFit c = fit_circle_robust(pts);
    REQUIRE(c.ok);
    CHECK_THAT(c.c.x, WithinAbs(12.5, 0.01));
    CHECK_THAT(c.c.y, WithinAbs(-3.0, 0.01));
    CHECK_THAT(c.r, WithinAbs(4.2, 0.01));
}

TEST_CASE("Contours of a rasterized square with a hole keep the hole and the area", "[Metrology]")
{
    Region r;
    r.outer = sim_rect(0, 0, 20, 10);
    r.holes.push_back(sim_circle({10, 5}, 3, 64, false));
    const Field f = rasterize({r}, -2, -2, 240, 140, 0.1);
    const auto  regions = contour_regions(f, 0.5f, true, 0.02, 0.5);
    REQUIRE(regions.size() == 1);
    CHECK(regions[0].holes.size() == 1);
    CHECK(signed_area(regions[0].outer) > 0);
    CHECK(signed_area(regions[0].holes[0]) < 0);
    CHECK_THAT(region_area(regions), WithinRel(200 - kPi * 9, 0.01));
}

TEST_CASE("The mat calibration matches the simulated camera's homography", "[Metrology]")
{
    const SimScene         scene = sim_preset_empty();
    const SimCamera        sc;
    const PlateCalibration cal = calibrate_mat(lit_frame(scene, sc, 0, 1), scene.mat);
    REQUIRE(cal.ok);
    CHECK(cal.matched >= int(0.95 * cal.expected));
    CHECK(cal.rms_mm < 0.05);
    const Mat3 truth = sim_plate_homography(sc);
    for (P2 p : {P2{0, 0}, P2{90, 70}, P2{180, 140}, P2{30, 120}}) {
        CHECK((cal.mm_to_px.apply(p) - truth.apply(p)).norm() < 0.3);
    }
}

TEST_CASE("Mat frames at two bed heights locate the camera centre", "[Metrology]")
{
    const SimScene    scene = sim_preset_empty();
    const SimCamera   sc;
    const CameraModel cam = calibrated_camera(scene, sc);
    REQUIRE(cam.has_center);
    CHECK_THAT(cam.cz, WithinRel(sc.center.z, 0.02));
    CHECK((cam.nadir - sim_nadir(sc)).norm() < 5.0);
}

TEST_CASE("The bed height is read back from the mat's dots", "[Metrology]")
{
    const SimScene    scene = sim_preset_empty();
    const SimCamera   sc;
    const CameraModel cam   = calibrated_camera(scene, sc);
    REQUIRE(cam.has_center);
    const double           raised = GENERATE(1.0, 3.0, 6.5);
    const PlateCalibration fr     = calibrate_mat(lit_frame(scene, sc, raised, 31), scene.mat);
    REQUIRE(fr.ok);
    double off = 0;
    REQUIRE(plane_offset(cam, fr, off));
    CHECK_THAT(off, WithinAbs(raised, 0.05));
}

TEST_CASE("A circle feature measures a thin disc's diameter", "[Metrology]")
{
    SimScene scene = sim_preset_empty();
    SimPrism disc;
    disc.footprint.outer = sim_circle({90, 70}, 10, 180);
    disc.height          = 0.3;
    scene.prisms.push_back(disc);
    const SimCamera   sc  = clean_camera();
    const CameraModel cam = calibrated_camera(scene, sc);
    const GrayImage   img = lit_frame(scene, sc, 0, 41);
    Program           prog;
    Feature           f;
    f.kind    = FeatureKind::Circle;
    f.center  = cam.to_px({90, 70}, 0.3);
    f.r_in    = 10 / cam.mm_per_px * 0.6;
    f.r_out   = 10 / cam.mm_per_px * 1.4;
    f.scans   = 48;
    f.plane_z = 0.3;
    f.tol     = {true, 20.0, 0.1, 0.1};
    prog.add(f);
    prog.run(img, cam);
    const Feature& m = prog.features.front();
    REQUIRE(m.ok);
    CHECK_THAT(m.value, WithinAbs(20.0, 0.1));
    CHECK(prog.overall() == Judge::OK);
}

TEST_CASE("Reverse engineering rebuilds the bracket's levels and outline", "[Metrology][Reverse]")
{
    const SimScene    scene = sim_preset_bracket();
    const SimCamera   sc    = clean_camera();
    const CameraModel cam   = calibrated_camera(scene, sc);
    REQUIRE(cam.has_center);
    const GrayImage img = lit_frame(scene, sc, 0, 51);
    const auto silhouette = trace_silhouette(img, cam, {50, 35}, {135, 100}, 0.1, -1, true, 0.03, 2.0);
    REQUIRE(silhouette.size() == 1);

    ReSettings set;
    set.grid = 5;
    ReSession s;
    s.begin(silhouette, cam, set);
    int guard = 0;
    while (s.stage() != ReSession::Stage::Done && guard++ < 40) {
        const auto batch = s.next_batch();
        std::vector<ReResult> res;
        for (const ReTouch& t : batch) res.push_back({t.id, sim_touch(scene, t.xy, set.tip_radius)});
        s.submit(res);
    }
    REQUIRE(s.stage() == ReSession::Stage::Done);
    const ReModel model = s.build();
    REQUIRE(model.levels.size() == 3);
    CHECK_THAT(model.levels[0].z, WithinAbs(4.0, 0.01));
    CHECK_THAT(model.levels[1].z, WithinAbs(7.0, 0.01));
    CHECK_THAT(model.levels[2].z, WithinAbs(10.0, 0.01));
    // Footprint: 64 x 44 with two 6.4 mm through holes.
    CHECK_THAT(region_area(model.footprint), WithinRel(64 * 44 - 2 * kPi * 3.2 * 3.2, 0.02));
    // At least 10 mm tall: the 20 x 44 wall.
    CHECK_THAT(region_area(model.levels[2].regions), WithinRel(20.0 * 44, 0.03));
    // At least 7 mm tall: the wall plus the 14 mm boss.
    CHECK_THAT(region_area(model.levels[1].regions), WithinRel(20.0 * 44 + kPi * 49, 0.04));
}

TEST_CASE("Outline anchors measure the camera's edge bias", "[Metrology][Reverse]")
{
    const SimScene    scene = sim_preset_coupon();
    SimCamera         sc;
    sc.edge_bias_mm         = 0.08;
    const CameraModel cam   = calibrated_camera(scene, sc);
    REQUIRE(cam.has_center);
    const auto silhouette = trace_silhouette(lit_frame(scene, sc, 0, 61), cam, {50, 40}, {130, 100}, 0.1, -1, true, 0.03, 2.0);
    REQUIRE(silhouette.size() == 1);
    ReSettings set;
    ReSession  s;
    s.begin(silhouette, cam, set);
    for (int guard = 0; s.stage() != ReSession::Stage::Done && guard < 40; ++guard) {
        std::vector<ReResult> res;
        for (const ReTouch& t : s.next_batch()) res.push_back({t.id, sim_touch(scene, t.xy, set.tip_radius)});
        s.submit(res);
    }
    const ReModel model = s.build();
    REQUIRE(model.anchors >= 2);
    CHECK_THAT(model.camera_bias, WithinAbs(0.08, 0.03));
    // 60 x 40 slab with a 10 mm hole, once the bias is removed.
    CHECK_THAT(region_area(model.footprint), WithinRel(60.0 * 40 - kPi * 25, 0.01));
}

TEST_CASE("A probe batch touches the mat first and every point after it", "[Metrology]")
{
    ProbeSettings s;
    s.mat_x       = 50;
    s.mat_y       = 40;
    s.mat_rot_deg = 90;
    CHECK((mat_to_printer({10, 0}, s) - P2{50, 50}).norm() < 1e-9);
    const std::string g = probe_gcode({{{20, 20}, "a"}, {{30, 20}, "b"}}, s, 0, 10, {5, 5});
    size_t            n = 0;
    for (size_t p = g.find("G380 S3"); p != std::string::npos; p = g.find("G380 S3", p + 1)) ++n;
    CHECK(n == 3);
    CHECK(g.find("mat reference") < g.find("touch 1"));
}

TEST_CASE("Still periods are found between motion", "[Metrology]")
{
    const std::vector<double> motion{9, 9, 0.1, 0.2, 0.1, 9, 9, 0.1, 0.1, 0.1, 0.1, 9};
    const auto                st = find_stills(motion, 1.0, 3);
    REQUIRE(st.size() == 2);
    CHECK(st[0].first == 2);
    CHECK(st[1].first == 7);
}
