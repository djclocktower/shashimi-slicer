#include <catch2/catch_all.hpp>

#include "libslic3r/CAM/Rotary.hpp"

#include <cmath>

using namespace Slic3r;
using namespace Slic3r::CAM;

namespace {

struct Job {
    CamDocument   doc;
    CamModel      model;
    CamOperation& op() { return doc.operations.front(); }
};

// Cylinder stock of radius r along X (0..len), axis on setup X.
Job rotary_job(OpType type, double r, double len, const TriangleMesh* mesh = nullptr)
{
    Job     j;
    CamTool t;
    t.number   = 1;
    t.type     = ToolType::BallEndMill;
    t.diameter = 6;
    j.doc.tools = {t};
    CamSetup s;
    s.stock.kind = StockKind::Cylinder;
    j.doc.setups = {s};
    CamOperation op;
    op.type        = type;
    op.tool_number = 1;
    j.doc.operations = {op};
    CamSetupFrame f;
    f.stock        = BoundingBoxf3(Vec3d(0, -r, -r), Vec3d(len, r, r));
    f.model        = mesh ? mesh->bounding_box() : f.stock;
    f.stock_radius = r;
    j.model.setups = {f};
    if (mesh) {
        CamBody b;
        b.body_id = 0;
        b.mesh    = *mesh;
        j.model.bodies.push_back(b);
    }
    return j;
}

} // namespace

TEST_CASE("Indexing 90 degrees about X maps +Y to +Z", "[CamRotary]")
{
    const Vec3d v = apply_index(Transform3d::Identity(), 90) * Vec3d(0, 1, 0);
    REQUIRE(v.isApprox(Vec3d(0, 0, 1), 1e-12));
    const Vec3d w = apply_index(Transform3d(Eigen::Translation3d(5, 0, 0)), 90) * Vec3d(0, 1, 0);
    REQUIRE(w.isApprox(Vec3d(5, 0, 1), 1e-12));
}

TEST_CASE("Wrapping a line 2 pi r long turns A through 360 degrees", "[CamRotary]")
{
    const double r = 10;
    Job          j = rotary_job(OpType::RotaryWrap, r, 100);
    CamSketch    sk;
    sk.feature = 5;
    sk.chains.push_back(Polyline(Point::new_scale(10, 0), Point::new_scale(10, 2 * M_PI * r)));
    j.model.sketches.push_back(sk);
    j.op().geom.sketch_feature = 5;
    j.op().wrap_strategy       = OpType::Engrave;
    j.op().heights.bottom      = Height{HeightRef::StockTop, -0.5};
    j.op().feeds_auto          = false;
    j.op().feeds.feed          = 600;

    const Toolpath tp = generate_rotary_wrap(j.doc, j.op(), j.model);
    REQUIRE(tp.ok());
    double a_max = -1, a_min = 1e9;
    int    feeds = 0;
    for (size_t k = 1; k < tp.moves.size(); ++k) {
        const Move& m = tp.moves[k];
        REQUIRE(m.to.y() == 0);
        if (m.kind != Move::Kind::Feed)
            continue;
        ++feeds;
        REQUIRE(m.to.z() == Catch::Approx(9.5));
        REQUIRE(m.to.x() == Catch::Approx(10));
        a_max = std::max(a_max, m.a_deg);
        a_min = std::min(a_min, tp.moves[k - 1].a_deg);
        // pure A move: the programmed feed (deg/min) makes the surface speed 600 mm/min at r
        if (std::abs(m.a_deg - tp.moves[k - 1].a_deg) > 1)
            REQUIRE(m.feed * M_PI / 180. * r == Catch::Approx(600));
    }
    REQUIRE(feeds >= 1);
    REQUIRE(a_min == Catch::Approx(0).margin(1e-9));
    REQUIRE(a_max == Catch::Approx(360).margin(1e-6));
}

TEST_CASE("Rotary finishing of a cylinder keeps a constant radius", "[CamRotary]")
{
    // cylinder r = 20 along X, 0..60
    TriangleMesh cyl = make_cylinder(20, 60);
    cyl.transform(Transform3d(Eigen::AngleAxisd(M_PI / 2, Vec3d::UnitY())));
    REQUIRE(cyl.bounding_box().min.x() == Catch::Approx(0).margin(1e-4));

    for (bool spiral : {false, true}) {
        Job j                     = rotary_job(OpType::RotaryFinish, 21, 60, &cyl);
        j.op().a_stepover_deg     = 10;
        j.op().stepover           = 2;
        j.op().rotary_spiral      = spiral;
        const Toolpath tp         = generate_rotary_finish(j.doc, j.op(), j.model);
        REQUIRE(tp.ok());
        size_t n = 0;
        double a_end = 0;
        for (size_t k = 1; k < tp.moves.size(); ++k) {
            const Move& m = tp.moves[k];
            if (m.kind != Move::Kind::Feed || (!spiral && m.a_deg != tp.moves[k - 1].a_deg))
                continue;
            ++n;
            INFO("spiral " << spiral << " x " << m.to.x() << " a " << m.a_deg);
            REQUIRE(m.to.z() == Catch::Approx(20).margin(0.01));
            REQUIRE(m.to.y() == 0);
            a_end = std::max(a_end, m.a_deg);
        }
        REQUIRE(n > 30);
        REQUIRE(a_end >= (spiral ? 360. * 29 : 350. - 1e-6));
    }
}
