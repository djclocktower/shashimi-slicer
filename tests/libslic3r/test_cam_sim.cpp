#include <catch2/catch_all.hpp>

#include "libslic3r/CAM/StockSim.hpp"

#include <cmath>

using namespace Slic3r;
using namespace Slic3r::CAM;

namespace {

CamTool make_tool(ToolType type, double d)
{
    CamTool t;
    t.type     = type;
    t.diameter = d;
    return t;
}

// Rapid over, plunge to z = 7, slot along X from x = 5 to 35 at y = 10.
Toolpath slot()
{
    Toolpath tp;
    const auto add = [&](Move::Kind k, double x, double y, double z) {
        Move m;
        m.kind = k;
        m.to   = Vec3d(x, y, z);
        tp.moves.push_back(m);
    };
    add(Move::Kind::Rapid, 5, 10, 15);
    add(Move::Kind::Plunge, 5, 10, 7);
    add(Move::Kind::Feed, 35, 10, 7);
    add(Move::Kind::Retract, 35, 10, 15);
    return tp;
}

} // namespace

TEST_CASE("A flat end mill slot has the tool width and the cut depth", "[CamSim]")
{
    StockSim sim;
    sim.init_box(BoundingBoxf3(Vec3d(0, 0, 0), Vec3d(40, 20, 10)), 0.1);
    sim.cut_upto(make_tool(ToolType::FlatEndMill, 6), slot(), 3);
    REQUIRE(sim.top_at(20, 10) == Catch::Approx(7));
    REQUIRE(sim.top_at(20, 12.9) == Catch::Approx(7));
    REQUIRE(sim.top_at(20, 7.1) == Catch::Approx(7));
    REQUIRE(sim.top_at(20, 13.1) == Catch::Approx(10));
    REQUIRE(sim.top_at(20, 6.9) == Catch::Approx(10));
    REQUIRE(sim.top_at(1.9, 10) == Catch::Approx(10)); // plunge at x = 5, r = 3
    REQUIRE(sim.top_at(38.1, 10) == Catch::Approx(10));
    // 30 mm slot + round ends, 3 mm deep
    REQUIRE(sim.removed_volume() == Catch::Approx((30 * 6 + M_PI * 9) * 3).epsilon(0.02));

    // restart: back to fresh stock
    sim.cut_upto(make_tool(ToolType::FlatEndMill, 6), slot(), 0);
    REQUIRE(sim.removed_volume() == 0);
}

TEST_CASE("A ball end mill slot has a round bottom", "[CamSim]")
{
    StockSim sim;
    sim.init_box(BoundingBoxf3(Vec3d(0, 0, 0), Vec3d(40, 20, 10)), 0.1);
    sim.cut_upto(make_tool(ToolType::BallEndMill, 6), slot(), 3);
    double lowest = 1e9, lowest_y = 0;
    for (double dy = -2.95; dy <= 2.95; dy += 0.1) {
        const double z = sim.top_at(20, 10 + dy);
        if (z < lowest) {
            lowest   = z;
            lowest_y = dy;
        }
        INFO("dy " << dy);
        REQUIRE(z == Catch::Approx(7 + 3 - std::sqrt(9 - dy * dy)).margin(0.1));
    }
    REQUIRE(std::abs(lowest_y) <= 0.1);
    REQUIRE(lowest == Catch::Approx(7).margin(0.01));
}

TEST_CASE("The simulated stock mesh volume matches the removed material", "[CamSim]")
{
    const BoundingBoxf3 stock(Vec3d(0, 0, 0), Vec3d(40, 20, 10));
    StockSim            sim;
    sim.init_box(stock, 0.2);
    sim.cut_upto(make_tool(ToolType::FlatEndMill, 6), slot(), 3);
    const TriangleMesh m = sim.to_mesh();
    REQUIRE(its_volume(m.its) > 0);
    REQUIRE(its_volume(m.its) == Catch::Approx(40. * 20. * 10. - sim.removed_volume()).epsilon(0.01));

    // cylinder stock: a flat cut across the top at A = 0 from a 10 mm radius down to 8
    StockSim cyl;
    cyl.init_cylinder(0, 40, 10, 0.2);
    Move a, b;
    a.to = Vec3d(-5, 0, 8);
    b.to = Vec3d(45, 0, 8);
    cyl.cut(make_tool(ToolType::FlatEndMill, 20), a, b);
    const TriangleMesh cm = cyl.to_mesh();
    REQUIRE(its_volume(cm.its) > 0);
    // circular segment above z = 8: r^2 acos(d/r) - d sqrt(r^2 - d^2)
    const double seg = 100 * std::acos(0.8) - 8 * 6;
    REQUIRE(cyl.removed_volume() == Catch::Approx(seg * 40).epsilon(0.05));
    REQUIRE(its_volume(cm.its) == Catch::Approx(M_PI * 100 * 40 - cyl.removed_volume()).epsilon(0.02));
}
