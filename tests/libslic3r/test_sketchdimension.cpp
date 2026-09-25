#include <catch2/catch_all.hpp>
using Catch::Approx;

#include "libslic3r/CAD/SketchDimension.hpp"
#include "libslic3r/CAD/SketchSolver.hpp"
#include "libslic3r/CAD/CadDocument.hpp"

#include <cereal/archives/binary.hpp>

#include <cmath>
#include <cstring>
#include <fstream>
#include <iterator>
#include <sstream>

using namespace Slic3r;
using K  = SketchDimKind;
using R  = SketchPointRole;
using CT = SketchConstraintType;

namespace {

SketchEntity line(Vec2d a, Vec2d b)
{
    SketchEntity e; e.type = SketchEntity::Type::Line; e.p0 = a; e.p1 = b; return e;
}
SketchEntity circle(Vec2d c, double r)
{
    SketchEntity e; e.type = SketchEntity::Type::Circle; e.center = c; e.p0 = c; e.radius = r; return e;
}
SketchEntity point(Vec2d p)
{
    SketchEntity e; e.type = SketchEntity::Type::Point; e.p0 = p; return e;
}
// CCW arc from a0 to a1 (radians).
SketchEntity arc(Vec2d c, double r, double a0, double a1)
{
    SketchEntity e; e.type = SketchEntity::Type::Arc; e.center = c; e.radius = r;
    e.start_angle = a0; e.end_angle = a1;
    e.p0 = c + r * Vec2d(std::cos(a0), std::sin(a0));
    e.p1 = c + r * Vec2d(std::cos(a1), std::sin(a1));
    return e;
}
SketchEntityConstraintDef con(CT t, int ea, R ra, int eb = -1, R rb = R::P0, double v = 0.0)
{
    SketchEntityConstraintDef c; c.type = t; c.ea = ea; c.ra = ra; c.eb = eb; c.rb = rb; c.value = v; return c;
}
SmartDimResolution pick1(const std::vector<SketchEntity>& ents, SmartDimPick a, Vec2d cursor)
{
    return resolve_smart_dimension(ents, a, std::nullopt, cursor);
}
SmartDimResolution pick2(const std::vector<SketchEntity>& ents, SmartDimPick a, SmartDimPick b, Vec2d cursor)
{
    return resolve_smart_dimension(ents, a, b, cursor);
}
double angle_deg(const Vec2d& u, const Vec2d& v)
{
    return std::acos(std::clamp(u.normalized().dot(v.normalized()), -1.0, 1.0)) * 180.0 / M_PI;
}

// Drive `d` to `value` through its constraint (plus `extra` holding the rest of the sketch),
// solve, and return the re-measured value.
double drive(std::vector<SketchEntity>& ents, const SketchDimension& d, double value,
             std::vector<SketchEntityConstraintDef> extra = {})
{
    auto c = sketch_dimension_constraint(ents, d, value);
    REQUIRE(c.has_value());
    extra.push_back(*c);
    REQUIRE(sketch_solve(ents, extra).ok);
    double m = -1.0;
    REQUIRE(measure_sketch_dimension(ents, d, m));
    return m;
}

} // namespace

// ---- resolver: one entity --------------------------------------------------------------------

TEST_CASE("a single oblique line is dimensioned by the cursor zone", "[SketchDimension]")
{
    std::vector<SketchEntity> ents = { line({0, 0}, {10, 5}) };
    const SmartDimPick L = SmartDimPick::whole(0);

    SECTION("inside the perpendicular band -> aligned length") {
        // Midpoint (5,2.5) pushed along the normal: inside the band between the endpoints.
        const Vec2d n = Vec2d(-5, 10).normalized();
        auto r = pick1(ents, L, Vec2d(5, 2.5) + 4.0 * n);
        REQUIRE(r.ok);
        CHECK(r.dim.kind == K::Length);
        CHECK(r.dim.value == Approx(std::sqrt(125.0)));
    }
    SECTION("outside the band, cursor offset mostly vertical -> horizontal") {
        auto r = pick1(ents, L, Vec2d(20, 20));
        REQUIRE(r.ok);
        CHECK(r.dim.kind == K::Horizontal);
        CHECK(r.dim.value == Approx(10.0));
    }
    SECTION("outside the band, cursor offset mostly horizontal -> vertical") {
        auto r = pick1(ents, L, Vec2d(30, 3));
        REQUIRE(r.ok);
        CHECK(r.dim.kind == K::Vertical);
        CHECK(r.dim.value == Approx(5.0));
    }
    SECTION("the cursor is stored as the text position") {
        auto r = pick1(ents, L, Vec2d(30, 3));
        CHECK(r.dim.text_pos.isApprox(Vec2d(30, 3)));
    }
}

TEST_CASE("an axis-aligned line always gets a horizontal or vertical dimension", "[SketchDimension]")
{
    std::vector<SketchEntity> ents = { line({0, 0}, {10, 0}), line({3, 1}, {3, 8}) };
    for (Vec2d c : {Vec2d(5, 4), Vec2d(40, 1), Vec2d(-30, -2)}) {
        auto h = pick1(ents, SmartDimPick::whole(0), c);
        REQUIRE(h.ok);
        CHECK(h.dim.kind == K::Horizontal);
        CHECK(h.dim.value == Approx(10.0));
        auto v = pick1(ents, SmartDimPick::whole(1), c);
        REQUIRE(v.ok);
        CHECK(v.dim.kind == K::Vertical);
        CHECK(v.dim.value == Approx(7.0));
    }
}

TEST_CASE("horizontal references are ordered so the signed constraint stays positive", "[SketchDimension]")
{
    // A line drawn right-to-left: the horizontal dimension must not ask DistanceX for -10.
    std::vector<SketchEntity> ents = { line({10, 0}, {0, 6}) };
    auto r = pick1(ents, SmartDimPick::whole(0), Vec2d(5, 30));
    REQUIRE(r.ok);
    REQUIRE(r.dim.kind == K::Horizontal);
    auto c = sketch_dimension_constraint(ents, r.dim, 10.0);
    REQUIRE(c.has_value());
    CHECK(c->type == CT::DistanceX);
    const Vec2d P = c->ra == R::P0 ? ents[0].p0 : ents[0].p1;
    const Vec2d Q = c->rb == R::P0 ? ents[0].p0 : ents[0].p1;
    CHECK(Q.x() - P.x() == Approx(10.0));
    // And driving it keeps the line where it is pointing.
    CHECK(drive(ents, r.dim, 14.0, {con(CT::Fix, 0, R::P1)}) == Approx(14.0));
    CHECK(ents[0].p0.x() > ents[0].p1.x());
}

TEST_CASE("a circle gets a diameter and an arc a radius", "[SketchDimension]")
{
    std::vector<SketchEntity> ents = { circle({0, 0}, 6), arc({20, 0}, 4, 0.0, M_PI / 2) };
    auto d = pick1(ents, SmartDimPick::whole(0), Vec2d(10, 10));
    REQUIRE(d.ok);
    CHECK(d.dim.kind == K::Diameter);
    CHECK(d.dim.value == Approx(12.0));
    auto r = pick1(ents, SmartDimPick::whole(1), Vec2d(30, 10));
    REQUIRE(r.ok);
    CHECK(r.dim.kind == K::Radius);
    CHECK(r.dim.value == Approx(4.0));
}

TEST_CASE("a lone point measures nothing", "[SketchDimension]")
{
    std::vector<SketchEntity> ents = { point({1, 2}), line({0, 0}, {5, 0}) };
    CHECK_FALSE(pick1(ents, SmartDimPick::whole(0), Vec2d(3, 3)).ok);
    CHECK_FALSE(pick1(ents, SmartDimPick::at(1, R::P0), Vec2d(3, 3)).ok);
    CHECK_FALSE(pick1(ents, SmartDimPick::whole(kSketchRefOrigin), Vec2d(3, 3)).ok);
    // A second pick identical to the first is no second pick.
    auto same = pick2(ents, SmartDimPick::whole(1), SmartDimPick::whole(1), Vec2d(2, 5));
    REQUIRE(same.ok);
    CHECK(same.dim.kind == K::Horizontal);
}

// ---- resolver: two entities ------------------------------------------------------------------

TEST_CASE("two lines get the angle of the sector holding the cursor", "[SketchDimension]")
{
    std::vector<SketchEntity> ents = { line({0, 0}, {10, 0}), line({0, 0}, {10, 10}) };
    const SmartDimPick A = SmartDimPick::whole(0), B = SmartDimPick::whole(1);
    struct Case { Vec2d cursor; double deg; };
    for (const Case& c : {Case{{5, 1}, 45.0}, Case{{-5, 1}, 135.0}, Case{{1, -5}, 135.0}, Case{{-5, -1}, 45.0}}) {
        auto r = pick2(ents, A, B, c.cursor);
        REQUIRE(r.ok);
        CHECK(r.dim.kind == K::Angle);
        CHECK(r.dim.value == Approx(c.deg));
    }
    // Four distinct sectors.
    CHECK(pick2(ents, A, B, {5, 1}).dim.sector   == 0);
    CHECK(pick2(ents, A, B, {-5, 1}).dim.sector  == 1);
    CHECK(pick2(ents, A, B, {1, -5}).dim.sector  == 2);
    CHECK(pick2(ents, A, B, {-5, -1}).dim.sector == 3);
}

TEST_CASE("the angle sector works for lines that do not touch", "[SketchDimension]")
{
    // Intersection at (0,0) lies outside both segments.
    std::vector<SketchEntity> ents = { line({2, 0}, {10, 0}), line({3, 3 * std::sqrt(3.0)}, {5, 5 * std::sqrt(3.0)}) };
    auto r = pick2(ents, SmartDimPick::whole(0), SmartDimPick::whole(1), Vec2d(6, 2));
    REQUIRE(r.ok);
    CHECK(r.dim.kind == K::Angle);
    CHECK(r.dim.value == Approx(60.0));
    auto r2 = pick2(ents, SmartDimPick::whole(0), SmartDimPick::whole(1), Vec2d(-6, 2));
    CHECK(r2.dim.value == Approx(120.0));
}

TEST_CASE("parallel lines get the distance between them", "[SketchDimension]")
{
    std::vector<SketchEntity> ents = { line({0, 0}, {10, 2}), line({1, 5.2}, {-4, 4.2}) };   // antiparallel
    auto r = pick2(ents, SmartDimPick::whole(0), SmartDimPick::whole(1), Vec2d(3, 3));
    REQUIRE(r.ok);
    CHECK(r.dim.kind == K::LineLine);
    const double expect = std::abs((Vec2d(1, 5.2) - Vec2d(0, 0)).dot(Vec2d(-2, 10).normalized()));
    CHECK(r.dim.value == Approx(expect));
    // A line with an axis reference parallel to it.
    std::vector<SketchEntity> e2 = { line({-3, 4}, {6, 4}) };
    auto ax = pick2(e2, SmartDimPick::whole(0), SmartDimPick::whole(kSketchRefAxisX), Vec2d(0, 2));
    REQUIRE(ax.ok);
    CHECK(ax.dim.kind == K::LineLine);
    CHECK(ax.dim.value == Approx(4.0));
}

TEST_CASE("point pairs follow the same cursor rule as a line", "[SketchDimension]")
{
    std::vector<SketchEntity> ents = { point({0, 0}), point({10, 4}) };
    const SmartDimPick P = SmartDimPick::whole(0), Q = SmartDimPick::whole(1);
    auto aligned = pick2(ents, P, Q, Vec2d(5, 2) + 3.0 * Vec2d(-4, 10).normalized());
    REQUIRE(aligned.ok);
    CHECK(aligned.dim.kind == K::PointPoint);
    CHECK(aligned.dim.value == Approx(std::sqrt(116.0)));
    auto h = pick2(ents, P, Q, Vec2d(20, 25));
    REQUIRE(h.ok);
    CHECK(h.dim.kind == K::Horizontal);
    CHECK(h.dim.value == Approx(10.0));
    auto v = pick2(ents, P, Q, Vec2d(40, 3));
    REQUIRE(v.ok);
    CHECK(v.dim.kind == K::Vertical);
    CHECK(v.dim.value == Approx(4.0));
    // Endpoints of lines count as points; the origin too.
    std::vector<SketchEntity> e2 = { line({3, 4}, {9, 9}) };
    auto o = pick2(e2, SmartDimPick::whole(kSketchRefOrigin), SmartDimPick::at(0, R::P0), Vec2d(1.5, 2) + Vec2d(-4, 3));
    REQUIRE(o.ok);
    CHECK(o.dim.kind == K::PointPoint);
    CHECK(o.dim.value == Approx(5.0));
    // Two coincident points cannot be dimensioned.
    std::vector<SketchEntity> e3 = { point({1, 1}), point({1, 1}) };
    CHECK_FALSE(pick2(e3, SmartDimPick::whole(0), SmartDimPick::whole(1), Vec2d(4, 4)).ok);
}

TEST_CASE("a point and a line get the perpendicular distance, point first", "[SketchDimension]")
{
    std::vector<SketchEntity> ents = { line({0, 0}, {10, 0}), point({4, 7}) };
    for (bool line_first : {true, false}) {
        auto r = line_first ? pick2(ents, SmartDimPick::whole(0), SmartDimPick::whole(1), Vec2d(8, 3))
                            : pick2(ents, SmartDimPick::whole(1), SmartDimPick::whole(0), Vec2d(8, 3));
        REQUIRE(r.ok);
        CHECK(r.dim.kind == K::PointLine);
        CHECK(r.dim.ea == 1);
        CHECK(r.dim.eb == 0);
        CHECK(r.dim.value == Approx(7.0));
    }
}

TEST_CASE("circles and arcs are dimensioned from their centres", "[SketchDimension]")
{
    std::vector<SketchEntity> ents = { circle({0, 0}, 3), point({8, 6}), line({0, 10}, {10, 10}),
                                       arc({20, 0}, 2, 0.0, M_PI) };
    auto pc = pick2(ents, SmartDimPick::whole(1), SmartDimPick::whole(0), Vec2d(4, 3) + Vec2d(-3, 4));
    REQUIRE(pc.ok);
    CHECK(pc.dim.kind == K::PointPoint);
    CHECK(pc.dim.value == Approx(10.0));
    CHECK((pc.dim.ea == 0 ? pc.dim.ra : pc.dim.rb) == R::Center);
    auto lc = pick2(ents, SmartDimPick::whole(2), SmartDimPick::whole(0), Vec2d(-5, 5));
    REQUIRE(lc.ok);
    CHECK(lc.dim.kind == K::PointLine);
    CHECK(lc.dim.ea == 0);
    CHECK(lc.dim.ra == R::Center);
    CHECK(lc.dim.value == Approx(10.0));
    auto cc = pick2(ents, SmartDimPick::whole(0), SmartDimPick::whole(3), Vec2d(10, 30));
    REQUIRE(cc.ok);
    CHECK(cc.dim.kind == K::Horizontal);   // centres (0,0)/(20,0) are level
    CHECK(cc.dim.value == Approx(20.0));
}

// ---- driving the solver ----------------------------------------------------------------------

TEST_CASE("every dimension kind drives the solver to the typed value", "[SketchDimension]")
{
    SECTION("length") {
        std::vector<SketchEntity> ents = { line({0, 0}, {6, 8}) };
        auto r = pick1(ents, SmartDimPick::whole(0), Vec2d(3, 4));
        REQUIRE(r.dim.kind == K::Length);
        CHECK(drive(ents, r.dim, 20.0, {con(CT::Fix, 0, R::P0)}) == Approx(20.0).margin(1e-6));
    }
    SECTION("horizontal and vertical") {
        std::vector<SketchEntity> ents = { line({0, 0}, {6, 8}) };
        auto h = pick1(ents, SmartDimPick::whole(0), Vec2d(3, 40));
        REQUIRE(h.dim.kind == K::Horizontal);
        CHECK(drive(ents, h.dim, 9.0, {con(CT::Fix, 0, R::P0)}) == Approx(9.0).margin(1e-6));
        auto v = pick1(ents, SmartDimPick::whole(0), Vec2d(40, 3));
        REQUIRE(v.dim.kind == K::Vertical);
        CHECK(drive(ents, v.dim, 2.5, {con(CT::Fix, 0, R::P0)}) == Approx(2.5).margin(1e-6));
    }
    SECTION("diameter and radius") {
        std::vector<SketchEntity> ents = { circle({0, 0}, 5), arc({20, 0}, 4, 0.0, M_PI / 2) };
        auto d = pick1(ents, SmartDimPick::whole(0), Vec2d(8, 8));
        CHECK(drive(ents, d.dim, 17.0) == Approx(17.0).margin(1e-6));
        auto r = pick1(ents, SmartDimPick::whole(1), Vec2d(30, 8));
        CHECK(drive(ents, r.dim, 6.0) == Approx(6.0).margin(1e-6));
    }
    SECTION("point-point aligned") {
        std::vector<SketchEntity> ents = { point({0, 0}), point({3, 4}) };
        auto r = pick2(ents, SmartDimPick::whole(0), SmartDimPick::whole(1), Vec2d(1.5, 2));
        REQUIRE(r.dim.kind == K::PointPoint);
        CHECK(drive(ents, r.dim, 12.0, {con(CT::Fix, 0, R::P0)}) == Approx(12.0).margin(1e-6));
    }
    SECTION("point-line keeps the point on its side") {
        std::vector<SketchEntity> ents = { line({0, 0}, {10, 0}), point({4, 9}) };
        auto r = pick2(ents, SmartDimPick::whole(1), SmartDimPick::whole(0), Vec2d(6, 4));
        REQUIRE(r.dim.kind == K::PointLine);
        CHECK(drive(ents, r.dim, 2.0, {con(CT::Fix, 0, R::P0), con(CT::Fix, 0, R::P1)}) == Approx(2.0).margin(1e-6));
        CHECK(ents[1].p0.y() == Approx(2.0).margin(1e-6));   // above, where it was — not flipped to -2
        // And below the line stays below.
        std::vector<SketchEntity> below = { line({0, 0}, {10, 0}), point({4, -9}) };
        auto rb = pick2(below, SmartDimPick::whole(1), SmartDimPick::whole(0), Vec2d(6, -4));
        CHECK(drive(below, rb.dim, 3.0, {con(CT::Fix, 0, R::P0), con(CT::Fix, 0, R::P1)}) == Approx(3.0).margin(1e-6));
        CHECK(below[1].p0.y() == Approx(-3.0).margin(1e-6));
    }
    SECTION("parallel line-line distance") {
        std::vector<SketchEntity> ents = { line({0, 0}, {10, 0}), line({0, 5}, {10, 5}) };
        auto r = pick2(ents, SmartDimPick::whole(0), SmartDimPick::whole(1), Vec2d(5, 2));
        REQUIRE(r.dim.kind == K::LineLine);
        std::vector<SketchEntityConstraintDef> hold = { con(CT::Fix, 0, R::P0), con(CT::Fix, 0, R::P1),
                                                        con(CT::Parallel, 0, R::P0, 1, R::P0) };
        CHECK(drive(ents, r.dim, 8.0, hold) == Approx(8.0).margin(1e-6));
        CHECK(ents[1].p0.y() == Approx(8.0).margin(1e-6));
    }
}

// ---- angle units -----------------------------------------------------------------------------

TEST_CASE("a 30 degree angle dimension reaches the solver as 30 degrees", "[SketchDimension]")
{
    // L0 fixed along +X, L1 hinged at the shared vertex (the origin end of L0).
    auto sketch = [] { return std::vector<SketchEntity>{ line({0, 0}, {10, 0}), line({0, 0}, {6, 8}) }; };
    const std::vector<SketchEntityConstraintDef> hold = {
        con(CT::Fix, 0, R::P0), con(CT::Fix, 0, R::P1), con(CT::Coincident, 0, R::P0, 1, R::P0) };

    SECTION("sketch-tool path: the smart dimension's constraint") {
        for (Vec2d cursor : {Vec2d(4, 1), Vec2d(-4, 1)}) {   // interior sector, and its supplement
            std::vector<SketchEntity> ents = sketch();
            auto r = pick2(ents, SmartDimPick::whole(0), SmartDimPick::whole(1), cursor);
            REQUIRE(r.dim.kind == K::Angle);
            CHECK(drive(ents, r.dim, 30.0, hold) == Approx(30.0).margin(1e-6));
            // The stored value is radians, never degrees.
            auto c = sketch_dimension_constraint(ents, r.dim, 30.0);
            CHECK(c->value < M_PI);
        }
        // Interior 30 -> L1 at 30 degrees from +X.
        std::vector<SketchEntity> ents = sketch();
        auto r = pick2(ents, SmartDimPick::whole(0), SmartDimPick::whole(1), Vec2d(4, 1));
        drive(ents, r.dim, 30.0, hold);
        CHECK(angle_deg(ents[1].p1 - ents[1].p0, Vec2d(1, 0)) == Approx(30.0).margin(1e-6));
    }
    SECTION("panel path: plan_entity_constraint prefill (degrees) converted like write_plan_value") {
        std::vector<SketchEntity> ents = sketch();
        ConstraintPlan plan = plan_entity_constraint(ents, 0, 1, -1, CT::Angle);
        REQUIRE(plan.kind == ConstraintPlan::Kind::AskValue);
        CHECK(plan.prefill == Approx(angle_deg(Vec2d(6, 8), Vec2d(1, 0))));   // degrees
        std::vector<SketchEntityConstraintDef> cons = hold;
        for (auto d : plan.defs) {
            d.value = 30.0 * M_PI / 180.0;    // DesignPanel write_plan_value
            cons.push_back(d);
        }
        REQUIRE(sketch_solve(ents, cons).ok);
        CHECK(angle_deg(ents[1].p1 - ents[1].p0, Vec2d(1, 0)) == Approx(30.0).margin(1e-6));
        // Same unit as the smart-dimension path for the same (unreversed) sector.
        CHECK(sketch_angle_constraint_value(30.0, 0) == Approx(30.0 * M_PI / 180.0));
    }
    SECTION("a degree value stored by mistake would NOT give 30 degrees") {
        std::vector<SketchEntity> ents = sketch();
        std::vector<SketchEntityConstraintDef> cons = hold;
        cons.push_back(con(CT::Angle, 0, R::P0, 1, R::P0, 30.0));   // the old bug
        if (sketch_solve(ents, cons).ok)
            CHECK(angle_deg(ents[1].p1 - ents[1].p0, Vec2d(1, 0)) != Approx(30.0).margin(1e-3));
    }
}

// ---- over-definition -------------------------------------------------------------------------

TEST_CASE("a dimension on an already-determined quantity over-defines the sketch", "[SketchDimension]")
{
    std::vector<SketchEntity> ents = { line({0, 0}, {10, 0}) };
    std::vector<SketchEntityConstraintDef> cons = { con(CT::Fix, 0, R::P0), con(CT::Fix, 0, R::P1) };
    REQUIRE(sketch_solve(ents, cons).ok);
    auto r = pick1(ents, SmartDimPick::whole(0), Vec2d(5, 3));
    auto c = sketch_dimension_constraint(ents, r.dim, r.dim.value);   // even at its own value
    REQUIRE(c.has_value());
    cons.push_back(*c);
    std::vector<SketchEntity> tmp = ents;
    CHECK_FALSE(sketch_solve(tmp, cons).ok);   // the GUI makes it driven
}

// ---- defined / under-defined per entity ------------------------------------------------------

TEST_CASE("each entity reports whether the constraints fully define it", "[SketchDimension]")
{
    // A rectangle: fixed corner, H/V sides, coincident corners, width and height.
    std::vector<SketchEntity> ents = { line({0, 0}, {10, 0}), line({10, 0}, {10, 5}),
                                       line({10, 5}, {0, 5}),  line({0, 5}, {0, 0}),
                                       circle({30, 30}, 2) };
    std::vector<SketchEntityConstraintDef> cons = {
        con(CT::Coincident, 0, R::P1, 1, R::P0), con(CT::Coincident, 1, R::P1, 2, R::P0),
        con(CT::Coincident, 2, R::P1, 3, R::P0), con(CT::Coincident, 3, R::P1, 0, R::P0),
        con(CT::Horizontal, 0, R::P0, 0, R::P1), con(CT::Horizontal, 2, R::P0, 2, R::P1),
        con(CT::Vertical,   1, R::P0, 1, R::P1), con(CT::Vertical,   3, R::P0, 3, R::P1),
        con(CT::Fix, 0, R::P0),
        con(CT::Distance, 0, R::P0, 0, R::P1, 10.0),
    };
    std::vector<char> def;
    REQUIRE(sketch_entities_defined(ents, cons, def));
    REQUIRE(def.size() == ents.size());
    // Height still free: the vertical sides and the top can move; the bottom cannot.
    CHECK(def[0] == 1);
    CHECK(def[1] == 0);
    CHECK(def[2] == 0);
    CHECK(def[3] == 0);
    CHECK(def[4] == 0);   // untouched circle
    cons.push_back(con(CT::Distance, 1, R::P0, 1, R::P1, 5.0));
    REQUIRE(sketch_entities_defined(ents, cons, def));
    CHECK(def[0] == 1);
    CHECK(def[1] == 1);
    CHECK(def[2] == 1);
    CHECK(def[3] == 1);
    CHECK(def[4] == 0);
    // Circle: centre fixed but radius free is NOT defined; with a diameter it is.
    cons.push_back(con(CT::Fix, 4, R::Center));
    REQUIRE(sketch_entities_defined(ents, cons, def));
    CHECK(def[4] == 0);
    cons.push_back(con(CT::Diameter, 4, R::P0, -1, R::P0, 4.0));
    REQUIRE(sketch_entities_defined(ents, cons, def));
    CHECK(def[4] == 1);
    // Over-defined: no answer.
    cons.push_back(con(CT::Distance, 2, R::P0, 2, R::P1, 10.0));
    CHECK_FALSE(sketch_entities_defined(ents, cons, def));
}

// ---- layout ----------------------------------------------------------------------------------

TEST_CASE("a linear dimension draws its arrows inside a wide span", "[SketchDimension]")
{
    std::vector<SketchEntity> ents = { line({0, 0}, {10, 0}) };
    auto r = pick1(ents, SmartDimPick::whole(0), Vec2d(5, 5));
    SketchDimStyle st;   // arrow 2, gap 0.6, overshoot 1
    SketchDimLayout L = layout_sketch_dimension(ents, r.dim, st);
    REQUIRE(L.ok);
    CHECK(L.arrows_inside);
    REQUIRE(L.ext_lines.size() == 2);
    CHECK(L.ext_lines[0].first.isApprox(Vec2d(0, 0.6)));
    CHECK(L.ext_lines[0].second.isApprox(Vec2d(0, 6.0)));
    CHECK(L.ext_lines[1].first.isApprox(Vec2d(10, 0.6)));
    CHECK(L.ext_lines[1].second.isApprox(Vec2d(10, 6.0)));
    REQUIRE(L.dim_lines.size() == 1);
    CHECK(L.dim_lines[0].first.isApprox(Vec2d(0, 5)));
    CHECK(L.dim_lines[0].second.isApprox(Vec2d(10, 5)));
    REQUIRE(L.arrows.size() == 2);
    CHECK(L.arrows[0][0].isApprox(Vec2d(0, 5)));                       // tip on the extension line
    CHECK(L.arrows[0][1].x() == Approx(2.0));                          // body inside the span
    CHECK(L.arrows[1][0].isApprox(Vec2d(10, 5)));
    CHECK(L.arrows[1][1].x() == Approx(8.0));
    CHECK(L.text.isApprox(Vec2d(5, 5)));
}

TEST_CASE("a small span puts the arrows and the text outside", "[SketchDimension]")
{
    std::vector<SketchEntity> ents = { line({0, 0}, {3, 0}) };
    auto r = pick1(ents, SmartDimPick::whole(0), Vec2d(2, -4));
    SketchDimStyle st;
    SketchDimLayout L = layout_sketch_dimension(ents, r.dim, st);
    REQUIRE(L.ok);
    CHECK_FALSE(L.arrows_inside);
    REQUIRE(L.arrows.size() == 2);
    CHECK(L.arrows[0][1].x() == Approx(-2.0));   // pointing in from outside
    CHECK(L.arrows[1][1].x() == Approx(5.0));
    CHECK(L.text.y() == Approx(-4.0));
    CHECK(L.text.x() > 3.0 + 2.0 * st.arrow_len);   // pushed past the right arrow
}

TEST_CASE("text beyond the extension lines extends the dimension line", "[SketchDimension]")
{
    std::vector<SketchEntity> ents = { point({0, 0}), point({10, 4}) };
    auto r = pick2(ents, SmartDimPick::whole(0), SmartDimPick::whole(1), Vec2d(12, 25));
    REQUIRE(r.dim.kind == K::Horizontal);
    SketchDimLayout L = layout_sketch_dimension(ents, r.dim, SketchDimStyle{});
    REQUIRE(L.dim_lines.size() == 1);
    CHECK(L.arrows_inside);
    CHECK(L.dim_lines[0].first.isApprox(Vec2d(0, 25)));
    CHECK(L.dim_lines[0].second.isApprox(Vec2d(12, 25)));
    CHECK(L.text.isApprox(Vec2d(12, 25)));
}

TEST_CASE("diameter, radius and angle dimensions lay out around their geometry", "[SketchDimension]")
{
    SketchDimStyle st;
    SECTION("diameter through the centre to the text") {
        std::vector<SketchEntity> ents = { circle({0, 0}, 5) };
        auto r = pick1(ents, SmartDimPick::whole(0), Vec2d(10, 0));
        SketchDimLayout L = layout_sketch_dimension(ents, r.dim, st);
        REQUIRE(L.ok);
        REQUIRE(L.dim_lines.size() == 1);
        CHECK(L.dim_lines[0].first.isApprox(Vec2d(-5, 0)));
        CHECK(L.dim_lines[0].second.isApprox(Vec2d(10, 0)));
        REQUIRE(L.arrows.size() == 2);
        CHECK(L.arrows[0][0].isApprox(Vec2d(-5, 0)));
        CHECK(L.arrows[1][0].isApprox(Vec2d(5, 0)));
    }
    SECTION("radius leader with one arrow on the arc, arc continued outside its sweep") {
        std::vector<SketchEntity> ents = { arc({0, 0}, 4, 0.0, M_PI / 2) };
        auto r = pick1(ents, SmartDimPick::whole(0), Vec2d(0, -8));   // below: outside the sweep
        SketchDimLayout L = layout_sketch_dimension(ents, r.dim, st);
        REQUIRE(L.ok);
        REQUIRE(L.arrows.size() == 1);
        CHECK(L.arrows[0][0].isApprox(Vec2d(0, -4)));
        CHECK_FALSE(L.ext_lines.empty());            // extension arc from (4,0) round to (0,-4)
        CHECK(L.ext_lines.back().second.isApprox(Vec2d(0, -4), 1e-6));
    }
    SECTION("angle arc through the text with its arrows on the arms") {
        std::vector<SketchEntity> ents = { line({0, 0}, {10, 0}), line({0, 0}, {10, 10}) };
        auto r = pick2(ents, SmartDimPick::whole(0), SmartDimPick::whole(1), Vec2d(9, 3));
        SketchDimLayout L = layout_sketch_dimension(ents, r.dim, st);
        REQUIRE(L.ok);
        CHECK(L.arrows_inside);
        const double rho = Vec2d(9, 3).norm();
        REQUIRE(L.arrows.size() == 2);
        CHECK(L.arrows[0][0].isApprox(Vec2d(rho, 0)));
        CHECK(L.arrows[1][0].isApprox(Vec2d(rho, rho) / std::sqrt(2.0)));
        for (const auto& s : L.dim_lines) {
            CHECK(s.first.norm() == Approx(rho));
            CHECK(s.second.norm() == Approx(rho));
        }
        CHECK(L.ext_lines.empty());   // both arms are longer than the arc radius
    }
}

// ---- text ------------------------------------------------------------------------------------

TEST_CASE("dimension text trims zeros and carries its prefix, suffix and parentheses", "[SketchDimension]")
{
    SketchDimension d;
    d.kind = K::Length;
    CHECK(format_sketch_dimension(d, 12.5) == "12.5");
    CHECK(format_sketch_dimension(d, 12.0) == "12");
    CHECK(format_sketch_dimension(d, 3.14159) == "3.14");
    CHECK(format_sketch_dimension(d, 0.0) == "0");
    d.kind = K::Diameter;
    CHECK(format_sketch_dimension(d, 12.0) == "\xC3\x98" "12");
    d.kind = K::Radius;
    CHECK(format_sketch_dimension(d, 5.0) == "R5");
    d.kind = K::Angle;
    CHECK(format_sketch_dimension(d, 30.0) == "30\xC2\xB0");
    d.kind = K::Length;
    d.driven = true;
    CHECK(format_sketch_dimension(d, 10.25) == "(10.25)");
}

// ---- re-indexing -----------------------------------------------------------------------------

TEST_CASE("dimension references follow entity and constraint re-indexing", "[SketchDimension]")
{
    std::vector<SketchDimension> dims(4);
    dims[0].kind = K::Length;     dims[0].ea = 0; dims[0].eb = 0; dims[0].constraint = 0;
    dims[1].kind = K::PointPoint; dims[1].ea = 1; dims[1].eb = 3; dims[1].constraint = 1;
    dims[2].kind = K::PointLine;  dims[2].ea = kSketchRefOrigin; dims[2].eb = 3; dims[2].constraint = 2;
    dims[3].kind = K::Diameter;   dims[3].ea = 2; dims[3].eb = -1; dims[3].constraint = 3;

    // Entity 1 deleted: 0->0, 2->1, 3->2. dims[1] referenced it and goes.
    sketch_dimensions_remap_entities(dims, {0, -1, 1, 2});
    REQUIRE(dims.size() == 3);
    CHECK(dims[0].ea == 0);
    CHECK(dims[1].ea == kSketchRefOrigin);
    CHECK(dims[1].eb == 2);
    CHECK(dims[2].ea == 1);
    CHECK(dims[2].eb == -1);

    // Constraint 1 was dropped with the entity: 0->0, 1->-1, 2->1, 3->2.
    sketch_dimensions_remap_constraints(dims, {0, -1, 1, 2});
    CHECK(dims[0].constraint == 0);
    CHECK(dims[1].constraint == 1);
    CHECK(dims[2].constraint == 2);
    CHECK_FALSE(dims[2].driven);

    // Erasing constraint 1 makes its dimension driven and shifts the later one.
    sketch_dimensions_erase_constraint(dims, 1);
    CHECK(dims[1].driven);
    CHECK(dims[1].constraint == -1);
    CHECK(dims[2].constraint == 1);

    // Sanitize against a smaller sketch: the origin-to-entity-2 distance loses its entity, the
    // diameter keeps its entity but loses its constraint.
    sketch_dimensions_sanitize(dims, 2, 1);
    REQUIRE(dims.size() == 2);
    CHECK(dims[0].constraint == 0);
    CHECK(dims[1].kind == K::Diameter);
    CHECK(dims[1].driven);
    CHECK(dims[1].constraint == -1);
}

// ---- persistence -----------------------------------------------------------------------------

TEST_CASE("sketch dimensions survive a recipe round trip", "[SketchDimension][CadDocument]")
{
    CadDocument doc;
    std::vector<SketchEntity> ents = { line({0, 0}, {10, 0}), line({10, 0}, {10, 6}), circle({4, 3}, 1.5) };
    std::vector<SketchEntityConstraintDef> cons = {
        con(CT::Coincident, 0, R::P1, 1, R::P0),
        con(CT::Distance, 0, R::P0, 0, R::P1, 10.0),
    };
    std::vector<SketchDimension> dims(3);
    dims[0].kind = K::Horizontal; dims[0].ea = 0; dims[0].ra = R::P0; dims[0].eb = 0; dims[0].rb = R::P1;
    dims[0].constraint = 1; dims[0].value = 10.0; dims[0].text_pos = Vec2d(5, -4);
    dims[1].kind = K::Angle; dims[1].ea = 0; dims[1].eb = 1; dims[1].sector = 1; dims[1].driven = true;
    dims[1].value = 90.0; dims[1].text_pos = Vec2d(8, 2);
    dims[2].kind = K::Diameter; dims[2].ea = 2; dims[2].driven = true; dims[2].value = 3.0;
    dims[2].text_pos = Vec2d(7.25, 5.5);

    const int sk = doc.add_sketch_entities(ents, SketchPlane::XY(), "S", cons, dims);
    REQUIRE(doc.features[sk].dimensions.size() == 3);

    CadDocument back;
    back.deserialize_recipe(doc.serialize_recipe());
    REQUIRE(back.features.size() == 1);
    const auto& got = back.features[0].dimensions;
    REQUIRE(got.size() == 3);
    for (size_t i = 0; i < 3; ++i) {
        CHECK(got[i].kind == dims[i].kind);
        CHECK(got[i].ea == dims[i].ea);
        CHECK(got[i].eb == dims[i].eb);
        CHECK(got[i].ra == dims[i].ra);
        CHECK(got[i].rb == dims[i].rb);
        CHECK(got[i].text_pos.isApprox(dims[i].text_pos));
        CHECK(got[i].driven == dims[i].driven);
        CHECK(got[i].constraint == dims[i].constraint);
        CHECK(got[i].value == Approx(dims[i].value));
        CHECK(got[i].sector == dims[i].sector);
    }

    // replace_feature carries them too.
    CadFeature edited = back.features[0];
    edited.dimensions.pop_back();
    REQUIRE(back.replace_feature(0, edited));
    CHECK(back.features[0].dimensions.size() == 2);
}

TEST_CASE("a recipe written before sketch dimensions still loads", "[SketchDimension][CadDocument]")
{
    SECTION("a feature frame without the trailing field") {
        CadDocument doc;
        std::vector<SketchEntity> ents = { line({0, 0}, {10, 0}) };
        doc.add_sketch_entities(ents, SketchPlane::XY(), "Old", { con(CT::Fix, 0, R::P0) });
        doc.features[0].coordsys_face_edges = 4242;   // the field right before `dimensions`
        std::string blob = doc.serialize_recipe();
        // Frame layout: [u32 version][u32 count][u32 len][feature bytes]... An empty `dimensions`
        // is cereal's 8-byte size at the very end of the feature: cut it, as an older build wrote.
        uint32_t len;
        std::memcpy(&len, blob.data() + 8, 4);
        const uint32_t cut = sizeof(uint64_t);
        blob.erase(12 + len - cut, cut);
        len -= cut;
        std::memcpy(&blob[8], &len, 4);
        CadDocument old;
        old.deserialize_recipe(blob);
        REQUIRE(old.features.size() == 1);
        CHECK(old.features[0].name == "Old");
        CHECK(old.features[0].entities.size() == 1);
        CHECK(old.features[0].entity_constraints.size() == 1);
        CHECK(old.features[0].coordsys_face_edges == 4242);
        CHECK(old.features[0].dimensions.empty());
    }
    SECTION("the unframed v4 fixture") {
        // v4 has no per-feature framing: reading a field v4 never wrote would consume the next
        // feature's bytes. The field list stops where v4's did.
        std::ifstream ifs(std::string(TEST_DATA_DIR) + "/cad_recipe_v4.bin", std::ios::binary);
        REQUIRE(ifs.is_open());
        std::string blob((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
        CadDocument doc;
        doc.deserialize_recipe(blob);
        CHECK(doc.error.find("version") == std::string::npos);
        REQUIRE_FALSE(doc.features.empty());
        for (const CadFeature& f : doc.features)
            CHECK(f.dimensions.empty());
    }
    SECTION("the v5 fixture") {
        std::ifstream ifs(std::string(TEST_DATA_DIR) + "/cad_recipe_v5.bin", std::ios::binary);
        REQUIRE(ifs.is_open());
        std::string blob((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
        CadDocument doc;
        doc.deserialize_recipe(blob);
        CHECK(doc.error.find("version") == std::string::npos);
        REQUIRE_FALSE(doc.features.empty());
        for (const CadFeature& f : doc.features)
            CHECK(f.dimensions.empty());
    }
}


TEST_CASE("a circle picked by its edge dimensions the minimum distance", "[SketchDimension]")
{
    SmartDimPick edge = SmartDimPick::whole(0);
    edge.edge = true;
    SECTION("edge to a point, driven through the centre distance") {
        std::vector<SketchEntity> ents = { circle({0, 0}, 3), point({10, 0.5}) };
        auto r = pick2(ents, edge, SmartDimPick::whole(1), Vec2d(40, 30));   // outside the band: still aligned
        REQUIRE(r.ok);
        CHECK(r.dim.kind == K::PointPoint);
        CHECK(r.dim.sector == 1);
        CHECK(r.dim.value == Approx(Vec2d(10, 0.5).norm() - 3.0));
        auto c = sketch_dimension_constraint(ents, r.dim, 5.0);
        REQUIRE(c.has_value());
        CHECK(c->value == Approx(8.0));   // centre distance = gap + radius
        CHECK(drive(ents, r.dim, 5.0, {con(CT::Fix, 0, R::Center), con(CT::Diameter, 0, R::P0, -1, R::P0, 6.0)})
              == Approx(5.0).margin(1e-6));
        SketchDimLayout L = layout_sketch_dimension(ents, r.dim, SketchDimStyle{});
        REQUIRE(L.ok);
        // The extension line leaves the circle at its point facing the other reference.
        const Vec2d tangent = ents[1].p0.normalized() * 3.0;
        bool from_edge = false;
        for (const auto& e : L.ext_lines)
            from_edge = from_edge || (e.first - tangent).norm() <= SketchDimStyle{}.ext_gap + 1e-9;
        CHECK(from_edge);
    }
    SECTION("edge to a line") {
        std::vector<SketchEntity> ents = { circle({0, 0}, 3), line({-10, 10}, {10, 10}) };
        auto r = pick2(ents, edge, SmartDimPick::whole(1), Vec2d(5, 5));
        REQUIRE(r.ok);
        CHECK(r.dim.kind == K::PointLine);
        CHECK(r.dim.value == Approx(7.0));
        CHECK(drive(ents, r.dim, 4.0, {con(CT::Fix, 1, R::P0), con(CT::Fix, 1, R::P1),
                                       con(CT::Diameter, 0, R::P0, -1, R::P0, 6.0)}) == Approx(4.0).margin(1e-6));
        CHECK(ents[0].center.y() == Approx(3.0).margin(1e-6));
    }
    SECTION("edge to edge") {
        std::vector<SketchEntity> ents = { circle({0, 0}, 3), circle({20, 0}, 2) };
        SmartDimPick edge2 = SmartDimPick::whole(1);
        edge2.edge = true;
        auto r = pick2(ents, edge, edge2, Vec2d(10, 5));
        REQUIRE(r.ok);
        CHECK(r.dim.sector == 3);
        CHECK(r.dim.value == Approx(15.0));
    }
}

TEST_CASE("the dimensions block frames every item so later fields can be appended", "[SketchDimension]")
{
    std::vector<SketchDimension> dims(2);
    dims[0].kind = K::Angle; dims[0].ea = 3; dims[0].eb = kSketchRefAxisX; dims[0].sector = 2;
    dims[0].text_pos = Vec2d(1.5, -2.5); dims[0].constraint = 4; dims[0].value = 30.0;
    dims[1].kind = K::PointLine; dims[1].ea = 1; dims[1].ra = R::Center; dims[1].eb = 0;
    dims[1].driven = true; dims[1].value = 7.25; dims[1].sector = 1;

    auto same = [](const SketchDimension& a, const SketchDimension& b, bool with_sector) {
        CHECK(a.kind == b.kind);
        CHECK(a.ea == b.ea);
        CHECK(a.eb == b.eb);
        CHECK(a.ra == b.ra);
        CHECK(a.rb == b.rb);
        CHECK(a.text_pos.isApprox(b.text_pos));
        CHECK(a.driven == b.driven);
        CHECK(a.constraint == b.constraint);
        CHECK(a.value == Approx(b.value));
        if (with_sector) CHECK(a.sector == b.sector);
    };
    auto payload = [](const SketchDimension& d) {
        std::ostringstream os;
        { cereal::BinaryOutputArchive ar(os); SketchDimension c = d; ar(c); }
        return os.str();
    };
    // A block written by hand: u32 version, u32 count, then [u32 len][item bytes] per item.
    auto block_of = [](uint32_t version, const std::vector<std::string>& items) {
        std::ostringstream os;
        {
            cereal::BinaryOutputArchive ar(os);
            const uint32_t count = uint32_t(items.size());
            ar(version, count);
            for (const std::string& it : items) {
                const uint32_t len = uint32_t(it.size());
                ar(len);
                ar(cereal::binary_data(it.data(), it.size()));
            }
        }
        return os.str();
    };

    SECTION("round trip") {
        std::vector<SketchDimension> back;
        REQUIRE(sketch_dimensions_decode(sketch_dimensions_encode(dims), back));
        REQUIRE(back.size() == 2);
        same(back[0], dims[0], true);
        same(back[1], dims[1], true);
        CHECK(sketch_dimensions_encode({}).empty());
        REQUIRE(sketch_dimensions_decode(std::string(), back));
        CHECK(back.empty());
    }
    SECTION("a newer build's extra trailing bytes per item are skipped") {
        const std::string junk("\x01\x02\x03\x04\x05\x06\x07", 7);
        std::vector<SketchDimension> back;
        REQUIRE(sketch_dimensions_decode(block_of(2, {payload(dims[0]) + junk, payload(dims[1]) + junk}), back));
        REQUIRE(back.size() == 2);
        same(back[0], dims[0], true);
        same(back[1], dims[1], true);   // the second item starts exactly after the first's frame
    }
    SECTION("an older build's shorter item keeps what it has and defaults the rest") {
        std::string p0 = payload(dims[0]);
        p0.resize(p0.size() - sizeof(int32_t));     // no `sector`
        std::vector<SketchDimension> back;
        REQUIRE(sketch_dimensions_decode(block_of(1, {p0, payload(dims[1])}), back));
        REQUIRE(back.size() == 2);
        same(back[0], dims[0], false);
        CHECK(back[0].sector == 0);
        same(back[1], dims[1], true);
    }
    SECTION("a truncated block is refused without throwing") {
        std::string b = sketch_dimensions_encode(dims);
        b.resize(b.size() / 2);
        std::vector<SketchDimension> back;
        CHECK_FALSE(sketch_dimensions_decode(b, back));
    }
}
