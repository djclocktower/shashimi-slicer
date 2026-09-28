#include <catch2/catch_all.hpp>

#include "libslic3r/CAM/CamInternal.hpp"
#include "libslic3r/CAM/Op3D.hpp"
#include "libslic3r/CAM/StockSim.hpp"

#include <chrono>
#include <cmath>
#include <map>
#include <set>

using namespace Slic3r;
using namespace Slic3r::CAM;

namespace {

CamTool make_tool(ToolType type, double d, double corner = 0)
{
    CamTool t;
    t.number        = 1;
    t.type          = type;
    t.diameter      = d;
    t.corner_radius = corner;
    return t;
}

TriangleMesh box(double x0, double y0, double z0, double x1, double y1, double z1)
{
    TriangleMesh m = make_cube(x1 - x0, y1 - y0, z1 - z0);
    m.translate(float(x0), float(y0), float(z0));
    return m;
}

TriangleMesh merged(std::initializer_list<TriangleMesh> parts)
{
    indexed_triangle_set its;
    for (const TriangleMesh& p : parts)
        its_merge(its, p.its);
    return TriangleMesh(std::move(its));
}

// Upper half of a sphere of radius r centred at the origin (open at z = 0).
TriangleMesh hemisphere(double r)
{
    indexed_triangle_set s = its_make_sphere(r, 2 * M_PI / 120);
    indexed_triangle_set out;
    out.vertices = s.vertices;
    for (const Vec3i32& f : s.indices)
        if (std::min({s.vertices[f(0)].z(), s.vertices[f(1)].z(), s.vertices[f(2)].z()}) > -1e-4f)
            out.indices.push_back(f);
    its_compactify_vertices(out);
    return TriangleMesh(std::move(out));
}

struct Job {
    CamDocument doc;
    CamModel    model;
    CamOperation& op() { return doc.operations.front(); }
};

Job make_job(const TriangleMesh& mesh, OpType type, const CamTool& tool, const Vec3d& neg = {1, 1, 0}, const Vec3d& pos = {1, 1, 1})
{
    Job j;
    j.doc.tools  = {tool};
    j.doc.setups = {CamSetup{}};
    CamOperation op;
    op.type        = type;
    op.tool_number = tool.number;
    j.doc.operations = {op};
    CamBody b;
    b.body_id = 0;
    b.mesh    = mesh;
    j.model.bodies.push_back(b);
    CamSetupFrame f;
    f.model = mesh.bounding_box();
    f.stock = BoundingBoxf3(f.model.min - neg, f.model.max + pos);
    j.model.setups = {f};
    return j;
}

double seconds_since(std::chrono::steady_clock::time_point t0)
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}

// Tip positions along every cutting move, `step` apart (lines only; arcs by their end points).
template<class Fn> void for_each_cut_point(const Toolpath& tp, double step, Fn fn)
{
    for (size_t k = 1; k < tp.moves.size(); ++k) {
        const Move& m = tp.moves[k];
        if (m.kind == Move::Kind::Rapid || m.kind == Move::Kind::Retract)
            continue;
        const Vec3d a = tp.moves[k - 1].to, b = m.to;
        const int   n = is_arc(m) ? 1 : std::max(1, int(std::ceil((b - a).norm() / step)));
        for (int i = 0; i <= n; ++i)
            fn(Vec3d(a + (b - a) * (double(i) / n)), k);
    }
}

} // namespace

TEST_CASE("Drop-cutter with a flat end mill stays on a box top up to one tool radius past its edge", "[Cam3D]")
{
    const TriangleMesh  m = box(0, 0, 0, 20, 20, 10);
    const CamTool       t = make_tool(ToolType::FlatEndMill, 6);
    const BoundingBoxf3 bb(Vec3d(-6, -6, 0), Vec3d(26, 26, 10));
    const HeightMap     hm = drop_cutter(m, t, bb, 0.25);
    REQUIRE(hm.nx == 129);
    for (int j = 0; j < hm.ny; ++j)
        for (int i = 0; i < hm.nx; ++i) {
            const double x = hm.x0 + i * hm.resolution, y = hm.y0 + j * hm.resolution;
            const double dx = std::max(0., std::abs(x - 10) - 10), dy = std::max(0., std::abs(y - 10) - 10);
            const double d  = std::hypot(dx, dy); // distance to the top face's footprint
            INFO("x " << x << " y " << y);
            if (d <= 2.99)
                REQUIRE(hm.at(i, j) == Catch::Approx(10).margin(1e-5));
            else if (d >= 3.01)
                REQUIRE(hm.at(i, j) == Catch::Approx(0).margin(1e-5));
        }
}

TEST_CASE("Drop-cutter with a ball end mill over a sphere follows the offset sphere", "[Cam3D]")
{
    const TriangleMesh sphere = TriangleMesh(its_make_sphere(20, 2 * M_PI / 180));
    const CamTool      t      = make_tool(ToolType::BallEndMill, 6);
    const internal::DropCutter dc(sphere.its, internal::make_cutter(t));
    for (double rho = 0; rho <= 22.5; rho += 0.75)
        for (double ang : {0.1, 1.3, 2.9, 4.4}) {
            const double x = rho * std::cos(ang), y = rho * std::sin(ang);
            INFO("rho " << rho);
            REQUIRE(dc.at(x, y, -100) == Catch::Approx(std::sqrt(23. * 23. - rho * rho) - 3).margin(0.02));
        }
}

TEST_CASE("Drop-cutter matches the analytic contact on a tilted plane and a ridge", "[Cam3D]")
{
    // z = 0.5 x
    indexed_triangle_set plane;
    plane.vertices = {{-50, -50, -25}, {50, -50, 25}, {50, 50, 25}, {-50, 50, -25}};
    plane.indices  = {{0, 1, 2}, {0, 2, 3}};
    const double s = 0.5;
    for (double x : {-10., 0., 7.5}) {
        const double base = s * x;
        const auto   at   = [&](const CamTool& t) { return internal::DropCutter(plane, internal::make_cutter(t)).at(x, 3, -100); };
        // flat: the disc rim uphill
        REQUIRE(at(make_tool(ToolType::FlatEndMill, 6)) == Catch::Approx(base + s * 3).margin(1e-6));
        // ball: centre R from the plane
        REQUIRE(at(make_tool(ToolType::BallEndMill, 6)) == Catch::Approx(base + 3 * (std::sqrt(1 + s * s) - 1)).margin(1e-6));
        // bull R=3 rc=1: flat part (r=2) + corner ball
        REQUIRE(at(make_tool(ToolType::BullEndMill, 6, 1)) == Catch::Approx(base + s * 2 + (std::sqrt(1 + s * s) - 1)).margin(1e-6));
        // 90 deg V-bit, flank slope 1 > 0.5: the tip touches
        REQUIRE(at(make_tool(ToolType::VBit, 6)) == Catch::Approx(base).margin(1e-6));
    }

    // Ridge along Y at z = 10 with steep sides: a ball touches the ridge edge.
    indexed_triangle_set ridge;
    ridge.vertices = {{-5, -50, 0}, {-5, 50, 0}, {0, -50, 10}, {0, 50, 10}, {5, -50, 0}, {5, 50, 0}};
    ridge.indices  = {{0, 2, 3}, {0, 3, 1}, {2, 4, 5}, {2, 5, 3}};
    const internal::DropCutter ball(ridge, internal::make_cutter(make_tool(ToolType::BallEndMill, 6)));
    for (double dx : {0., 0.5, 1., 1.3})
        REQUIRE(ball.at(dx, 0, -100) == Catch::Approx(10 + std::sqrt(9 - dx * dx) - 3).margin(1e-6));
    const internal::DropCutter flat(ridge, internal::make_cutter(make_tool(ToolType::FlatEndMill, 6)));
    REQUIRE(flat.at(2.5, 0, -100) == Catch::Approx(10).margin(1e-6));
}

TEST_CASE("Parallel finishing of a hemisphere covers it without gouging", "[Cam3D]")
{
    const TriangleMesh hemi = hemisphere(50);
    Job                j    = make_job(hemi, OpType::Parallel3D, make_tool(ToolType::BallEndMill, 6));
    j.op().stepover         = 0.3;
    j.op().angle_deg        = 30;

    const auto     t0   = std::chrono::steady_clock::now();
    const Toolpath tp   = generate_parallel3d(j.doc, j.op(), j.model);
    const double   secs = seconds_since(t0);
    WARN("parallel3d hemisphere: " << secs << " s, " << tp.moves.size() << " moves");
    REQUIRE(tp.ok());
    REQUIRE(secs < 3.0);

    const internal::DropCutter dc(hemi.its, internal::make_cutter(make_tool(ToolType::BallEndMill, 6)));
    double worst = 0;
    // Coverage raster over the projected disc: cells within stepover/2 of a cutting move.
    const double      res = 0.25, x0 = -50;
    const int         n   = 401;
    std::vector<char> cov(size_t(n) * n, 0);
    const auto        mark = [&](const Vec3d& p) {
        const int ci = int(std::lround((p.x() - x0) / res)), cj = int(std::lround((p.y() - x0) / res));
        for (int dj = -1; dj <= 1; ++dj)
            for (int di = -1; di <= 1; ++di) {
                const int i = ci + di, jj = cj + dj;
                if (i >= 0 && jj >= 0 && i < n && jj < n && std::hypot(x0 + i * res - p.x(), x0 + jj * res - p.y()) <= 0.15 + 0.2)
                    cov[size_t(jj) * n + i] = 1;
            }
    };
    for_each_cut_point(tp, 0.2, [&](const Vec3d& p, size_t) {
        worst = std::max(worst, dc.at(p.x(), p.y(), 0) - p.z());
        mark(p);
    });
    INFO("worst gouge " << worst);
    REQUIRE(worst <= 0.01);
    size_t in = 0, covered = 0;
    for (int jj = 0; jj < n; ++jj)
        for (int i = 0; i < n; ++i)
            if (std::hypot(x0 + i * res, x0 + jj * res) <= 50) {
                ++in;
                covered += cov[size_t(jj) * n + i];
            }
    INFO("coverage " << double(covered) / in);
    REQUIRE(double(covered) / in >= 0.95);
    REQUIRE(gouge_check(tp, make_tool(ToolType::BallEndMill, 6), hemi).empty());
}

TEST_CASE("Z-level finishing of a stepped block cuts each level at the tool radius", "[Cam3D]")
{
    const TriangleMesh block = merged({box(0, 0, 0, 60, 60, 10), box(10, 10, 10, 50, 50, 20), box(20, 20, 20, 40, 40, 30)});
    Job                j     = make_job(block, OpType::Contour3D, make_tool(ToolType::FlatEndMill, 6), {1, 1, 0}, {1, 1, 0});
    j.op().stepdown          = 5;
    const Toolpath tp        = generate_contour3d(j.doc, j.op(), j.model);
    REQUIRE(tp.ok());

    std::map<double, std::pair<size_t, size_t>> levels; // z -> (feed points, points at offset R)
    double closest = 1e9;
    for (const Move& m : tp.moves) {
        if (m.kind != Move::Kind::Feed)
            continue;
        const double z    = std::round(m.to.z() * 1000) / 1000;
        const double half = z >= 20 ? 10 : z >= 10 ? 20 : 30;
        const double d    = std::hypot(std::max(0., std::abs(m.to.x() - 30) - half), std::max(0., std::abs(m.to.y() - 30) - half));
        closest           = std::min(closest, d);
        auto& l           = levels[z];
        ++l.first;
        l.second += std::abs(d - 3) < 0.02;
    }
    std::set<double> zs;
    for (auto& [z, c] : levels) {
        INFO("z " << z << " points " << c.first << " on offset " << c.second);
        REQUIRE(c.second >= 0.9 * c.first);
        zs.insert(z);
    }
    REQUIRE(zs == std::set<double>{0, 5, 10, 15, 20, 25});
    REQUIRE(closest >= 3 - 0.02);
    REQUIRE(gouge_check(tp, make_tool(ToolType::FlatEndMill, 6), block).empty());
}

TEST_CASE("Adaptive roughing of a box with a boss removes the stock down to the leave", "[Cam3D]")
{
    const TriangleMesh part = merged({box(0, 0, 0, 80, 80, 20), box(25, 25, 20, 55, 55, 30)});
    const CamTool      tool = make_tool(ToolType::FlatEndMill, 6);
    Job                j    = make_job(part, OpType::Adaptive3D, tool);
    j.op().stepdown              = 3;
    j.op().stock_to_leave_radial = 0.5;
    j.op().stock_to_leave_axial  = 0.3;

    auto           t0   = std::chrono::steady_clock::now();
    const Toolpath tp   = generate_adaptive3d(j.doc, j.op(), j.model);
    const double   secs = seconds_since(t0);
    WARN("adaptive3d box+boss: " << secs << " s, " << tp.moves.size() << " moves");
    for (const Warning& w : tp.warnings)
        WARN(w.text);
    REQUIRE(tp.ok());
    REQUIRE(secs < 10.0);

    StockSim sim;
    sim.init_box(j.model.setups[0].stock);
    t0 = std::chrono::steady_clock::now();
    sim.cut_upto(tool, tp, int(tp.moves.size()) - 1);
    const double sim_secs = seconds_since(t0);
    WARN("stock sim: " << sim_secs << " s at " << sim.resolution() << " mm");
    REQUIRE(sim_secs < 5.0);

    // stock 82 x 82 x 31 minus the part grown by the leave (0.5 radial, 0.3 axial)
    const double expected = 82. * 82. * 31. - (81. * 81. * 20.3 + 31. * 31. * 10.);
    WARN("removed " << sim.removed_volume() << " mm^3, expected " << expected);
    REQUIRE(sim.removed_volume() == Catch::Approx(expected).epsilon(0.05));

    // never below the part
    double worst = 0;
    for (double y = -0.9; y < 81; y += 0.2)
        for (double x = -0.9; x < 81; x += 0.2) {
            const double model_top = (x > 25 && x < 55 && y > 25 && y < 55) ? 30 : (x > 0 && x < 80 && y > 0 && y < 80) ? 20 : 0;
            worst                  = std::max(worst, model_top - sim.top_at(x, y));
        }
    INFO("deepest cut into the part " << worst);
    REQUIRE(worst <= 0.05);
    REQUIRE(gouge_check(tp, tool, part).empty());
}

