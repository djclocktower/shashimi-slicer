#include "MetrologyMesh.hpp"

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Tesselate.hpp"

namespace Slic3r { namespace GUI {

static Polygon to_polygon(const Metrology::Polygon2& loop)
{
    Polygon p;
    p.points.reserve(loop.size());
    for (const Metrology::P2& q : loop) p.points.emplace_back(scale_(q.x), scale_(q.y));
    return p;
}

ExPolygons to_expolygons(const std::vector<Metrology::Region>& regions)
{
    ExPolygons out;
    for (const Metrology::Region& r : regions) {
        if (r.outer.size() < 3) continue;
        ExPolygon e;
        e.contour = to_polygon(r.outer);
        if (!e.contour.is_counter_clockwise()) e.contour.reverse();
        for (const Metrology::Polygon2& h : r.holes) {
            if (h.size() < 3) continue;
            Polygon hp = to_polygon(h);
            if (hp.is_counter_clockwise()) hp.reverse();
            e.holes.push_back(std::move(hp));
        }
        out.push_back(std::move(e));
    }
    return out;
}

static void add_walls(Pointf3s& walls, const ExPolygons& ex, double z0, double z1)
{
    auto wall = [&](const Polygon& poly) {
        for (size_t i = 0, n = poly.points.size(); i < n; ++i) {
            const Vec2d p = unscale(poly.points[i]), q = unscale(poly.points[(i + 1) % n]);
            const Vec3d p0(p.x(), p.y(), z0), q0(q.x(), q.y(), z0), p1(p.x(), p.y(), z1), q1(q.x(), q.y(), z1);
            walls.insert(walls.end(), {p0, q0, q1, p0, q1, p1});
        }
    };
    for (const ExPolygon& e : ex) {
        wall(e.contour);
        for (const Polygon& h : e.holes) wall(h);
    }
}

indexed_triangle_set extrude_regions(const std::vector<Metrology::Region>& regions, double z0, double z1)
{
    indexed_triangle_set its;
    const ExPolygons     ex = to_expolygons(regions);
    if (ex.empty() || z1 <= z0) return its;
    its_merge(its, triangulate_expolygons_3d(ex, z1, NORMALS_UP));
    its_merge(its, triangulate_expolygons_3d(ex, z0, NORMALS_DOWN));
    // Walls: contours are CCW and holes CW, so (q - p) x up points out of the material.
    Pointf3s walls;
    add_walls(walls, ex, z0, z1);
    its_merge(its, walls);
    its_merge_vertices(its);
    return its;
}

TriangleMesh model_mesh(const Metrology::ReModel& model)
{
    // One closed shell: the bottom, each level's walls from the level below it, and each level's
    // exposed top (its area minus the next level's).
    std::vector<ExPolygons> ex;
    std::vector<double>     z;
    for (const Metrology::ReLevel& l : model.levels) {
        ExPolygons e = to_expolygons(l.regions);
        if (!ex.empty()) e = intersection_ex(e, ex.back());   // a level never overhangs the one below
        if (e.empty() || (!z.empty() && l.z <= z.back() + 1e-6)) continue;
        ex.push_back(std::move(e));
        z.push_back(l.z);
    }
    indexed_triangle_set its;
    if (ex.empty()) return TriangleMesh(std::move(its));
    its_merge(its, triangulate_expolygons_3d(ex.front(), 0., NORMALS_DOWN));
    Pointf3s walls;
    for (size_t k = 0; k < ex.size(); ++k) {
        add_walls(walls, ex[k], k == 0 ? 0. : z[k - 1], z[k]);
        const ExPolygons top = k + 1 < ex.size() ? diff_ex(ex[k], ex[k + 1]) : ex[k];
        its_merge(its, triangulate_expolygons_3d(top, z[k], NORMALS_UP));
    }
    its_merge(its, walls);
    its_merge_vertices(its);
    return TriangleMesh(std::move(its));
}

}} // namespace Slic3r::GUI
