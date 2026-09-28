#pragma once

// Private to Op3D.cpp, Rotary.cpp and StockSim.cpp (engineer C). Not part of the CAM contract.
//
// Cutter profile + drop-cutter (defined in Op3D.cpp) and the generators' common preamble.

#include "libslic3r/CAM/CamDocument.hpp"
#include "libslic3r/CAM/FeedsSpeeds.hpp"
#include "libslic3r/CAM/Machines.hpp"
#include "libslic3r/CAM/Toolpath.hpp"
#include "libslic3r/AABBTreeIndirect.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace Slic3r::CAM::internal {

// ---- Cutter ---------------------------------------------------------------------------------

// Rotationally symmetric cutter: h(d) = height of the cutting surface above the tip at distance d
// from the axis (d <= R). Flat: rc = 0, cot = 0. Ball: rc = R. Bull: 0 < rc < R. Cone (V-bit,
// chamfer, drill): cot = dh/dd of the flank, rt = flat tip radius. h is convex and non-decreasing,
// which the edge test relies on.
struct Cutter {
    double R{3}, rc{0}, rt{0}, cot{0};

    bool   cone() const { return cot > 0; }
    double cone_h() const { return (R - rt) * cot; }
    // Height above the tip where the profile reaches full radius.
    double profile_h() const { return cone() ? cone_h() : rc; }
    double h(double d) const
    {
        if (cone())
            return d <= rt ? 0. : (d - rt) * cot;
        if (rc <= 0 || d <= R - rc)
            return 0.;
        const double e = d - (R - rc);
        return rc - std::sqrt(std::max(0., rc * rc - e * e));
    }
    // Contact radius against a plane of slope s (|grad z|): argmax_d s*d - h(d).
    double facet_d(double s) const
    {
        if (cone())
            return s > cot ? R : rt;
        if (rc <= 0)
            return R;
        return R - rc + rc * s / std::sqrt(1 + s * s);
    }
    // Cutter radius at height t above the tip (R above the profile).
    double radius_at(double t) const
    {
        if (t <= 0)
            return cone() ? rt : R - rc;
        if (cone())
            return t >= cone_h() ? R : rt + t / cot;
        if (rc <= 0 || t >= rc)
            return R;
        return R - rc + std::sqrt(std::max(0., rc * rc - (rc - t) * (rc - t)));
    }
};

// `inflate` grows the cutter by an offset surface (stock to leave): R + inflate, corners + inflate.
// ponytail: cones are inflated by widening the tip only (no rounded offset); fine for leave << tip.
inline Cutter make_cutter(const CamTool& tool, double inflate = 0)
{
    Cutter c;
    const double r = std::max(0.01, tool.diameter / 2);
    c.R            = r + inflate;
    switch (tool.type) {
    case ToolType::BallEndMill: c.rc = c.R; break;
    case ToolType::BullEndMill: c.rc = std::max(0., std::clamp(tool.corner_radius, 0., r) + inflate); break;
    case ToolType::ChamferMill:
    case ToolType::VBit:
    case ToolType::Drill:
    case ToolType::SpotDrill:
        if (tool.tip_angle_deg > 1 && tool.tip_angle_deg < 179) {
            c.cot = 1. / std::tan(tool.tip_angle_deg * M_PI / 360.);
            c.rt  = std::max(0., std::clamp(tool.tip_diameter / 2, 0., r) + inflate);
            break;
        }
        c.rc = std::max(0., inflate);
        break;
    default: c.rc = std::max(0., inflate); break; // flat, tap
    }
    return c;
}

// Highest tool-tip Z at (x, y) such that the cutter touches `its` without penetrating it
// (vertex, edge and facet tests over the triangles under the footprint). Keeps a reference to `its`.
class DropCutter {
public:
    DropCutter(const indexed_triangle_set& its, const Cutter& cutter);
    // `floor` when the cutter touches nothing.
    double at(double x, double y, double floor) const;
    const Cutter& cutter() const { return m_c; }

private:
    const indexed_triangle_set& m_its;
    Cutter                      m_c;
    AABBTreeIndirect::Tree3f    m_tree;
};

// Selected sketch mapped to setup XY (scaled): closed profiles -> regions, open chains -> chains.
void sketch_to_setup_xy(const CamSketch& sk, const Transform3d& to_setup, ExPolygons& regions, Polylines& chains);
// Union of the mesh's triangles projected to XY (scaled).
ExPolygons silhouette(const indexed_triangle_set& its);
// Drops pass points (collinear in XY) whose Z is within `tol` of the chord between kept points.
std::vector<Vec3d> simplify_pass(const std::vector<Vec3d>& pts, double tol);

// Z of the upward-facing horizontal triangles (flat floors / tops), sorted descending, deduplicated.
std::vector<double> horizontal_face_zs(const indexed_triangle_set& its);

// ---- Shared generator preamble ----------------------------------------------------------------

inline void add_move(Toolpath& tp, Move::Kind k, const Vec3d& to, double feed, double a_deg = 0)
{
    Move m;
    m.kind  = k;
    m.to    = to;
    m.feed  = k == Move::Kind::Rapid ? 0. : feed;
    m.a_deg = a_deg;
    tp.moves.push_back(m);
}

struct OpContext {
    const CamTool*       tool{nullptr};
    const CamSetupFrame* frame{nullptr};
    TriangleMesh         mesh;   // setup frame
    FeedsSpeeds          fs;
    ResolvedHeights      h;
    std::string          error;
};

// Validates the op (tool, setup, bodies) and resolves heights with the model extent as the selection.
inline OpContext op_context(const CamDocument& doc, const CamOperation& op, const CamModel& model, bool need_mesh = true)
{
    OpContext c;
    if (op.setup_index < 0 || op.setup_index >= int(doc.setups.size()) || op.setup_index >= int(model.setups.size())) {
        c.error = "This operation's setup is missing.";
        return c;
    }
    c.tool = doc.find_tool(op.tool_number);
    if (!c.tool) {
        c.error = "Tool " + std::to_string(op.tool_number) + " is not in the tool library.";
        return c;
    }
    const CamSetup& setup = doc.setups[op.setup_index];
    c.frame               = &model.setups[op.setup_index];
    if (need_mesh) {
        c.mesh = setup_mesh(doc, model, op.setup_index);
        if (c.mesh.its.indices.empty()) {
            c.error = "There is no model to machine in this setup.";
            return c;
        }
    }
    c.fs = effective_feeds(op, *c.tool, setup.material, find_machine(setup.machine));
    c.h  = resolve_heights(op.heights, *c.frame, c.frame->model.max.z(), c.frame->model.min.z());
    return c;
}

} // namespace Slic3r::CAM::internal
