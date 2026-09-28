#pragma once

// 2D strategies: Face, Adaptive2D, Pocket2D, Contour2D, Slot, Chamfer2D, Engrave, Trace.

#include "libslic3r/CAM/CamDocument.hpp"

namespace Slic3r::CAM {

// An op's selection resolved to setup XY (scaled): picked planar faces and closed sketch profiles
// -> regions; picked edges and open sketch chains -> chains. z_top/z_bottom = the selection's Z
// extent (sketch: its plane's Z). Non-empty error when nothing usable was selected.
struct Region2D {
    ExPolygons  regions;
    Polylines   chains;
    double      z_top{0};
    double      z_bottom{0};
    std::string error;
};
Region2D resolve_selection_2d(const CamDocument& doc, const CamOperation& op, const CamModel& model);

// Dispatch for every 2D OpType above.
Toolpath generate_2d(const CamDocument& doc, const CamOperation& op, const CamModel& model);

// Region-level building blocks (reused by Adaptive3D levels and RotaryWrap on the unrolled
// cylinder). Cut from h.top down to h.bottom in op.stepdown levels, linking at h.retract/h.clearance.
// Concentric inward offsets, helix/ramp entry, finishing passes.
Toolpath pocket_region(const ExPolygons& region, const CamTool& tool, const CamOperation& op,
                       const FeedsSpeeds& fs, const ResolvedHeights& h);
// Tool-radius offset per op.side, multi-depth, lead-in/out arcs of op.lead_in_radius.
Toolpath contour_region(const ExPolygons& region, const CamTool& tool, const CamOperation& op,
                        const FeedsSpeeds& fs, const ResolvedHeights& h);
// Zig-zag over `outline` (normally the stock outline) at op.stepover.
Toolpath face_region(const ExPolygons& outline, const CamTool& tool, const CamOperation& op,
                     const FeedsSpeeds& fs, const ResolvedHeights& h);
// Tool centre along each chain (Slot/Engrave/Trace).
Toolpath trace_chains(const Polylines& chains, const CamTool& tool, const CamOperation& op,
                      const FeedsSpeeds& fs, const ResolvedHeights& h);

} // namespace Slic3r::CAM
