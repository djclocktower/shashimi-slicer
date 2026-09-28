#pragma once

// Drill (spot/drill/peck/chip-break/bore/tap cycles) and Bore (helical bore) operations.

#include "libslic3r/CAM/CamDocument.hpp"

#include <vector>

namespace Slic3r::CAM {

// Holes the op machines, setup frame, axis +Z only: find_holes() on the selected faces/edges
// (whole_model: every body of the setup), plus op.geom.points and the selected sketch's points
// (depth from the op heights). Sorted for a short travel path.
std::vector<HoleFeature> holes_for_op(const CamDocument& doc, const CamOperation& op, const CamModel& model);

// OpType::Drill: one canned-cycle style sequence per hole, tagged with Move::cycle (the post turns
// them into G81/G83/... or keeps them expanded). Depth = op heights with the hole's top/bottom as
// the selection; through holes add op.break_through + the drill point (118 deg: 0.3 d). A spot
// drill on a recognised hole cuts a chamfer op.chamfer_width wider than the hole's radius.
// OpType::Bore: helical bore of each hole (one op.stepdown per turn).
Toolpath generate_drill(const CamDocument& doc, const CamOperation& op, const CamModel& model,
                        const ProgressFn& progress = {});

} // namespace Slic3r::CAM
