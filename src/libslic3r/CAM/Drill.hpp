#pragma once

// Drill (spot/drill/peck/chip-break/bore/tap cycles) and Bore (helical bore) operations.

#include "libslic3r/CAM/CamDocument.hpp"

#include <vector>

namespace Slic3r::CAM {

// Holes the op machines, setup frame, axis +Z only: find_holes() on the selected faces/edges
// (whole_model: every body of the setup), plus op.geom.points and the selected sketch's points
// (depth from the op heights). Sorted for a short travel path.
std::vector<HoleFeature> holes_for_op(const CamDocument& doc, const CamOperation& op, const CamModel& model);

// OpType::Drill: one canned-cycle style sequence per hole (Move kinds only; the post turns them
// into G81/G83/... or keeps them expanded). OpType::Bore: helical bore of each hole.
Toolpath generate_drill(const CamDocument& doc, const CamOperation& op, const CamModel& model);

} // namespace Slic3r::CAM
