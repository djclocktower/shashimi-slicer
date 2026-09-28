#pragma once

// 4-axis (A about +X) strategies. The rotary axis is the setup-frame X axis (Y = Z = 0).

#include "libslic3r/CAM/CamDocument.hpp"

#include <cmath>

namespace Slic3r::CAM {

// Unrolled cylinder of radius r: setup Y (arc length) <-> A.
inline double wrap_y_to_a_deg(double y, double r) { return y / r * 180.0 / M_PI; }
inline double wrap_a_deg_to_y(double a_deg, double r) { return a_deg * M_PI / 180.0 * r; }

// op.wrap_strategy (a 2D op) computed on the unrolled cylinder (op.wrap_radius, 0 = stock radius)
// and emitted as X/Z/A moves (Y = 0).
Toolpath generate_rotary_wrap(const CamDocument& doc, const CamOperation& op, const CamModel& model);
// Radial drop-cutter around X; passes along X at op.a_stepover_deg steps, or a spiral.
Toolpath generate_rotary_finish(const CamDocument& doc, const CamOperation& op, const CamModel& model);

// `frame` followed by a rotation of a_deg about +X: what an indexed setup's to_setup is made of.
Transform3d apply_index(const Transform3d& frame, double a_deg);

} // namespace Slic3r::CAM
