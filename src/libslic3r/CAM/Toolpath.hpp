#pragma once

// Toolpath building and measuring helpers shared by every generator.

#include "libslic3r/CAM/CamTypes.hpp"

namespace Slic3r::CAM {

// Moves the tool from the toolpath's last position to `to` without cutting stock:
//  - empty toolpath: Rapid to (to.xy, clearance), Rapid to retract, Plunge to to.z;
//  - `to` within `short_link` (mm, XY; generators pass their stepover) of the last point and at
//    the same Z (+-1e-6): Retract to max(retract, current Z), Rapid across, Plunge to to.z;
//  - otherwise: Retract to clearance, Rapid to (to.xy, clearance), Rapid to retract, Plunge to to.z.
// Plunges use fs.plunge_feed; a plunge that starts below `retract` is not possible by construction.
void append_link_move(Toolpath& tp, const Vec3d& to, const ResolvedHeights& h, double short_link,
                      const FeedsSpeeds& fs);

// Final Retract to clearance (no-op when already there).
void append_retract(Toolpath& tp, double clearance_z);

// Recomputes tp.cut_length and tp.rapid_length (arcs measured as arcs).
void toolpath_stats(Toolpath& tp);

// Machining time in seconds: rapids at machine.rapid_feed, cutting moves at their feed clamped to
// machine.max_feed_xy / max_feed_z. No acceleration model.
double estimate_time(const Toolpath& tp, const MachineProfile& machine);

} // namespace Slic3r::CAM
