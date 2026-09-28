#pragma once

// Toolpath building and measuring helpers shared by every generator.

#include "libslic3r/CAM/CamTypes.hpp"

#include <algorithm>
#include <atomic>

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

// Signed XY sweep of an arc move starting at `from` (radians, CCW positive). A start equal to the
// end (XY) is a full turn. 0 for a non-arc move.
double arc_sweep(const Vec3d& from, const Move& m);
// Points along an arc move from `from` (excluded) to m.to (included), chord error <= tolerance
// (mm), Z (and A) interpolated linearly: for posts without arcs and for sampling. Non-arc: {m.to}.
std::vector<Vec3d> arc_points(const Vec3d& from, const Move& m, double tolerance);
// Length of one move from `from` (arcs as helices; A rotation as arc length at the tool's
// distance from the X axis, the shorter way from `from_a` to m.a_deg: a move turns <= 180 deg).
double move_length(const Vec3d& from, double from_a, const Move& m);

// Machining time in seconds: rapids at machine.rapid_feed, cutting moves at their feed clamped to
// machine.max_feed_xy / max_feed_z. No acceleration model.
double estimate_time(const Toolpath& tp, const MachineProfile& machine);

// Progress over `total` work items, safe to use from parallel loops: step() counts one item and
// reports lo..hi; poll() re-reports without counting (e.g. from Adaptive::Params::cancel). Both
// return true once the callback asked to cancel; skip remaining work then.
class ProgressCounter
{
public:
    ProgressCounter(const ProgressFn& fn, size_t total, double lo = 0, double hi = 1)
        : m_fn(fn), m_total(std::max<size_t>(1, total)), m_lo(lo), m_hi(hi) {}
    bool step() { ++m_done; return poll(); }
    bool poll()
    {
        if (!m_stop && m_fn && m_fn(m_lo + (m_hi - m_lo) * double(std::min(m_done.load(), m_total)) / double(m_total)))
            m_stop = true;
        return m_stop;
    }
    bool cancelled() const { return m_stop; }

private:
    const ProgressFn&   m_fn;
    size_t              m_total;
    double              m_lo, m_hi;
    std::atomic<size_t> m_done{0};
    std::atomic<bool>   m_stop{false};
};

} // namespace Slic3r::CAM
