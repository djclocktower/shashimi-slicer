#include "libslic3r/CAM/Toolpath.hpp"

#include <algorithm>
#include <cmath>

namespace Slic3r::CAM {

static Move make_move(Move::Kind kind, const Vec3d& to, double a_deg, double feed)
{
    Move m;
    m.kind  = kind;
    m.to    = to;
    m.a_deg = a_deg;
    m.feed  = feed;
    return m;
}

void append_link_move(Toolpath& tp, const Vec3d& to, const ResolvedHeights& h, double short_link, const FeedsSpeeds& fs)
{
    using K = Move::Kind;
    auto plunge = [&](double a) {
        const Vec3d& cur = tp.moves.back().to;
        if (to.z() < cur.z() - 1e-9)
            tp.moves.push_back(make_move(K::Plunge, to, a, fs.plunge_feed));
        else if ((to - cur).norm() > 1e-9)
            tp.moves.push_back(make_move(K::Rapid, to, a, 0));
    };
    if (tp.moves.empty()) {
        tp.moves.push_back(make_move(K::Rapid, Vec3d(to.x(), to.y(), h.clearance), 0, 0));
        if (h.retract < h.clearance - 1e-9)
            tp.moves.push_back(make_move(K::Rapid, Vec3d(to.x(), to.y(), h.retract), 0, 0));
        plunge(0);
        return;
    }
    const Vec3d  cur = tp.moves.back().to;
    const double a   = tp.moves.back().a_deg;
    if ((to - cur).norm() < 1e-9)
        return;
    const double dxy = (to - cur).head<2>().norm();
    if (dxy <= short_link && std::abs(to.z() - cur.z()) <= 1e-6) {
        const double z = std::max(h.retract, cur.z());
        if (z > cur.z() + 1e-9)
            tp.moves.push_back(make_move(K::Retract, Vec3d(cur.x(), cur.y(), z), a, 0));
        tp.moves.push_back(make_move(K::Rapid, Vec3d(to.x(), to.y(), z), a, 0));
    } else {
        if (cur.z() < h.clearance - 1e-9)
            tp.moves.push_back(make_move(K::Retract, Vec3d(cur.x(), cur.y(), h.clearance), a, 0));
        tp.moves.push_back(make_move(K::Rapid, Vec3d(to.x(), to.y(), std::max(h.clearance, cur.z())), a, 0));
        if (h.retract < tp.moves.back().to.z() - 1e-9)
            tp.moves.push_back(make_move(K::Rapid, Vec3d(to.x(), to.y(), h.retract), a, 0));
    }
    plunge(a);
}

void append_retract(Toolpath& tp, double clearance_z)
{
    if (tp.moves.empty())
        return;
    const Move& last = tp.moves.back();
    if (last.to.z() < clearance_z - 1e-9)
        tp.moves.push_back(make_move(Move::Kind::Retract, Vec3d(last.to.x(), last.to.y(), clearance_z), last.a_deg, 0));
}

double arc_sweep(const Vec3d& from, const Move& m)
{
    const ArcDir dir = arc_dir(m);
    if (dir == ArcDir::None)
        return 0;
    const Vec2d a = from.head<2>() - m.center.head<2>();
    const Vec2d b = m.to.head<2>() - m.center.head<2>();
    double      s = std::atan2(a.x() * b.y() - a.y() * b.x(), a.dot(b));   // (-pi, pi]
    if (dir == ArcDir::CCW) {
        if (s <= 1e-9) s += 2 * M_PI;
    } else {
        if (s >= -1e-9) s -= 2 * M_PI;
    }
    return s;
}

std::vector<Vec3d> arc_points(const Vec3d& from, const Move& m, double tolerance)
{
    if (!is_arc(m))
        return {m.to};
    const double sweep = arc_sweep(from, m);
    const Vec2d  c     = m.center.head<2>();
    const Vec2d  a     = from.head<2>() - c;
    const double r     = a.norm();
    // chord error r(1 - cos(step/2)) <= tolerance
    const double step = r > tolerance ? 2 * std::acos(std::max(-1.0, 1 - tolerance / r)) : M_PI / 2;
    const int    n    = std::max(1, int(std::ceil(std::abs(sweep) / std::max(step, 1e-3))));
    const double a0   = std::atan2(a.y(), a.x());
    // Radius interpolated too, so an end point slightly off the start radius is hit exactly.
    const double r1 = (m.to.head<2>() - c).norm();
    std::vector<Vec3d> out;
    out.reserve(n);
    for (int i = 1; i < n; ++i) {
        const double t  = double(i) / n;
        const double an = a0 + sweep * t;
        const double ri = r + (r1 - r) * t;
        out.emplace_back(c.x() + ri * std::cos(an), c.y() + ri * std::sin(an), from.z() + (m.to.z() - from.z()) * t);
    }
    out.push_back(m.to);
    return out;
}

double move_length(const Vec3d& from, double from_a, const Move& m)
{
    double len;
    if (is_arc(m)) {
        const double r  = (from.head<2>() - m.center.head<2>()).norm();
        const double xy = std::abs(arc_sweep(from, m)) * r;
        len             = std::hypot(xy, m.to.z() - from.z());
    } else
        len = (m.to - from).norm();
    // Shortest way round, as the post unwraps it: one move turns at most 180 deg.
    double da = std::abs(std::fmod(m.a_deg - from_a, 360.0));
    if (da > 180) da = 360 - da;
    if (da > 1e-9) {
        // Surface travel at the tool's (mean) distance from the A axis.
        const double r = 0.5 * (Vec2d(from.y(), from.z()).norm() + Vec2d(m.to.y(), m.to.z()).norm());
        len            = std::hypot(len, da * M_PI / 180.0 * r);
    }
    return len;
}

void toolpath_stats(Toolpath& tp)
{
    tp.cut_length = tp.rapid_length = 0;
    for (size_t i = 1; i < tp.moves.size(); ++i) {
        const double l = move_length(tp.moves[i - 1].to, tp.moves[i - 1].a_deg, tp.moves[i]);
        (tp.moves[i].kind == Move::Kind::Rapid || tp.moves[i].kind == Move::Kind::Retract ? tp.rapid_length : tp.cut_length) += l;
    }
}

double estimate_time(const Toolpath& tp, const MachineProfile& machine)
{
    double t = 0;
    for (size_t i = 1; i < tp.moves.size(); ++i) {
        const Move&  m    = tp.moves[i];
        const Vec3d& from = tp.moves[i - 1].to;
        const double len  = move_length(from, tp.moves[i - 1].a_deg, m);
        if (len <= 0)
            continue;
        double feed;
        if (m.kind == Move::Kind::Rapid || m.kind == Move::Kind::Retract)
            feed = machine.rapid_feed;
        else {
            const bool mostly_z = !is_arc(m) && std::abs(m.to.z() - from.z()) > (m.to - from).head<2>().norm();
            const double limit  = mostly_z ? machine.max_feed_z : machine.max_feed_xy;
            feed = limit > 0 ? std::min(m.feed, limit) : m.feed;
        }
        if (feed > 0)
            t += len / feed * 60.0;
    }
    return t;
}

} // namespace Slic3r::CAM
