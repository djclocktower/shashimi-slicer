#include "libslic3r/CAM/Drill.hpp"

#include "libslic3r/CAD/GeometryEngine.hpp"
#include "libslic3r/CAM/CamGeometry.hpp"
#include "libslic3r/CAM/FeedsSpeeds.hpp"
#include "libslic3r/CAM/Machines.hpp"
#include "libslic3r/CAM/Toolpath.hpp"

#include <Standard_Failure.hxx>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>

namespace Slic3r::CAM {

namespace {

using K = Move::Kind;

bool setup_has_body(const CamSetup& s, int id)
{
    return s.body_ids.empty() || std::find(s.body_ids.begin(), s.body_ids.end(), id) != s.body_ids.end();
}

// Same axis line (setup frame).
bool coaxial(const HoleFeature& h, const Vec3d& p, const Vec3d& axis)
{
    const Vec3d d = p - h.center;
    return std::abs(std::abs(h.axis.dot(axis.normalized())) - 1) < 1e-6 && (d - h.axis * d.dot(h.axis)).norm() < 1e-3;
}

void add(Toolpath& tp, K kind, const Vec3d& to, double feed, int cycle)
{
    Move m;
    m.kind  = kind;
    m.to    = to;
    m.feed  = kind == K::Rapid || kind == K::Retract ? 0 : feed;
    m.cycle = cycle;
    m.a_deg = tp.moves.empty() ? 0 : tp.moves.back().a_deg;
    tp.moves.push_back(m);
}

std::string fmt(const char* f, double a, double b = 0)
{
    char buf[256];
    snprintf(buf, sizeof(buf), f, a, b);
    return buf;
}

} // namespace

std::vector<HoleFeature> holes_for_op(const CamDocument& doc, const CamOperation& op, const CamModel& model)
{
    std::vector<HoleFeature> out;
    if (op.setup_index < 0 || op.setup_index >= int(doc.setups.size()))
        return out;
    const CamSetup&     setup = doc.setups[op.setup_index];
    const CamSetupFrame frame = op.setup_index < int(model.setups.size()) ? model.setups[op.setup_index] : compute_setup_frame(setup, model);
    const Transform3d&  T     = frame.to_setup;

    std::map<int, std::vector<HoleFeature>> found;   // per body, only the +Z ones
    auto holes_of = [&](int body_id) -> const std::vector<HoleFeature>& {
        auto it = found.find(body_id);
        if (it == found.end()) {
            std::vector<HoleFeature> up;
            if (const CamBody* b = model.body(body_id); b && b->shape)
                for (HoleFeature h : find_holes(*b->shape, T))
                    if (h.axis.z() > 1 - 1e-3) {
                        h.body = body_id;
                        up.push_back(h);
                    }
            it = found.emplace(body_id, std::move(up)).first;
        }
        return it->second;
    };
    auto add_hole = [&](const HoleFeature& h) {
        // diameter filter: only recognised holes (picked points have no diameter)
        if (h.diameter > 0 && ((op.hole_diameter_min > 0 && h.diameter < op.hole_diameter_min - 1e-6) ||
                               (op.hole_diameter_max > 0 && h.diameter > op.hole_diameter_max + 1e-6)))
            return;
        for (const HoleFeature& o : out)
            if ((o.center - h.center).head<2>().norm() < 1e-3)
                return;
        out.push_back(h);
    };

    if (op.geom.whole_model)
        for (const CamBody& b : model.bodies)
            if (setup_has_body(setup, b.body_id))
                for (const HoleFeature& h : holes_of(b.body_id))
                    add_hole(h);

    for (const FaceRef& f : op.geom.faces) {
        const CamBody* b = model.body(f.body);
        if (!b || !b->shape)
            continue;
        try {
            const std::vector<TopoDS_Face> faces = GeometryEngine::faces_of(b->shape->shape);
            if (f.face < 0 || f.face >= int(faces.size()))
                continue;
            const GeometryEngine::CylinderFace cf = GeometryEngine::cylinder_of_face(faces[f.face]);
            if (cf.ok) {
                // a bore face: its hole
                for (const HoleFeature& h : holes_of(f.body))
                    if (coaxial(h, T * cf.base, T.linear() * cf.axis))
                        add_hole(h);
                continue;
            }
        } catch (const Standard_Failure&) {
            continue;
        }
        // a flat face: every hole opening in it
        double           z  = 0;
        const ExPolygons ex = face_outline(*b->shape, f.face, T, z);
        for (const HoleFeature& h : holes_of(f.body)) {
            if (std::abs(h.center.z() - z) > 1e-3)
                continue;
            const Point p(coord_t(scale_(h.center.x())), coord_t(scale_(h.center.y())));
            for (const ExPolygon& e : ex)
                if (e.contour.contains(p))
                    add_hole(h);
        }
    }

    for (const EdgeRef& e : op.geom.edges) {
        const CamBody* b = model.body(e.body);
        if (!b || !b->shape)
            continue;
        GeometryEngine::CylinderFace c;
        try {
            const std::vector<TopoDS_Edge> edges = GeometryEngine::edges_of(b->shape->shape);
            if (e.edge < 0 || e.edge >= int(edges.size()))
                continue;
            c = GeometryEngine::circle_of_edge(edges[e.edge]);
        } catch (const Standard_Failure&) {
            continue;
        }
        if (!c.ok)
            continue;
        const Vec3d center = T * c.base, axis = T.linear() * c.axis;
        if (std::abs(axis.normalized().z()) < 1 - 1e-3)
            continue;
        bool known = false;
        for (const HoleFeature& h : holes_of(e.body))
            if (coaxial(h, center, axis)) {
                add_hole(h);
                known = true;
            }
        if (!known) {
            HoleFeature h;
            h.center   = center;
            h.diameter = 2 * c.radius;
            h.body     = e.body;
            add_hole(h);
        }
    }

    for (const Vec3d& p : op.geom.points) {
        HoleFeature h;
        h.center = p;
        add_hole(h);
    }
    if (op.geom.sketch_feature >= 0)
        if (const CamSketch* s = model.sketch(op.geom.sketch_feature))
            for (const Vec2d& p : s->points) {
                HoleFeature h;
                h.center = T * (s->origin + s->x_axis * p.x() + s->y_axis * p.y());
                add_hole(h);
            }

    // Nearest neighbour from the WCS origin.
    std::vector<HoleFeature> sorted;
    Vec2d                    at = Vec2d::Zero();
    while (!out.empty()) {
        size_t best = 0;
        for (size_t i = 1; i < out.size(); ++i)
            if ((out[i].center.head<2>() - at).squaredNorm() < (out[best].center.head<2>() - at).squaredNorm())
                best = i;
        at = out[best].center.head<2>();
        sorted.push_back(out[best]);
        out.erase(out.begin() + best);
    }
    return sorted;
}

Toolpath generate_drill(const CamDocument& doc, const CamOperation& op, const CamModel& model, const ProgressFn& progress)
{
    Toolpath       tp;
    const CamTool* tool = doc.find_tool(op.tool_number);
    if (!tool) {
        tp.error = "The operation's tool (T" + std::to_string(op.tool_number) + ") is not in the tool library.";
        return tp;
    }
    if (op.setup_index < 0 || op.setup_index >= int(doc.setups.size())) {
        tp.error = "The operation's setup does not exist.";
        return tp;
    }
    const CamSetup&       setup = doc.setups[op.setup_index];
    const CamSetupFrame   frame = op.setup_index < int(model.setups.size()) ? model.setups[op.setup_index] : compute_setup_frame(setup, model);
    const FeedsSpeeds     fs    = effective_feeds(op, *tool, setup.material, find_machine(setup.machine));
    const std::vector<HoleFeature> holes = holes_for_op(doc, op, model);
    if (holes.empty()) {
        tp.error = op.hole_diameter_min > 0 || op.hole_diameter_max > 0
                       ? "No hole in the selection is within the hole diameter filter."
                       : "No holes found in the selection. Pick hole faces, hole edges, a face with holes or points.";
        return tp;
    }
    if (op.type == OpType::Drill && op.cycle == DrillCycle::Tap && tool->type != ToolType::Tap)
        tp.warnings.push_back({-1, Vec3d::Zero(), "The tapping cycle is used with a tool that is not a tap."});
    if (op.type == OpType::Drill && op.cycle == DrillCycle::Tap && !(tool->thread_pitch > 0)) {
        tp.error = "The tap has no thread pitch. Set its pitch in the Tool Library.";
        return tp;
    }

    const bool   is_drill = tool->type == ToolType::Drill;
    // Cone length of the drill point (118 deg: 0.3 d), added to through holes.
    const double tip_len = is_drill ? 0.5 * tool->diameter / std::tan(0.5 * std::clamp(tool->tip_angle_deg, 60.0, 179.0) * M_PI / 180.0) : 0;
    const double feed    = fs.plunge_feed > 0 ? (op.type == OpType::Drill ? fs.plunge_feed : fs.feed) : fs.feed;

    double clearance = -1e30;
    int    done      = 0;
    for (size_t i = 0; i < holes.size(); ++i) {
        if (progress && progress(double(i) / holes.size())) {
            tp.error = "Cancelled";
            return tp;
        }
        const HoleFeature& hole  = holes[i];
        const bool         spot  = tool->type == ToolType::SpotDrill || tool->type == ToolType::ChamferMill;
        if (hole.diameter > 0 && !spot && tool->diameter > hole.diameter + 1e-3) {
            tp.warnings.push_back({-1, hole.center, fmt("The %g mm tool is larger than the %g mm hole; the hole was skipped.",
                                                        tool->diameter, hole.diameter)});
            continue;
        }
        const double sel_bottom = hole.center.z() - hole.depth - (hole.through ? std::max(0., op.break_through) + tip_len : 0);
        const ResolvedHeights h = resolve_heights(op.heights, frame, hole.center.z(), sel_bottom);
        clearance = std::max(clearance, h.clearance);
        double       bottom = h.bottom;
        const double top    = std::min(hole.center.z(), h.retract);
        if (tool->type == ToolType::SpotDrill && hole.diameter > 0 && op.type == OpType::Drill)
            // Spot a known hole to a chamfer op.chamfer_width wider than its radius.
            bottom = hole.center.z() - std::min(0.5 * hole.diameter + std::max(0., op.chamfer_width), 0.5 * tool->diameter) /
                                           std::tan(0.5 * std::clamp(tool->tip_angle_deg, 60.0, 170.0) * M_PI / 180.0);
        if (bottom >= top - 1e-6) {
            tp.warnings.push_back({-1, hole.center, "A hole has no depth to drill (check the Bottom height); it was skipped."});
            continue;
        }
        const double x = hole.center.x(), y = hole.center.y(), R = h.retract;
        append_link_move(tp, Vec3d(x, y, R), h, 1e9, fs);   // between holes: hop at the R plane

        const int c = int(i);
        if (op.type == OpType::Bore) {
            // Helical bore: tool centre on a circle of (hole - tool) / 2, one stepdown per turn.
            const double rb = 0.5 * (hole.diameter - tool->diameter);
            if (hole.diameter <= 0 || rb < 0.01 * tool->diameter) {
                tp.warnings.push_back({-1, hole.center, "A hole is not wider than the tool, so it cannot be bored with a helix; it was skipped."});
                continue;
            }
            if (hole.diameter > 2 * tool->diameter)
                tp.warnings.push_back({-1, hole.center, fmt("The %g mm hole is more than twice the tool width: its centre is left standing. Use a Pocket for it.",
                                                            hole.diameter)});
            const ArcDir dir   = op.climb ? ArcDir::CCW : ArcDir::CW;
            const double pitch = std::max(0.05, op.stepdown);   // per full turn
            const int    n     = std::max(1, int(std::ceil((top - bottom) / (0.5 * pitch) - 1e-9)));   // half turns
            add(tp, K::Feed, Vec3d(x + rb, y, top), fs.feed, -1);
            for (int k = 1; k <= n + 2; ++k) {
                Move m;
                m.kind   = k <= n ? K::Ramp : K::Feed;
                m.to     = Vec3d(x + ((k % 2) ? -rb : rb), y, k <= n ? top + (bottom - top) * k / n : bottom);
                m.feed   = k <= n ? fs.ramp_feed : fs.feed;
                m.arc    = dir;
                m.center = Vec3d(x, y, 0);
                m.a_deg  = tp.moves.back().a_deg;
                tp.moves.push_back(m);
            }
            add(tp, K::Feed, Vec3d(x, y, bottom), fs.feed, -1);
            add(tp, K::Retract, Vec3d(x, y, R), 0, -1);
            ++done;
            continue;
        }

        switch (op.cycle) {
        case DrillCycle::Drill:
            add(tp, K::Plunge, Vec3d(x, y, bottom), feed, c);
            add(tp, K::Retract, Vec3d(x, y, R), 0, c);
            break;
        case DrillCycle::Peck:
        case DrillCycle::ChipBreak: {
            const double q = std::max(0.05, op.peck_depth);
            double       z = top;
            while (z > bottom + 1e-9) {
                const double zn = std::max(bottom, z - q);
                add(tp, K::Plunge, Vec3d(x, y, zn), feed, c);
                if (zn <= bottom + 1e-9)
                    break;
                if (op.cycle == DrillCycle::Peck) {
                    // G83: full retract to clear chips, rapid back to just above the last depth.
                    add(tp, K::Retract, Vec3d(x, y, R), 0, c);
                    add(tp, K::Rapid, Vec3d(x, y, std::min(R, zn + 0.5)), 0, c);
                } else
                    add(tp, K::Retract, Vec3d(x, y, zn + 0.2), 0, c);   // G73: short chip-breaking retract
                z = zn;
            }
            add(tp, K::Retract, Vec3d(x, y, R), 0, c);
            break;
        }
        case DrillCycle::Bore:   // G85: feed in, feed out
            add(tp, K::Plunge, Vec3d(x, y, bottom), feed, c);
            add(tp, K::Feed, Vec3d(x, y, R), feed, c);
            break;
        case DrillCycle::Tap: {  // G84: feed = rpm * pitch in, spindle reverse, same feed out
            const double tf = fs.rpm * tool->thread_pitch;   // pitch checked above
            add(tp, K::Plunge, Vec3d(x, y, bottom), tf, c);
            add(tp, K::Feed, Vec3d(x, y, R), tf, c);
            break;
        }
        }
        ++done;
    }
    if (done == 0 && tp.error.empty()) {
        tp.error = tp.warnings.empty() ? "No holes could be machined." : tp.warnings.front().text;
        return tp;
    }
    append_retract(tp, clearance);
    return tp;
}

} // namespace Slic3r::CAM
