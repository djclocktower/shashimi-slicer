#include "Features.hpp"

#include "Image.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace Slic3r::Metrology {

const char* feature_kind_name(FeatureKind k)
{
    switch (k) {
    case FeatureKind::Line: return "Line";
    case FeatureKind::Circle: return "Circle";
    case FeatureKind::Width: return "Width";
    case FeatureKind::Distance: return "Distance";
    case FeatureKind::Angle: return "Angle";
    }
    return "?";
}

Feature& Program::add(Feature f)
{
    f.id = next_id++;
    if (f.name.empty()) f.name = std::string(feature_kind_name(f.kind)) + " " + std::to_string(f.id);
    features.push_back(std::move(f));
    return features.back();
}

Feature* Program::find(int id)
{
    for (Feature& f : features)
        if (f.id == id) return &f;
    return nullptr;
}

const Feature* Program::find(int id) const
{
    for (const Feature& f : features)
        if (f.id == id) return &f;
    return nullptr;
}

void Program::remove(int id)
{
    features.erase(std::remove_if(features.begin(), features.end(), [id](const Feature& f) { return f.id == id; }), features.end());
    for (Feature& f : features) {
        if (f.ref_a == id) f.ref_a = -1;
        if (f.ref_b == id) f.ref_b = -1;
    }
}

// Edge points onto the plate. An edge against the background is the part's outline: seen at an
// angle it is the top edge where the material's outward normal faces away from the camera's nadir
// and the bottom edge elsewhere. An edge against more of the part (a step, a wall) is a top edge.
// Each point is mapped onto the plane it actually lies on.
static std::vector<P2> to_mm(const std::vector<EdgeHit>& hits, const GrayImage& img, const Levels& lv, const CameraModel& cam, double z,
                             bool part_bright)
{
    std::vector<P2> out;
    for (const EdgeHit& h : hits) {
        const P2 pz = cam.to_plane(h.p, z);
        if (z <= 0 || !cam.has_center) {
            out.push_back(z <= 0 ? cam.to_plane(h.p, 0.0) : pz);
            continue;
        }
        // Out of the material is towards the dark side for a bright part.
        const bool   rising = h.strength > 0;   // dark to light along the scan
        const P2     out_px = (rising == part_bright) ? h.dir * -1.0 : h.dir;
        const P2     probe  = h.p + out_px * 4.0;
        const double beyond = img.sample(probe.x, probe.y);
        const bool   background = part_bright ? beyond < lv.threshold : beyond > lv.threshold;
        if (!background) {
            out.push_back(pz);
            continue;
        }
        const P2 out_mm = cam.to_plane(h.p + out_px, z) - pz;
        out.push_back(out_mm.dot(pz - cam.nadir) > 0 ? pz : cam.to_plane(h.p, 0.0));
    }
    return out;
}

static void run_primitive(Feature& f, const GrayImage& img, const Levels& lv, const CameraModel& cam, bool part_bright)
{
    f.edge_px.clear();
    f.edge2_px.clear();
    switch (f.kind) {
    case FeatureKind::Line: {
        const auto hits = scan_rect(img, f.rect, f.scans, f.edge);
        f.edge_px       = hit_points(hits);
        if (f.edge_px.size() < 3) {
            f.error = "Fewer than 3 edge points: move the box over a clear edge or lower the contrast threshold.";
            return;
        }
        f.line = fit_line_robust(to_mm(hits, img, lv, cam, f.plane_z, part_bright));
        if (!f.line.ok) {
            f.error = "The line fit failed.";
            return;
        }
        f.value = line_direction_deg(f.line);
        f.aux   = f.line.max_dev;
        f.ok    = true;
        break;
    }
    case FeatureKind::Circle: {
        const auto hits = scan_annulus(img, f.center, f.r_in, f.r_out, f.scans, f.edge);
        f.edge_px       = hit_points(hits);
        if (f.edge_px.size() < 5) {
            f.error = "Fewer than 5 edge points: centre the ring on the circle's edge.";
            return;
        }
        f.circle = fit_circle_robust(to_mm(hits, img, lv, cam, f.plane_z, part_bright));
        if (!f.circle.ok) {
            f.error = "The circle fit failed.";
            return;
        }
        f.value = 2 * f.circle.r;
        f.aux   = f.circle.rms;
        f.ok    = true;
        break;
    }
    case FeatureKind::Width: {
        std::vector<EdgeHit> h1, h2;
        scan_rect_pair(img, f.rect, f.scans, f.edge, h1, h2);
        f.edge_px  = hit_points(h1);
        f.edge2_px = hit_points(h2);
        if (f.edge_px.size() < 3) {
            f.error = "Fewer than 3 scans found two edges: the box must span both edges.";
            return;
        }
        const auto a = to_mm(h1, img, lv, cam, f.plane_z, part_bright), b = to_mm(h2, img, lv, cam, f.plane_z, part_bright);
        f.line  = fit_line_robust(a);
        f.line2 = fit_line_robust(b);
        if (!f.line.ok || !f.line2.ok) {
            f.error = "The edge lines could not be fitted.";
            return;
        }
        // Mean distance of each side's points to the other side's line, both ways.
        double s = 0, mn = 1e9, mx = -1e9;
        for (const P2& p : b) {
            const double d = std::abs(f.line.distance(p));
            s += d;
            mn = std::min(mn, d);
            mx = std::max(mx, d);
        }
        for (const P2& p : a) s += std::abs(f.line2.distance(p));
        f.value = s / double(a.size() + b.size());
        f.aux   = mx - mn;
        f.ok    = true;
        break;
    }
    default: break;
    }
}

// A representative point of a feature for distances: the circle centre or the line's centroid.
static bool anchor(const Feature& f, P2& p)
{
    if (!f.ok) return false;
    if (f.kind == FeatureKind::Circle) {
        p = f.circle.c;
        return true;
    }
    if (f.kind == FeatureKind::Line || f.kind == FeatureKind::Width) {
        p = f.line.p;
        return true;
    }
    return false;
}

void Program::run(const GrayImage& img, const CameraModel& cam)
{
    for (Feature& f : features) {
        f.ok      = false;
        f.error.clear();
        f.verdict = Judge::None;
    }
    const Levels lv = part_levels(img, part_bright);
    for (Feature& f : features)
        if (f.kind == FeatureKind::Line || f.kind == FeatureKind::Circle || f.kind == FeatureKind::Width) run_primitive(f, img, lv, cam, part_bright);
    for (Feature& f : features) {
        if (f.kind != FeatureKind::Distance && f.kind != FeatureKind::Angle) continue;
        const Feature* a = find(f.ref_a);
        const Feature* b = find(f.ref_b);
        if (!a || !b || !a->ok || !b->ok) {
            f.error = "Needs two measured features.";
            continue;
        }
        if (f.kind == FeatureKind::Angle) {
            if (a->kind == FeatureKind::Circle || b->kind == FeatureKind::Circle) {
                f.error = "An angle needs two lines.";
                continue;
            }
            f.value = line_angle_deg(a->line, b->line);
            f.ok    = true;
        } else {
            P2 pa, pb;
            anchor(*a, pa);
            anchor(*b, pb);
            // A circle or point to a line: perpendicular distance; otherwise point to point.
            if (a->kind != FeatureKind::Circle && b->kind == FeatureKind::Circle) f.value = std::abs(a->line.distance(pb));
            else if (b->kind != FeatureKind::Circle && a->kind == FeatureKind::Circle) f.value = std::abs(b->line.distance(pa));
            else if (a->kind != FeatureKind::Circle && b->kind != FeatureKind::Circle && line_angle_deg(a->line, b->line) < 5)
                f.value = std::abs(a->line.distance(pb));   // near-parallel lines
            else
                f.value = (pa - pb).norm();
            f.ok = true;
        }
    }
    for (Feature& f : features)
        if (f.ok) f.verdict = judge(f.tol, f.value);
}

Judge Program::overall() const
{
    bool any_ok = false;
    for (const Feature& f : features) {
        if (f.verdict == Judge::NG) return Judge::NG;
        if (!f.ok && f.tol.enabled) return Judge::NG;
        any_ok = any_ok || f.verdict == Judge::OK;
    }
    return any_ok ? Judge::OK : Judge::None;
}

std::string Program::to_csv() const
{
    std::ostringstream s;
    s << std::fixed << std::setprecision(4);
    s << "No,Name,Type,Measured,Unit,Nominal,Plus,Minus,Deviation,Judge,Note\n";
    for (const Feature& f : features) {
        s << f.id << ",\"" << f.name << "\"," << feature_kind_name(f.kind) << ',';
        if (f.ok) s << f.value;
        s << ',' << f.unit() << ',';
        if (f.tol.enabled) s << f.tol.nominal << ',' << f.tol.plus << ',' << f.tol.minus << ',' << (f.ok ? f.value - f.tol.nominal : 0.0);
        else s << ",,,";
        s << ',' << (f.verdict == Judge::OK ? "OK" : f.verdict == Judge::NG ? "NG" : "") << ",\"" << f.error << "\"\n";
    }
    return s.str();
}

} // namespace Slic3r::Metrology
