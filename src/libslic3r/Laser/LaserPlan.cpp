#include "libslic3r/Laser/LaserPlan.hpp"

#include "libslic3r/Laser/ImageEngrave.hpp"
#include "libslic3r/Laser/VectorOps.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace Slic3r::Laser {

namespace {

// Overscan length of a scan line, the same rule as raster_to_scanlines().
double overscan_mm(const LaserLayer& l, double speed) { return l.overscan_mm > 0 ? l.overscan_mm : std::max(0., speed * l.overscan_pct / 100.); }

struct Builder {
    LaserJob&          job;
    const LaserDevice& device;
    Vec2d              pos{0, 0};   // workspace until finish()
    BoundingBoxf       lit;         // lit geometry without overscan (job-origin anchor, bed check)

    // One job segment, preceded by a travel when the head is elsewhere.
    void travel_to(const Vec2d& p, const Segment& like)
    {
        if ((p - pos).squaredNorm() < 1e-12) return;
        Segment t;
        t.kind       = Segment::Kind::Travel;
        t.from       = pos;
        t.to         = p;
        t.speed_mm_s = device.travel_speed_mm_s;
        t.z          = like.z;
        t.layer      = like.layer;
        t.air_assist = like.air_assist;
        job.segments.push_back(t);
        pos = p;
    }
    void cut_path(const Points& pts, bool closed, Segment like)
    {
        if (pts.size() < 2) return;
        like.kind = Segment::Kind::Cut;
        travel_to(unscale(pts.front()), like);
        const size_t n = closed ? pts.size() + 1 : pts.size();
        for (size_t i = 1; i < n; ++i) {
            like.from = pos;
            like.to   = unscale(pts[i % pts.size()]);
            if ((like.to - like.from).squaredNorm() < 1e-12) continue;
            lit.merge(like.from);
            lit.merge(like.to);
            job.segments.push_back(like);
            pos = like.to;
        }
    }
    void dots(const Points& pts, bool closed, double spacing, Segment like)
    {
        like.kind = Segment::Kind::Dwell;
        const double step = std::max(spacing, 0.01);
        const size_t n    = closed ? pts.size() + 1 : pts.size();
        double total = 0;
        for (size_t i = 1; i < n; ++i) total += (unscale(pts[i % pts.size()]) - unscale(pts[i - 1])).norm();
        // A closed path does not repeat its first dot at the end.
        const double last = closed ? total - std::min(step, total) * 0.5 : total + 1e-9;
        double at = 0, next = 0;   // arc length of the edge start / of the next dot
        for (size_t i = 1; i < n; ++i) {
            const Vec2d a = unscale(pts[i - 1]), b = unscale(pts[i % pts.size()]);
            const double len = (b - a).norm();
            for (; next <= std::min(at + len + 1e-9, last); next += step) {
                const Vec2d p = len > 0 ? Vec2d(a + (b - a) * ((next - at) / len)) : a;
                travel_to(p, like);
                like.from = like.to = p;
                lit.merge(p);
                job.segments.push_back(like);
            }
            at += len;
        }
    }
    void scan(ScanLine sl, Segment like)
    {
        if (sl.runs.empty()) return;
        like.kind = Segment::Kind::Scan;
        travel_to(sl.start, like);
        const Vec2d d = (sl.end - sl.start).normalized();
        lit.merge(sl.start + d * sl.runs.front().x0);
        lit.merge(sl.start + d * sl.runs.back().x1);
        like.from = sl.start;
        like.to   = sl.end;
        like.scan = int(job.scans.size());
        job.scans.push_back(std::move(sl));
        job.segments.push_back(like);
        pos = like.to;
    }
};

// Leaf shapes (not groups) that burn: visible with every ancestor, on an output layer, selected
// when cut_selected_only.
std::vector<int> burnable_shapes(const LaserDocument& doc, const std::vector<int>& selection)
{
    const int n = int(doc.shapes.size());
    std::set<int> sel(selection.begin(), selection.end());
    auto in_selection = [&](int i) {
        for (int a = i, steps = 0; a >= 0 && steps <= n; a = doc.shapes[a].parent, ++steps)
            if (sel.count(a)) return true;
        return false;
    };
    std::vector<int> out;
    for (int i = 0; i < n; ++i) {
        const LaserShape& s = doc.shapes[i];
        if (s.type == ShapeType::Group || s.layer < 0 || s.layer >= kLayerCount) continue;
        const LaserLayer& l = doc.layers[s.layer];
        if (!l.output || !l.visible) continue;
        bool visible = true;
        for (int a = i, steps = 0; a >= 0 && steps <= n && visible; a = doc.shapes[a].parent, ++steps)
            visible = doc.shapes[a].visible;
        if (!visible || (doc.job.cut_selected_only && !in_selection(i))) continue;
        out.push_back(i);
    }
    return out;
}

// Burn units in job order: (layer, shapes). OrderBy::Groups makes one unit per top-level shape
// and layer; the others one per layer.
std::vector<std::pair<int, std::vector<int>>> job_units(const LaserDocument& doc, const std::vector<int>& shapes)
{
    std::vector<int> layer_order(kLayerCount);
    for (int i = 0; i < kLayerCount; ++i) layer_order[i] = i;
    if (doc.job.optimize.order_by != JobSettings::OrderBy::Layer)
        std::stable_sort(layer_order.begin(), layer_order.end(),
                         [&](int a, int b) { return doc.layers[a].priority < doc.layers[b].priority; });
    std::vector<std::pair<int, std::vector<int>>> units;
    auto add_layers = [&](const std::vector<int>& subset) {
        for (int l : layer_order) {
            std::vector<int> on;
            for (int i : subset)
                if (doc.shapes[i].layer == l) on.push_back(i);
            if (!on.empty()) units.emplace_back(l, std::move(on));
        }
    };
    if (doc.job.optimize.order_by == JobSettings::OrderBy::Groups) {
        std::map<int, std::vector<int>> by_root;   // root index -> shapes (document order of roots)
        for (int i : shapes) by_root[doc.root_of(i)].push_back(i);
        for (auto& [root, subset] : by_root) add_layers(subset);
    } else
        add_layers(shapes);
    return units;
}

// Fill regions of the unit's shapes per LaserLayer::fill_grouping.
std::vector<ExPolygons> fill_regions(const LaserDocument& doc, const std::vector<int>& shapes, FillGrouping grouping)
{
    std::map<int, LaserPaths> groups;   // key: 0 (all), shape index or root index
    for (int i : shapes) {
        if (doc.shapes[i].type == ShapeType::Image) continue;
        const int key = grouping == FillGrouping::AllAtOnce ? 0 : grouping == FillGrouping::PerShape ? i : doc.root_of(i);
        append(groups[key], doc.flatten(i));
    }
    std::vector<ExPolygons> out;
    for (auto& [k, paths] : groups) {
        ExPolygons r = to_expolygons(paths);
        if (!r.empty()) out.push_back(std::move(r));
    }
    return out;
}

} // namespace

LaserJob plan(const LaserDocument& doc, const LaserDevice& device, const std::vector<int>& selection, const ProgressFn& progress)
{
    LaserJob job;
    job.start_from = doc.job.start_from;
    if (device.type == DeviceType::Ruida) {
        job.error = "Ruida controllers are not supported yet. Choose a GRBL, grblHAL, Marlin or Smoothie device.";
        return job;
    }
    const std::vector<int> shapes = burnable_shapes(doc, selection);
    if (shapes.empty()) {
        job.error = doc.job.cut_selected_only ? "Nothing is selected to burn. Select shapes or turn off \"Cut selected graphics\"."
                                              : "Nothing to burn: add shapes on a layer with Output turned on.";
        return job;
    }
    auto cancelled = [&](double f) {
        if (progress && progress(f)) {
            job = LaserJob{};
            job.error = "Cancelled";
            return true;
        }
        return false;
    };

    Builder b{job, device, Vec2d(0, 0), BoundingBoxf()};
    const auto units = job_units(doc, shapes);
    for (size_t u = 0; u < units.size(); ++u) {
        if (cancelled(double(u) / units.size())) return job;
        const int         li = units[u].first;
        const LaserLayer& L  = doc.layers[li];
        const double speed   = device.max_speed_mm_s > 0 ? std::min(L.speed_mm_s, device.max_speed_mm_s) : L.speed_mm_s;
        Segment like;
        like.layer      = li;
        like.speed_mm_s = speed;
        like.power_pct  = std::clamp(L.power_max, 0., 100.);
        like.air_assist = L.air_assist;
        like.dwell_ms   = L.dot_dwell_ms;

        // Geometry that does not change between passes.
        std::vector<std::vector<ScanLine>> image_scans;
        LaserPaths vectors;
        for (int i : units[u].second) {
            const LaserShape& s = doc.shapes[i];
            if (s.type == ShapeType::Image) {
                const Raster r = prepare_image(s, L);
                if (r.w > 0) image_scans.push_back(raster_to_scanlines(r, L, device));
                else job.warnings.push_back("Image \"" + s.name + "\" has no pixels to engrave and was skipped.");
                continue;
            }
            if (s.type == ShapeType::Text && s.text_outlines.empty() && !s.text.empty())
                job.warnings.push_back("Text \"" + s.text + "\" has no outlines (its font is missing) and was skipped.");
            append(vectors, doc.flatten(i));
        }
        std::vector<ScanLine> fill_scans;
        LaserPaths            offset_rings;
        if (L.mode == LayerMode::Fill || L.mode == LayerMode::FillLine) {
            const double os = overscan_mm(L, speed);
            for (const ExPolygons& region : fill_regions(doc, units[u].second, L.fill_grouping))
                for (const Polyline& row : hatch_fill(region, L.interval_mm, L.angle_deg, L.crosshatch, L.bidirectional)) {
                    // One scan per hatch row with a single full-power run, so fills and images share
                    // the raster path (overscan, G-code, preview).
                    const Vec2d  p0 = unscale(row.first_point()), p1 = unscale(row.last_point());
                    const double len = (p1 - p0).norm();
                    if (len <= 0) continue;
                    const Vec2d d = (p1 - p0) / len;
                    ScanLine sl;
                    sl.start = p0 - d * os;
                    sl.end   = p1 + d * os;
                    sl.runs.push_back({float(os), float(os + len), float(like.power_pct)});
                    fill_scans.push_back(std::move(sl));
                }
        } else if (L.mode == LayerMode::OffsetFill)
            for (const ExPolygons& region : fill_regions(doc, units[u].second, L.fill_grouping))
                append(offset_rings, offset_fill(region, L.interval_mm));
        const bool line = L.mode == LayerMode::Line || L.mode == LayerMode::FillLine;
        if (line) vectors = kerf_offset(vectors, L.kerf_offset_mm);

        for (int pass = 0; pass < std::max(1, L.passes); ++pass) {
            like.z = L.z_offset_mm - pass * L.z_step_per_pass;
            for (const std::vector<ScanLine>& img : image_scans)
                for (const ScanLine& sl : img) b.scan(sl, like);
            for (const ScanLine& sl : fill_scans) b.scan(sl, like);
            // Offset fill: rings innermost first, as offset_fill() returns them.
            for (const LaserPath& p : offset_rings) b.cut_path(p.pts.points, p.closed, like);
            if (line && !vectors.empty()) {
                LaserPaths paths = vectors;
                optimize_order(paths, b.pos, doc.job.optimize);
                // Tabs and lead-ins both open a closed path: tabs win (they keep the part in place).
                if (L.tabs) paths = insert_tabs(paths, L.tab_count, L.tab_spacing_mm, L.tab_size_mm);
                else if (L.lead_in_mm > 0) paths = add_lead_in(paths, L.lead_in_mm);
                for (const LaserPath& p : paths) {
                    if (L.dot_mode) b.dots(p.pts.points, p.closed, L.dot_spacing_mm, like);
                    else b.cut_path(p.pts.points, p.closed, like);
                }
            }
        }
    }
    if (cancelled(1.0)) return job;
    if (job.segments.empty()) {
        job.error = "Nothing to burn: the shapes on the output layers have no geometry for their layer mode.";
        return job;
    }

    // ---- Job frame. Absolute: workspace = machine coordinates, the lit area must be on the bed.
    // UserOrigin / CurrentPosition: translate by -anchor, where the anchor is the job_origin point
    // of the lit bounds (or of the selection with use_selection_origin); 0 = rear-left .. 8 =
    // front-right in reading order: anchor = (min.x + col/2 * w, max.y - row/2 * h). The G-code
    // writer then flips axes for the machine's origin corner (to_machine). Rotary: Y distances
    // are mapped after the shift (LaserTypes.hpp), so Y = 0 stays the rotary's zero.
    Vec2d shift(0, 0);
    if (doc.job.start_from == JobSettings::StartFrom::Absolute) {
        const double eps = 1e-6;
        const bool off_bed = b.lit.min.x() < -eps || b.lit.max.x() > device.bed_w + eps ||
                             (!device.rotary.enabled && (b.lit.min.y() < -eps || b.lit.max.y() > device.bed_h + eps));
        if (off_bed) {
            job = LaserJob{};
            job.error = "Some shapes are outside the laser's work area. Move them onto the bed, or use \"User origin\" / \"Current position\".";
            return job;
        }
    } else {
        BoundingBoxf ab = b.lit;
        if (doc.job.use_selection_origin && !selection.empty()) {
            const BoundingBoxf sb = doc.bounds(selection);
            if (sb.defined) ab = sb;
        }
        const int k = std::clamp(doc.job.job_origin, 0, 8), row = k / 3, col = k % 3;
        shift = -Vec2d(ab.min.x() + col * 0.5 * (ab.max.x() - ab.min.x()), ab.max.y() - row * 0.5 * (ab.max.y() - ab.min.y()));
    }
    double ky = 1;
    if (device.rotary.enabled) {
        const RotarySettings& r = device.rotary;
        const double dia = r.type == RotaryType::Chuck ? r.object_diameter : r.roller_diameter;
        if (dia <= 0 || r.mm_per_rotation <= 0) {
            job = LaserJob{};
            job.error = "The rotary settings need a diameter and mm per rotation greater than zero.";
            return job;
        }
        ky = r.mm_per_rotation / (M_PI * dia);
    }
    auto map = [&](const Vec2d& p) { return Vec2d(p.x() + shift.x(), (p.y() + shift.y()) * ky); };
    const bool first_travel_from_origin = !job.segments.empty() && job.segments.front().kind == Segment::Kind::Travel;
    for (Segment& s : job.segments) {
        s.from = map(s.from);
        s.to   = map(s.to);
    }
    // Travel starts at the job-frame origin (the head's position for relative jobs).
    if (first_travel_from_origin) job.segments.front().from = Vec2d(0, 0);
    for (ScanLine& sl : job.scans) {
        const double old_len = (sl.end - sl.start).norm();
        sl.start = map(sl.start);
        sl.end   = map(sl.end);
        const double k = old_len > 0 ? (sl.end - sl.start).norm() / old_len : 1.;
        if (k != 1.)
            for (Run& r : sl.runs) { r.x0 = float(r.x0 * k); r.x1 = float(r.x1 * k); }
    }
    for (const Segment& s : job.segments)
        if (s.kind != Segment::Kind::Travel) {
            job.bounds.merge(s.from);
            job.bounds.merge(s.to);
        }
    estimate_time(job, device);
    return job;
}

void estimate_time(LaserJob& job, const LaserDevice& device)
{
    const double a = device.accel_mm_s2 > 0 ? device.accel_mm_s2 : 1000.;
    job.estimated_time_s = 0;
    job.layer_time_s.fill(0);
    for (const Segment& s : job.segments) {
        double t = 0;
        if (s.kind == Segment::Kind::Dwell)
            t = std::max(0., s.dwell_ms) / 1000.;
        else {
            double v = s.speed_mm_s;
            const double cap = s.kind == Segment::Kind::Travel ? device.travel_speed_mm_s : device.max_speed_mm_s;
            if (cap > 0) v = v > 0 ? std::min(v, cap) : cap;
            const double L = (s.to - s.from).norm();
            if (v > 0 && L > 0)
                // Trapezoid from rest to rest: accelerate, cruise, decelerate; a triangle when short.
                t = L >= v * v / a ? L / v + v / a : 2. * std::sqrt(L / a);
        }
        if (!std::isfinite(t)) t = 0;
        job.estimated_time_s += t;
        if (s.layer >= 0 && s.layer < kLayerCount) job.layer_time_s[s.layer] += t;
    }
}

} // namespace Slic3r::Laser
