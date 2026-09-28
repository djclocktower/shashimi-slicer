#include "libslic3r/Laser/LaserGCode.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>

namespace Slic3r::Laser {

namespace {

bool relative(JobSettings::StartFrom f) { return f != JobSettings::StartFrom::Absolute; }

// Axis flips for the origin corner. Rotary jobs keep Y as planned (it is the rotary axis).
Vec2d flip(const Vec2d& p, const LaserDevice& d, JobSettings::StartFrom frame)
{
    const bool rear  = d.origin_corner == OriginCorner::RearLeft || d.origin_corner == OriginCorner::RearRight;
    const bool right = d.origin_corner == OriginCorner::FrontRight || d.origin_corner == OriginCorner::RearRight;
    Vec2d q = p;
    if (right) q.x() = relative(frame) ? -p.x() : d.bed_w - p.x();
    if (rear && !d.rotary.enabled) q.y() = relative(frame) ? -p.y() : d.bed_h - p.y();
    return q;
}

std::string num(double v, int decimals)
{
    if (!std::isfinite(v)) v = 0;
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*f", std::clamp(decimals, 0, 6), v);
    std::string s = buf;
    if (s.find('.') != std::string::npos) {
        s.erase(s.find_last_not_of('0') + 1);
        if (s.back() == '.') s.pop_back();
    }
    if (s == "-0") s = "0";
    return s;
}

// Emits moves with modal X/Y/Z/F/S: a word is written only when it changes (unless !modal).
class Writer {
public:
    Writer(const LaserDevice& d, JobSettings::StartFrom frame, const GCodeOptions& o) : dev(d), frame(frame), opts(o) {}

    std::ostringstream out;

    bool marlin() const { return dev.type == DeviceType::Marlin; }
    bool smoothie() const { return dev.type == DeviceType::Smoothie; }
    bool fan() const { return marlin() && dev.marlin_fan_laser; }
    // Power travels as an S word on G1 (GRBL, grblHAL, Smoothie, Marlin inline) or as M3/M4/M106.
    bool inline_power() const { return !marlin() || dev.marlin_inline; }
    bool dynamic() const { return dev.laser_mode_dynamic; }

    std::string s_word(double pct) const
    {
        pct = std::clamp(std::isfinite(pct) ? pct : 0., 0., 100.);
        if (smoothie()) return num(pct / 100., 3);
        return num(std::round(pct * (fan() ? 255. : dev.s_max) / 100.), 0);
    }

    void line(const std::string& s) { out << s << '\n'; }
    void comment(const std::string& s)
    {
        if (opts.comments) out << "; " << s << '\n';
    }

    void header(const LaserJob* job)
    {
        if (job && opts.comments) {
            comment("Shashimi Laser job: " + opts.job_name);
            comment("Device: " + dev.name);
            const int t = int(std::lround(job->estimated_time_s));
            char buf[64];
            std::snprintf(buf, sizeof(buf), "Estimated time: %d:%02d:%02d", t / 3600, t / 60 % 60, t % 60);
            comment(buf);
            if (job->bounds.defined)
                comment("Bounds: X" + num(job->bounds.min.x(), 2) + " Y" + num(job->bounds.min.y(), 2) + " to X" +
                        num(job->bounds.max.x(), 2) + " Y" + num(job->bounds.max.y(), 2));
        }
        line(smoothie() || marlin() ? "G21\nG90" : "G21 G90 G54");
        if (frame == JobSettings::StartFrom::CurrentPosition) line("G92 X0 Y0");
        laser_ready();
    }
    // The laser armed at zero power (GRBL M4 S0 / M3 S0, Marlin inline M3 I / M4 I).
    void laser_ready()
    {
        if (smoothie() || fan()) return;
        if (marlin()) {
            if (dev.marlin_inline) line(std::string(dynamic() ? "M4" : "M3") + " I");
            armed = dev.marlin_inline;
            return;
        }
        line(std::string(dynamic() ? "M4" : "M3") + " S0");
        armed = true;
        s.clear();
    }
    void footer()
    {
        if (fan()) line("M107");
        else if (!smoothie()) line("M5");
        on = false;
        if (air) line(dev.air_off_gcode);
        air = false;
        if (!dev.end_gcode.empty()) line(dev.end_gcode);
        if (frame == JobSettings::StartFrom::CurrentPosition) line("G92.1");
        if (!marlin()) line("M2");
    }

    void set_air(bool on)
    {
        if (on == air) return;
        const std::string& g = on ? dev.air_on_gcode : dev.air_off_gcode;
        if (!g.empty()) line(g);
        air = on;
    }
    void set_z(double z)
    {
        if (!dev.enable_z || (have_z && std::abs(z - cur_z) < 1e-9)) return;
        line("G0 Z" + num(z, opts.decimals));
        cur_z  = z;
        have_z = true;
    }

    // Separate-command power (Marlin M3/M4/M106): switch before a lit move, off before travel.
    void laser_power(double pct)
    {
        if (inline_power()) return;
        const std::string sw = s_word(pct);
        if (on && sw == ext_s) return;
        line(fan() ? "M106 S" + sw : std::string(dynamic() ? "M4" : "M3") + " S" + sw);
        on    = true;
        ext_s = sw;
    }
    void laser_off()
    {
        if (inline_power() || !on) return;
        line(fan() ? "M107" : "M5");
        on = false;
        ext_s.clear();
    }

    void travel(const Vec2d& p, double speed)
    {
        laser_off();
        if (dev.uses_g0_for_travel) {
            // GRBL laser mode switches the laser off on G0 only in M4; M3 needs S0 on the move.
            const bool need_s0 = inline_power() && !smoothie() && !dynamic() && !marlin();
            move("G0", p, -1, need_s0 ? 0. : -1.);
        } else
            move("G1", p, speed, inline_power() ? 0. : -1.);
    }
    void burn(const Vec2d& p, double speed, double pct)
    {
        if (!inline_power()) {
            laser_power(pct);
            move("G1", p, speed, -1);
        } else {
            if (marlin() && !armed) laser_ready();
            if (!marlin() && !smoothie() && !armed) laser_ready();
            move("G1", p, speed, pct);
        }
    }
    void dwell(const Vec2d& p, double ms, double pct)
    {
        (void) p;
        // Firing in place: GRBL / Marlin keep the laser on while stopped only in M3.
        if (smoothie()) {
            line("G1 S" + s_word(pct));   // Smoothie: power word only, no motion
            line("G4 P" + num(ms, 0));
            line("G1 S0");
            s = "0";
            return;
        }
        laser_off();
        line((fan() ? "M106 S" : "M3 S") + s_word(pct));
        line(marlin() ? "G4 P" + num(ms, 0) : "G4 P" + num(ms / 1000., 3));
        line(fan() ? "M107" : "M5");
        armed = false;
        s.clear();
    }

private:
    // speed < 0: no F; pct < 0: no S.
    void move(const char* g, const Vec2d& p_job, double speed, double pct)
    {
        const Vec2d p = flip(p_job, dev, frame);
        std::string l = g;
        const std::string x = num(p.x(), opts.decimals), y = num(p.y(), opts.decimals);
        const bool always = !opts.modal_fs;
        if (x != cur_x || always) l += " X" + x;
        if (y != cur_y || always) l += " Y" + y;
        const bool moved = x != cur_x || y != cur_y;
        cur_x = x;
        cur_y = y;
        if (pct >= 0) {
            const std::string sw = s_word(pct);
            if (sw != s || always) l += " S" + sw;
            s = sw;
        }
        if (speed > 0) {
            const std::string f = num(speed * 60., 0);
            if (f != cur_f || always) l += " F" + f;
            cur_f = f;
        }
        if (moved || l.size() > 2) line(l);
    }

    const LaserDevice&     dev;
    JobSettings::StartFrom frame;
    const GCodeOptions&    opts;
    std::string            cur_x, cur_y, cur_f, s, ext_s;
    double                 cur_z{0};
    bool                   have_z{false}, air{false}, on{false}, armed{false};
};

bool finite(const Vec2d& p) { return std::isfinite(p.x()) && std::isfinite(p.y()); }

} // namespace

Vec2d to_machine(const Vec2d& p, const LaserDevice& device, JobSettings::StartFrom frame) { return flip(p, device, frame); }
Vec2d from_machine(const Vec2d& p, const LaserDevice& device, JobSettings::StartFrom frame) { return flip(p, device, frame); }

std::string gcode(const LaserJob& job, const LaserDevice& device, const GCodeOptions& opts, std::string* error)
{
    auto fail = [&](const std::string& msg) {
        if (error) *error = msg;
        return std::string();
    };
    if (device.type == DeviceType::Ruida) return fail("Ruida controllers are not supported yet.");
    if (!job.ok()) return fail(job.error);
    if (job.segments.empty()) return fail("The job is empty.");
    for (const Segment& s : job.segments)
        if (!finite(s.from) || !finite(s.to) || !std::isfinite(s.speed_mm_s) || !std::isfinite(s.z))
            return fail("The job contains invalid coordinates; it was not written.");

    Writer w(device, job.start_from, opts);
    w.header(&job);
    if (!device.start_gcode.empty()) w.line(device.start_gcode);
    int layer = -2;
    for (const Segment& seg : job.segments) {
        if (seg.layer != layer) {
            layer = seg.layer;
            if (layer >= 0) {
                char buf[16];
                std::snprintf(buf, sizeof(buf), "C%02d", layer);
                w.comment(std::string("Layer ") + buf);
            }
        }
        w.set_air(seg.air_assist);
        w.set_z(seg.z);
        switch (seg.kind) {
        case Segment::Kind::Travel: w.travel(seg.to, seg.speed_mm_s); break;
        case Segment::Kind::Cut: w.burn(seg.to, seg.speed_mm_s, seg.power_pct); break;
        case Segment::Kind::Dwell: w.dwell(seg.to, seg.dwell_ms, seg.power_pct); break;
        case Segment::Kind::Scan: {
            if (seg.scan < 0 || seg.scan >= int(job.scans.size())) break;
            const ScanLine& sl  = job.scans[seg.scan];
            const double    len = (sl.end - sl.start).norm();
            if (len <= 0) break;
            const Vec2d d = (sl.end - sl.start) / len;
            // Dark stretches (overscan, gaps) at S0, each run at its power.
            float at = 0;
            for (const Run& r : sl.runs) {
                if (r.x0 > at + 1e-6f) w.burn(sl.start + d * double(r.x0), seg.speed_mm_s, 0);
                w.burn(sl.start + d * double(r.x1), seg.speed_mm_s, r.power_pct);
                at = r.x1;
            }
            if (len > at + 1e-6) w.burn(sl.end, seg.speed_mm_s, 0);
            break;
        }
        }
    }
    w.footer();
    if (error) error->clear();
    return w.out.str();
}

std::string frame_gcode(const LaserJob& job, const LaserDevice& device, FrameMode mode)
{
    if (!job.ok() || !job.bounds.defined || device.type == DeviceType::Ruida) return {};
    std::vector<Vec2d> loop;
    if (mode == FrameMode::Rect) {
        const BoundingBoxf& b = job.bounds;
        loop = {b.min, {b.max.x(), b.min.y()}, b.max, {b.min.x(), b.max.y()}};
    } else {
        // Convex hull (monotone chain) of every lit endpoint.
        std::vector<Vec2d> pts;
        for (const Segment& s : job.segments)
            if (s.kind != Segment::Kind::Travel) {
                pts.push_back(s.from);
                pts.push_back(s.to);
            }
        std::sort(pts.begin(), pts.end(), [](const Vec2d& a, const Vec2d& b) { return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y()); });
        auto cross = [](const Vec2d& o, const Vec2d& a, const Vec2d& b) {
            return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
        };
        std::vector<Vec2d> h(2 * pts.size());
        size_t k = 0;
        for (size_t i = 0; i < pts.size(); ++i) {
            while (k >= 2 && cross(h[k - 2], h[k - 1], pts[i]) <= 0) --k;
            h[k++] = pts[i];
        }
        for (size_t i = pts.size() - 1, t = k + 1; i > 0; --i) {
            while (k >= t && cross(h[k - 2], h[k - 1], pts[i - 1]) <= 0) --k;
            h[k++] = pts[i - 1];
        }
        h.resize(k > 1 ? k - 1 : k);
        loop = std::move(h);
    }
    if (loop.empty()) return {};

    GCodeOptions opts;
    opts.comments = false;
    Writer w(device, job.start_from, opts);
    w.header(nullptr);
    const double v = device.travel_speed_mm_s > 0 ? device.travel_speed_mm_s : 50;
    const bool   lit = device.frame_power_pct > 0;
    w.travel(loop.front(), v);
    for (size_t i = 1; i <= loop.size(); ++i) {
        const Vec2d& p = loop[i % loop.size()];
        // Laser off: G1 at S0 keeps the head on the traced line at the framing speed.
        w.burn(p, v, lit ? device.frame_power_pct : 0.);
    }
    w.travel(Vec2d(0, 0), v);   // back to the job-frame origin
    w.footer();
    return w.out.str();
}

} // namespace Slic3r::Laser
