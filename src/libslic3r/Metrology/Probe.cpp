#include "Probe.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace Slic3r::Metrology {

P2 mat_to_printer(const P2& p, const ProbeSettings& s)
{
    const double a = s.mat_rot_deg * kPi / 180.0;
    return {s.mat_x + p.x * std::cos(a) - p.y * std::sin(a), s.mat_y + p.x * std::sin(a) + p.y * std::cos(a)};
}

static void header(std::ostringstream& g, const char* what)
{
    g << "; Shashimi Measure: " << what << "\n"
      << "; UNTESTED on hardware. Run it first on an empty plate with the filament unloaded and\n"
      << "; the nozzle cold and clean, watch the first touch and be ready to stop the print.\n"
      << "; G380 is Bambu's guarded move (G38): it stops when the nozzle meets the surface.\n"
      << "G28                  ; home\n"
      << "M104 S0              ; keep the nozzle cold\n"
      << "M106 S0              ; part fan off\n"
      << "G90\n";
}

static void touch(std::ostringstream& g, const P2& xy, double travel_z, double search, const ProbeSettings& s, const std::string& label,
                  int index)
{
    g << "; touch " << index << ": " << label << "\n";
    g << "G1 Z" << travel_z << " F1200\n";
    g << "G1 X" << xy.x << " Y" << xy.y << " F" << s.feed_xy << "\n";
    g << "G91\n";
    g << "G380 S3 Z-" << search << " F" << s.probe_feed << "\n";
    g << "M400 S" << int(std::ceil(s.hold_s)) << "\n";
    g << "G1 Z" << s.backoff << " F1200\n";
    g << "G90\n";
}

std::string probe_gcode(const std::vector<ProbeTouch>& touches, const ProbeSettings& s, double z_min, double z_max, const P2& mat_ref_xy)
{
    std::ostringstream g;
    g << std::fixed << std::setprecision(3);
    header(g, "probe batch");
    // Heights are above the mat's top face; the printer's Z is above the plate.
    const double base     = s.mat_thickness;
    const double travel_z = base + std::max(0.0, z_max) + s.clearance;
    const double search   = travel_z - (base + std::min(0.0, z_min)) + s.overtravel;
    g << "; " << touches.size() + 1 << " touches, travel Z " << travel_z << ", search " << search << " mm\n";
    touch(g, mat_to_printer(mat_ref_xy, s), travel_z, search, s, "mat reference", 0);
    for (size_t i = 0; i < touches.size(); ++i) touch(g, mat_to_printer(touches[i].xy, s), travel_z, search, s, touches[i].label, int(i + 1));
    g << "G1 Z" << travel_z + 10 << " F1200\n";
    return g.str();
}

std::string repeatability_gcode(const P2& printer_xy, int n, const ProbeSettings& s)
{
    std::ostringstream g;
    g << std::fixed << std::setprecision(3);
    header(g, "repeatability test");
    for (int i = 0; i < std::max(1, n); ++i) touch(g, printer_xy, 10.0, 12.0, s, "repeat", i + 1);
    g << "G1 Z20 F1200\n";
    return g.str();
}

std::vector<StillSegment> find_stills(const std::vector<double>& motion, double threshold, int min_frames)
{
    std::vector<StillSegment> out;
    int start = -1;
    for (int i = 0; i <= int(motion.size()); ++i) {
        const bool still = i < int(motion.size()) && motion[size_t(i)] < threshold;
        if (still && start < 0) start = i;
        if (!still && start >= 0) {
            // motion[i] compares frames i and i+1, so a still run of motions [start, i) spans frames start..i.
            if (i - start + 1 >= min_frames) out.push_back({start, i});
            start = -1;
        }
    }
    return out;
}

RepeatStats repeat_stats(const std::vector<double>& v)
{
    RepeatStats r;
    r.n = int(v.size());
    if (v.empty()) return r;
    double s = 0;
    for (double x : v) s += x;
    r.mean = s / v.size();
    double ss = 0;
    for (double x : v) ss += (x - r.mean) * (x - r.mean);
    r.stddev = v.size() > 1 ? std::sqrt(ss / (v.size() - 1)) : 0.0;
    const auto mm = std::minmax_element(v.begin(), v.end());
    r.range = *mm.second - *mm.first;
    return r;
}

} // namespace Slic3r::Metrology
