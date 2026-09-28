#include "libslic3r/CAM/FeedsSpeeds.hpp"

#include <algorithm>
#include <cmath>

namespace Slic3r::CAM {

namespace {

// Conservative hobby-machine starting points (router / small mill), per material.
struct MaterialData {
    const char* name;
    double      vc_carbide;   // surface speed, m/min, carbide tool
    double      hss_factor;   // HSS surface speed = vc_carbide * hss_factor
    double      chipload[5];  // mm/tooth for D < 3, 3-6, 6-10, 10-16, > 16 mm
    double      plunge;       // plunge feed as a fraction of the cutting feed (0.3 - 0.5)
};

constexpr MaterialData kMaterials[kMaterialCount] = {
    {"Aluminum",        200, 0.30, {0.015, 0.025, 0.040, 0.060, 0.080}, 0.30},
    {"Brass",           150, 0.35, {0.015, 0.025, 0.040, 0.060, 0.075}, 0.35},
    {"Mild steel",       90, 0.30, {0.010, 0.020, 0.030, 0.045, 0.060}, 0.30},
    {"Stainless steel",  60, 0.25, {0.008, 0.015, 0.025, 0.035, 0.050}, 0.30},
    {"Plastic",         250, 0.50, {0.030, 0.060, 0.100, 0.130, 0.150}, 0.50},
    {"Acrylic",         250, 0.50, {0.030, 0.050, 0.080, 0.100, 0.130}, 0.40},
    {"HDPE",            300, 0.50, {0.050, 0.080, 0.120, 0.150, 0.200}, 0.50},
    {"Hardwood",        350, 0.50, {0.040, 0.080, 0.130, 0.180, 0.230}, 0.50},
    {"Softwood",        400, 0.50, {0.050, 0.100, 0.150, 0.200, 0.250}, 0.50},
    {"Plywood",         350, 0.50, {0.040, 0.080, 0.120, 0.160, 0.200}, 0.50},
    {"MDF",             350, 0.50, {0.050, 0.100, 0.150, 0.200, 0.250}, 0.50},
    {"Foam",            500, 0.60, {0.100, 0.200, 0.300, 0.400, 0.500}, 0.50},
    {"Wax",             300, 0.60, {0.050, 0.100, 0.150, 0.200, 0.250}, 0.50},
};

int diameter_class(double d) { return d < 3 ? 0 : d < 6 ? 1 : d < 10 ? 2 : d < 16 ? 3 : 4; }

// ISO metric coarse pitch for a tap without a pitch set.
double coarse_pitch(double d)
{
    static constexpr double table[][2] = {{2, 0.4}, {2.5, 0.45}, {3, 0.5}, {4, 0.7}, {5, 0.8}, {6, 1.0},
                                          {8, 1.25}, {10, 1.5}, {12, 1.75}, {16, 2.0}, {20, 2.5}};
    double best = table[0][1], err = 1e9;
    for (const auto& row : table)
        if (std::abs(row[0] - d) < err) {
            err  = std::abs(row[0] - d);
            best = row[1];
        }
    return best;
}

double clamp_max(double v, double hi) { return hi > 0 ? std::min(v, hi) : v; }

} // namespace

const char* material_name(Material m)
{
    const int i = int(m);
    return i >= 0 && i < kMaterialCount ? kMaterials[i].name : "Unknown";
}

FeedsSpeeds recommend_feeds(const CamTool& tool, Material material, const MachineProfile& machine)
{
    const MaterialData& md = kMaterials[std::clamp(int(material), 0, kMaterialCount - 1)];
    const bool          drill = tool.type == ToolType::Drill || tool.type == ToolType::SpotDrill;
    const bool          tap   = tool.type == ToolType::Tap;
    const bool          vee   = tool.type == ToolType::VBit || tool.type == ToolType::ChamferMill;
    // V-bits and chamfer mills cut well below their widest diameter.
    const double d  = std::max(0.1, vee ? std::max(tool.tip_diameter, 0.5 * tool.diameter) : tool.diameter);
    double       vc = md.vc_carbide * (tool.material == ToolMaterial::HSS ? md.hss_factor : 1.0);
    if (drill)
        vc *= 0.6;
    if (tap)
        vc *= 0.15;

    double rpm = vc * 1000.0 / (M_PI * d);
    if (tap)
        rpm = std::min(rpm, 500.0);   // hobby tapping: slow, even under power
    rpm = std::max(clamp_max(rpm, machine.max_rpm), machine.min_rpm);

    FeedsSpeeds fs;
    fs.rpm           = std::round(rpm);
    fs.chipload      = md.chipload[diameter_class(d)] * (vee ? 0.5 : 1.0);
    fs.surface_speed = fs.rpm * M_PI * d / 1000.0;
    if (tap) {
        const double pitch = tool.thread_pitch > 0 ? tool.thread_pitch : coarse_pitch(tool.diameter);
        fs.chipload    = pitch;
        fs.feed        = fs.rpm * pitch;   // synchronised: must not be clamped below this
        fs.plunge_feed = fs.feed;
        fs.ramp_feed   = fs.feed;
        return fs;
    }
    const int flutes = drill ? 2 : std::max(1, tool.flutes);
    const double feed = fs.rpm * flutes * fs.chipload;
    if (drill) {
        // A drill's feed is its Z feed.
        fs.feed = fs.plunge_feed = fs.ramp_feed = std::round(clamp_max(feed, machine.max_feed_z));
        return fs;
    }
    fs.feed        = std::round(clamp_max(feed, machine.max_feed_xy));
    fs.plunge_feed = std::round(clamp_max(fs.feed * md.plunge, machine.max_feed_z));
    fs.ramp_feed   = std::round(clamp_max(fs.feed * 0.5, machine.max_feed_xy));
    return fs;
}

FeedsSpeeds effective_feeds(const CamOperation& op, const CamTool& tool, Material material, const MachineProfile& machine)
{
    if (!op.feeds_auto)
        return op.feeds;
    if (tool.override_feeds)
        return tool.feeds;
    return recommend_feeds(tool, material, machine);
}

} // namespace Slic3r::CAM
